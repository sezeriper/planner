#pragma once

#include "../utils.hpp"
#include "../dubins/dubins.hpp"
#include "../math.hpp"
#include "../messages.hpp"

#include <condition_variable>
#include <mutex>
#include <spdlog/spdlog.h>
#include <mavsdk.h>
#include <plugins/action/action.h>
#include <plugins/offboard/offboard.h>
#include <plugins/telemetry/telemetry.h>

#include <atomic>
#include <cmath>
#include <cstddef>
#include <thread>
#include <vector>

namespace rota {

using namespace messages;

class plane_control {
public:
    plane_control() : _mavsdk{mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::CompanionComputer}} {}
    bool init(const std::string& connection_string)
    {
        auto connection_result = _mavsdk.add_any_connection(connection_string);

        if (connection_result != mavsdk::ConnectionResult::Success) {
            spdlog::error("Connection failed: {}.", static_cast<int>(connection_result));
            return false;
        }

        auto system = _mavsdk.first_autopilot(3.0);
        if (!system) {
            spdlog::error("No system found.");
            return false;
        }

        _action = std::make_unique<mavsdk::Action>(system.value());
        _offboard = std::make_unique<mavsdk::Offboard>(system.value());
        _telemetry = std::make_unique<mavsdk::Telemetry>(system.value());

        while (!_telemetry->health_all_ok()) {
            spdlog::info("Waiting for system to be ready.");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        _telemetry->subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode flight_mode) {
            if (flight_mode == mavsdk::Telemetry::FlightMode::Offboard) {
                spdlog::info("Flight mode switched to Offboard.");
            }
        });

        _telemetry->set_rate_position_velocity_ned(60.0);
        _telemetry->subscribe_position_velocity_ned([this](mavsdk::Telemetry::PositionVelocityNed position_velocity_ned) {
            _position.write([&position_velocity_ned](position_t& pos) {
                pos = {
                    static_cast<real_t>(position_velocity_ned.position.north_m),
                    static_cast<real_t>(position_velocity_ned.position.east_m),
                    static_cast<real_t>(position_velocity_ned.position.down_m)
                };
            });
        });

        _telemetry->set_rate_unix_epoch_time(60.0);

        spdlog::info("Plane Controller deady.");
        return true;
    }

    void start_offboard() {
        if (_offboard->is_active()) {
            return;
        }
        mavsdk::Offboard::Result result = _offboard->start();
        if (result != mavsdk::Offboard::Result::Success) {
            spdlog::error("Offboard start failed: {}.", static_cast<int>(result));
        }
    }

    position_t get_position() const {
        return _position.read([](const position_t& pos) {
            return pos;
        });
    }

    std::uint64_t get_unix_epoch_time() const {
        return _telemetry->unix_epoch_time();
    }

    void set_configuration(configuration_t conf) {
        auto ned = xyz_to_ned(conf.x, 50.0f, conf.y);
        // mavsdk::Offboard::VelocityNedYaw vel_ned_yaw {15.0f, 0.0f, 0.0f, 0.0f};
        mavsdk::Offboard::PositionNedYaw pos_ned_yaw {ned.x, ned.y, ned.z, 0.0f};
        _offboard->set_position_ned(pos_ned_yaw);
        // spdlog::info("Setting position to: ({}, {}, {}, {})", ned.x, ned.y, ned.z, rad_to_deg(conf.yaw));
    }

    vec3_t get_position_xyz() {
        auto pos = get_position();
        return ned_to_xyz(pos.north, pos.east, pos.down);
    }

private:
    mavsdk::Mavsdk _mavsdk;
    std::unique_ptr<mavsdk::Action> _action;
    std::unique_ptr<mavsdk::Offboard> _offboard;
    std::unique_ptr<mavsdk::Telemetry> _telemetry;

    mutex_guarded<position_t> _position;
};

class path_tracker {
    using paths_t = std::vector<DubinsPath>;
    static constexpr int DELAY_MS = 100;
public:
    path_tracker(plane_control& plane_controller) :
        _plane_control{plane_controller},
        _is_running{true},
        _got_work{false},
        _thread(std::bind([this]() { thread_func(); }))
    {}

    ~path_tracker() {
        _is_running = false;
        _cv.notify_one();
        _thread.join();
    }

    void set_path(const paths_t& new_paths) {
        _paths.write([&new_paths](paths_t& old_p) {
            old_p = new_paths;
        });

        {
            std::unique_lock<std::mutex> lock(_mutex);
            _got_work = true;
            _cv.notify_one();
        }
        _plane_control.start_offboard();
    }

private:
    plane_control& _plane_control;
    std::atomic_bool _is_running;
    bool _got_work;
    std::mutex _mutex;
    std::condition_variable _cv;
    mutex_guarded<paths_t> _paths;
    std::thread _thread;

    void thread_func() {
        while (_is_running) {
            {
                std::unique_lock<std::mutex> lock(_mutex);
                _cv.wait(lock, [this]() { return _got_work; });
            }

            {
                std::unique_lock<std::mutex> lock(_mutex);
                _got_work = false;
            }

            auto path = _paths.read([](const paths_t& paths) {
                return paths;
            })[0];
            auto traj = sample_path_to_end(path);

            while (_is_running) {
                {
                    std::unique_lock<std::mutex> lock(_mutex);
                    if (_got_work) {
                        break;
                    }
                }

                const vec3_t vehicle_pos = _plane_control.get_position_xyz();
                bool found = false;
                vec2_t target;
                real_t yaw;
                for (std::size_t i = traj.size()-1; i > 0; --i)
                {
                    const auto c1 = traj[i-1];
                    const auto c2 = traj[i];

                    circle_t circle{ {vehicle_pos.x, vehicle_pos.z}, 10.0f};
                    line_t line { c1.get_point(), c2.get_point() };

                    const auto intersection = compute_intersection(line, circle);
                    if (intersection.solution_case == intersection_t::solution_case_t::ONE_SOLUTION)
                    {
                        target = intersection.p1;

                        // interpolate yaw
                        auto local_target = subtract(target, c1.get_point());
                        real_t mag = magnitude(local_target);
                        yaw = std::lerp(c1.yaw, c2.yaw, mag / distance(c1.get_point(), c2.get_point()));

                        found = true;
                        break;
                    }
                    else if (intersection.solution_case == intersection_t::solution_case_t::TWO_SOLUTIONS)
                    {
                        real_t dist1 = distance_sqr(intersection.p1, c2.get_point());
                        real_t dist2 = distance_sqr(intersection.p2, c2.get_point());

                        if (dist1 < dist2) {
                            target = intersection.p1;
                        } else {
                            target = intersection.p2;
                        }

                        // interpolate yaw
                        auto local_target = subtract(target, c1.get_point());
                        real_t mag = magnitude(local_target);
                        yaw = std::lerp(c1.yaw, c2.yaw, mag / distance(c1.get_point(), c2.get_point()));

                        found = true;
                        break;
                    }
                }
                if (found) {
                    _plane_control.set_configuration({target.x, target.y, yaw});
                    std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_MS));
                }
                else {
                    spdlog::info("No intersection found.");
                    break;
                }
            }
        }
    }
};
}
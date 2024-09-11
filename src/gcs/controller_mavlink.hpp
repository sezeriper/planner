#pragma once

#include "../math.hpp"
#include "../utils.hpp"
#include "../messages.hpp"

#include <mavsdk.h>
#include <plugins/action/action.h>
#include <plugins/telemetry/telemetry.h>
#include <plugins/offboard/offboard.h>
#include <geometry.h>
#include <spdlog/spdlog.h>

#include <queue>


namespace rota {

using namespace messages;

class controller_mavlink {
public:

    controller_mavlink() :
        _mavsdk{mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::GroundStation}}
    {}

    bool init_mavsdk(const std::string& mavsdk_connection_string) {
        bool result = setup_mavsdk(mavsdk_connection_string);

        if (!result) return false;

        setup_mavsdk_subscription();

        return true;
    }

    void do_takeoff() {
        _action->takeoff_async(std::bind(action_handler, std::placeholders::_1, "Takeoff"));
    }

    void do_arm() {
        _action->arm_async(std::bind(action_handler, std::placeholders::_1, "Arm"));
    }
    
    void do_land() {
        _action->land_async(std::bind(action_handler, std::placeholders::_1, "Land"));
    }

    void do_hold() {
        _action->hold_async(std::bind(action_handler, std::placeholders::_1, "Hold"));
    }

    void do_offboard() {
        auto result = _offboard->start();
        if (result != mavsdk::Offboard::Result::Success) {
            spdlog::error("Offboard start failed: {}.", static_cast<int>(result));
        }
    }

    coords_t get_coords() {
        return _data_coords.read([](const coords_t& coords) {
            return coords;
        });
    }

    position_t get_position() {
        return _data_position.read([](const position_t& pos) {
            return pos;
        });
    }

    vec3_t get_position_xyz() {
        auto pos = get_position();
        return ned_to_xyz(pos.north, pos.east, pos.down);
    }

    attitude_t get_attitude() {
        return _data_attitude.read([](const attitude_t& att) {
            return att;
        });
    }

    heading_t get_heading() {
        return _data_heading.read([](const heading_t& head) {
            return deg_to_rad(head);
        });
    }

    distance_sensor_t get_distance_sensor() {
        return _data_distance_sensor.read([](const distance_sensor_t& dist) {
            return dist;
        });
    }

    battery_t get_battery() {
        return _data_battery.read([](const battery_t& bat) {
            return bat;
        });
    }

    metrics_t get_metrics() {
        return _data_metrics.read([](const metrics_t& met) {
            return met;
        });
    }

    gps_origin_t get_gps_origin() {
        return _data_gps_origin.read([](const gps_origin_t& orig) {
            return orig;
        });
    }

    bool is_armed() {
        return _data_is_armed.read([](const bool& armed) {
            return armed;
        });
    }

    flight_mode_t get_flight_mode() {
        return _data_flight_mode.read([](const flight_mode_t& fm) {
            return fm;
        });
    }

    std::queue<vec3_t> get_position_history() {
        return _position_history.read([](const std::queue<vec3_t>& history) {
            return history;
        });
    }

private:
    static constexpr std::size_t MAX_POSITION_HISTORY_LENGTH = 4096;

    mavsdk::Mavsdk _mavsdk;
    std::unique_ptr<mavsdk::Action> _action;
    std::unique_ptr<mavsdk::Telemetry> _telemetry;
    std::unique_ptr<mavsdk::Offboard> _offboard;

    mutex_guarded<coords_t> _data_coords;
    mutex_guarded<position_t> _data_position;
    mutex_guarded<attitude_t> _data_attitude;
    mutex_guarded<heading_t> _data_heading;
    mutex_guarded<distance_sensor_t> _data_distance_sensor;
    mutex_guarded<battery_t> _data_battery;
    mutex_guarded<metrics_t> _data_metrics;
    mutex_guarded<gps_origin_t> _data_gps_origin;
    mutex_guarded<bool> _data_is_armed;
    mutex_guarded<flight_mode_t> _data_flight_mode;

    mutex_guarded<std::queue<vec3_t>> _position_history;

    static constexpr vec3_t ned_to_xyz(real_t north, real_t east, real_t down) {
        return {north, -down, east};
    }

    static void action_handler(mavsdk::Action::Result result, const std::string& action_name) {
        if (result == mavsdk::Action::Result::Success) {
            spdlog::info("Action {} succeeded.", action_name);
        } else {
            spdlog::error("Action {} failed.", action_name);
        }
    }

    bool setup_mavsdk(const std::string& mavsdk_connection_string) {
        auto connection_result = _mavsdk.add_any_connection(mavsdk_connection_string);

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
        _telemetry = std::make_unique<mavsdk::Telemetry>(system.value());
        _offboard = std::make_unique<mavsdk::Offboard>(system.value());

        while (!_telemetry->health_all_ok()) {
            spdlog::info("Waiting for system to be ready.");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        spdlog::info("Plane Controller deady.");

        return true;
    }

    void setup_mavsdk_subscription() {
        _telemetry->subscribe_position([this](mavsdk::Telemetry::Position position) {
            _data_coords.write([&position](coords_t& coords) {
                coords = {
                    static_cast<real_t>(position.latitude_deg),
                    static_cast<real_t>(position.longitude_deg),
                    static_cast<real_t>(position.relative_altitude_m)
                };
            });
        });

        _telemetry->subscribe_position_velocity_ned([this](mavsdk::Telemetry::PositionVelocityNed position_velocity_ned) {
            _data_position.write([&position_velocity_ned](position_t& pos) {
                pos = {
                    static_cast<real_t>(position_velocity_ned.position.north_m),
                    static_cast<real_t>(position_velocity_ned.position.east_m),
                    static_cast<real_t>(position_velocity_ned.position.down_m)
                };
            });

            _position_history.write([&position_velocity_ned](std::queue<vec3_t>& history) {
                auto xyz = ned_to_xyz(position_velocity_ned.position.north_m, position_velocity_ned.position.east_m, position_velocity_ned.position.down_m);
                history.push(xyz);
                if (history.size() > MAX_POSITION_HISTORY_LENGTH) {
                    history.pop();
                }
            });
        });

        _telemetry->subscribe_attitude_euler([this](mavsdk::Telemetry::EulerAngle euler) {
            _data_attitude.write([&euler](attitude_t& att) {
                att = {
                    static_cast<real_t>(euler.roll_deg),
                    static_cast<real_t>(euler.pitch_deg),
                    static_cast<real_t>(euler.yaw_deg)
                };
            });
        });

        _telemetry->subscribe_heading([this](mavsdk::Telemetry::Heading heading) {
            _data_heading.write([heading](heading_t& head) {
                head = static_cast<real_t>(heading.heading_deg);
            });
        });

        _telemetry->subscribe_distance_sensor([this](mavsdk::Telemetry::DistanceSensor distance_sensor) {
            _data_distance_sensor.write([&distance_sensor](distance_sensor_t& dist) {
                dist = {
                    static_cast<real_t>(distance_sensor.current_distance_m),
                    static_cast<real_t>(distance_sensor.minimum_distance_m),
                    static_cast<real_t>(distance_sensor.maximum_distance_m)
                };
            });
        });

        _telemetry->subscribe_battery([this](mavsdk::Telemetry::Battery battery) {
            _data_battery.write([&battery](battery_t& bat) {
                bat = {
                    static_cast<real_t>(battery.voltage_v),
                    static_cast<real_t>(battery.remaining_percent)
                };
            });
        });

        _telemetry->subscribe_fixedwing_metrics([this](mavsdk::Telemetry::FixedwingMetrics metrics) {
            _data_metrics.write([&metrics](metrics_t& met) {
                met = {
                    static_cast<real_t>(metrics.airspeed_m_s),
                    static_cast<real_t>(metrics.throttle_percentage),
                    static_cast<real_t>(metrics.climb_rate_m_s)
                };
            });
        });

        _telemetry->subscribe_armed([this](bool is_armed) {
            _data_is_armed.write([is_armed](bool& armed) {
                armed = is_armed;
            });
        });

        _telemetry->subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode flight_mode) {
            _data_flight_mode.write([flight_mode](flight_mode_t fm) {
                fm = static_cast<flight_mode_t>(flight_mode);
            });
        });
    }
};
}
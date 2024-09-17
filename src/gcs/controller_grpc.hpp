#pragma once

#include "../dubins/dubins.hpp"
#include "grpcpp/client_context.h"
#include "grpcpp/support/status.h"
#include <chrono>
#include <functional>
#include <grpcpp/grpcpp.h>
#include <air_control.grpc.pb.h>

#include <memory>
#include <spdlog/spdlog.h>

#include <mutex>
#include <condition_variable>
#include <string>
#include <thread>
#include <vector>

namespace rota {

class grpc_client {
public:
    grpc_client(std::shared_ptr<grpc::Channel> channel) :
        _stub(air_control::NewStub(channel)) {}

    void do_mission_chase(const std::vector<dubins_path_t>& trajectory, std::function<void(std::uint64_t, std::uint64_t)> end_callback, std::function<void(void)>& cancel_func)
    {
        class recorder : public grpc::ClientWriteReactor<dubins_path_t> {
        public:
            recorder(air_control::Stub* stub, const std::vector<dubins_path_t>* paths, std::function<void(std::uint64_t, std::uint64_t)> end_callback) :
                _paths(paths), _index{0}, _done(false), _end_callback(end_callback)
            {
                stub->async()->do_mission_chase(&_context, &_result, this);

                StartWrite(&_paths->operator[](_index++));
                StartCall();
            }

            void OnWriteDone(bool) override {
                if (_index == _paths->size()) {
                    StartWritesDone();
                }
                else {
                    StartWrite(&_paths->at(_index++));
                }
            }

            void OnDone(const grpc::Status& s) override {
                std::unique_lock<std::mutex> lock(_mutex);
                spdlog::info("RPC completed");
                _done = true;
                _cv.notify_one();
            }

            void Await() {
                std::unique_lock<std::mutex> lock(_mutex);
                _cv.wait(lock, [this] { return _done; });
                _end_callback(_result.start_time(), _result.end_time());
                return;
            }

            void Cancel() {
                _context.TryCancel();
            }

        private:
            const std::vector<dubins_path_t>* _paths;
            std::size_t _index;
            bool _done;
            std::function<void(std::uint64_t, std::uint64_t)> _end_callback;

            chase_result_t _result;
            grpc::ClientContext _context;
            std::mutex _mutex;
            std::condition_variable _cv;
        };

        auto rec = std::make_shared<recorder>(_stub.get(), &trajectory, end_callback);

        cancel_func = [rec](){
            if (rec.use_count() > 1)
                rec->Cancel();
        };

        std::thread thread([this, rec = std::move(rec)]() {
            rec->Await();
        });
        thread.detach();
    }

    void do_mission_kamikaze(const std::vector<dubins_path_t>& trajectory, std::function<void(const std::string&)> end_callback, std::function<void(void)>& cancel_func)
    {
        class recorder : public grpc::ClientWriteReactor<dubins_path_t> {
        public:
            recorder(air_control::Stub* stub, const std::vector<dubins_path_t>* paths, std::function<void(const std::string&)> end_callback) :
                _paths(paths), _index{0}, _done(false), _end_callback(end_callback)
            {
                stub->async()->do_mission_kamikaze(&_context, &_qr_result, this);

                StartWrite(&_paths->operator[](_index++));
                StartCall();
            }

            void OnWriteDone(bool) override {
                if (_index == _paths->size()) {
                    StartWritesDone();
                }
                else {
                    StartWrite(&_paths->at(_index++));
                }
            }

            void OnDone(const grpc::Status& s) override {
                std::unique_lock<std::mutex> lock(_mutex);
                spdlog::info("RPC completed");
                _done = true;
                _cv.notify_one();
            }

            void Await() {
                std::unique_lock<std::mutex> lock(_mutex);
                _cv.wait(lock, [this] { return _done; });
                _end_callback(_qr_result.text());
                return;
            }

            void Cancel() {
                _context.TryCancel();
            }

        private:
            const std::vector<dubins_path_t>* _paths;
            std::size_t _index;
            bool _done;
            std::function<void(const std::string&)> _end_callback;

            qr_result_t _qr_result;
            grpc::ClientContext _context;
            std::mutex _mutex;
            std::condition_variable _cv;
        };

        auto rec = std::make_shared<recorder>(_stub.get(), &trajectory, end_callback);

        cancel_func = [rec](){
            if (rec.use_count() > 1)
                rec->Cancel();
        };

        std::thread thread([this, rec = std::move(rec)]() {
            rec->Await();
        });
        thread.detach();
    }

    // bool do_action(action_request_t action_request) {
    //     grpc::ClientContext context;
    //     bool result;
    //     std::mutex mu;
    //     std::condition_variable cv;
    //     bool done = false;
    //     action_status_t action_status;
    //     action_status.set_success(false);

    //     _stub->async()->do_action(&context, &action_request, &action_status,
    //     [&result, &mu, &cv, &done, &action_status](grpc::Status status){
    //         bool ret = false;

    //         if (!status.ok()) {
    //             spdlog::error("RPC do_takeoff failed.");
    //         }
    //         else if (!action_status.success()) {
    //             spdlog::error("RPC do_takeoff returned false.");
    //         }
    //         else {
    //             spdlog::info("RPC do_takeoff succeeded.");
    //             ret = true;
    //         }

    //         std::lock_guard<std::mutex> lock(mu);
    //         result = ret;
    //         done = true;
    //         cv.notify_one();
    //     });

    //     std::unique_lock<std::mutex> lock(mu);
    //     cv.wait(lock, [&done] { return done; });
    //     return result;
    // }

private:
    std::unique_ptr<air_control::Stub> _stub;
};

class controller_grpc {
public:
    controller_grpc(const std::string& address) :
        _client(grpc::CreateChannel(address, grpc::InsecureChannelCredentials())) {}

    void do_mission_chase(std::function<void(std::uint64_t, std::uint64_t)> end_callback) {
        spdlog::info("Doing mission chase.");
        auto msg = prepare_path_message({ DubinsPath{} });
        _client.do_mission_chase(msg, end_callback, cancel_chase);
    }

    void do_mission_kamikaze(std::function<void(const std::string&)> end_callback) {
        spdlog::info("Doing mission kamikaze.");
        auto msg = prepare_path_message({ DubinsPath{} });
        _client.do_mission_kamikaze(msg, end_callback, cancel_kamikaze);
    }

    void abort_mission_kamikaze() {
        if (cancel_kamikaze) cancel_kamikaze();
    }

    // bool do_action(action_request_t action_request) {
    //     return _client.do_action(action_request);
    // }

    std::vector<dubins_path_t> prepare_path_message(const std::vector<DubinsPath>& paths) {
        std::vector<dubins_path_t> trajectory;
        trajectory.resize(paths.size());
        for (std::size_t i = 0; i < trajectory.size(); ++i) {
            auto path = paths[i];

            // set start
            auto point = new ::point_t;
            point->set_x(path.qi[0]);
            point->set_y(path.qi[1]);

            auto start = new ::configuration_t;
            start->set_allocated_position(point);
            start->set_yaw(path.qi[2]);

            trajectory[i].set_allocated_start(start);

            // set segment_lengths
            auto segment_lengths = new ::segment_lengths_t;
            segment_lengths->set_segment_1(path.param[0]);
            segment_lengths->set_segment_2(path.param[1]);
            segment_lengths->set_segment_3(path.param[2]);

            trajectory[i].set_allocated_segment_lengths(segment_lengths);

            // set radius
            trajectory[i].set_radius(path.rho);

            // set type
            trajectory[i].set_type(
                static_cast<::dubins_path_type_t>((static_cast<int>(path.type) + 1))
            );
        }
        return trajectory;
    }
private:
    grpc_client _client;
    std::weak_ptr<grpc::ClientWriteReactor<dubins_path_t>> context_mission_kamikaze;
    std::function<void(void)> cancel_chase;
    std::function<void(void)> cancel_kamikaze;
};
}
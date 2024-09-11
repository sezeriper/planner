#pragma once

#include "path_tracker.hpp"

#include <mavsdk.h>
#include <plugins/action/action.h>
#include <plugins/offboard/offboard.h>
#include <plugins/telemetry/telemetry.h>

#include <air_control.grpc.pb.h>
#include <air_control.pb.h>
#include <grpcpp/grpcpp.h>

#include <spdlog/spdlog.h>


namespace rota {
using namespace messages;


class air_controller final : public air_control::CallbackService
{
public:
    air_controller(path_tracker& tracker) : _tracker(tracker) {}

    grpc::ServerReadReactor<dubins_path_t>* do_mission_chase(
        grpc::CallbackServerContext* context, empty_t* empty) override
    {
        class recorder : public grpc::ServerReadReactor<dubins_path_t>
        {
        public:
            recorder(empty_t* empty, path_tracker& tracker, std::vector<dubins_path_t>* paths) :
                _tracker(tracker),
                _paths(paths)
                { StartRead(&_path); }

                void OnReadDone(bool ok) override {
                    if (ok) {
                        _paths->push_back(_path);
                        StartRead(&_path);
                    }
                    else {
                        Finish(grpc::Status::OK);
                    }
                }

                void OnDone() override {
                    // spdlog::info("Received {} paths.", _paths->size());
                    {
                        std::vector<DubinsPath> paths;
                        paths.reserve(_paths->size());

                        for (const auto& path : *_paths) {
                            paths.push_back({
                                {path.start().position().x(), path.start().position().y(), path.start().yaw()},
                                {path.segment_lengths().segment_1(), path.segment_lengths().segment_2(), path.segment_lengths().segment_3()},
                                path.radius(),
                                static_cast<DubinsPathType>(path.type() - 1)
                            });
                        }

                        _tracker.set_path(std::move(paths));
                    }
                    _paths->clear();
                    delete this;
                }

                void OnCancel() override {
                    spdlog::error("RPC cancelled.");
                }
        private:
            path_tracker& _tracker;
            std::vector<dubins_path_t>* _paths;
            dubins_path_t _path;
        };
        return new recorder(empty, _tracker, &paths);
    }

private:
    std::vector<dubins_path_t> paths;
    path_tracker& _tracker;
};
}
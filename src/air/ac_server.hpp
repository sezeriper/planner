#pragma once

#include "path_tracker.hpp"
#include "qr_reader.hpp"
#include "uav_detector.hpp"

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
    air_controller(path_tracker& tracker, qr_reader& reader, uav_detector& detector) : _tracker(tracker), _reader(reader), _detector(detector) {}

    grpc::ServerReadReactor<dubins_path_t>* do_mission_chase(
        grpc::CallbackServerContext* context, chase_result_t* result) override
    {
        class recorder : public grpc::ServerReadReactor<dubins_path_t>
        {
        public:
            recorder(chase_result_t* result, std::vector<dubins_path_t>* paths, path_tracker& tracker, uav_detector& detector) :
                _result(result),
                _paths(paths),
                _tracker(tracker),
                _detector(detector)
            {
                detector.on_read([this](std::uint64_t start, std::uint64_t end) {
                    _detector.end();
                    _result->set_start_time(start);
                    _result->set_end_time(end);
                    spdlog::info("Detected UAV.");
                    Finish(grpc::Status::OK);
                });
                detector.start();
                StartRead(&_path);
            }

            void OnReadDone(bool ok) override {
                if (ok) {
                    _paths->push_back(_path);
                    StartRead(&_path);
                }
            }

            void OnDone() override {
                // spdlog::info("Received {} paths.", _paths->size());
                // {
                //     std::vector<DubinsPath> paths;
                //     paths.reserve(_paths->size());

                //     for (const auto& path : *_paths) {
                //         paths.push_back({
                //             {path.start().position().x(), path.start().position().y(), path.start().yaw()},
                //             {path.segment_lengths().segment_1(), path.segment_lengths().segment_2(), path.segment_lengths().segment_3()},
                //             path.radius(),
                //             static_cast<DubinsPathType>(path.type() - 1)
                //         });
                //     }

                //     _tracker.set_path(std::move(paths));
                // }
                // _paths->clear();

                _detector.end();
                delete this;
            }

            void OnCancel() override {
                _detector.end();
                spdlog::error("RPC cancelled.");
            }
        private:
            chase_result_t* _result;
            std::vector<dubins_path_t>* _paths;
            path_tracker& _tracker;
            uav_detector& _detector;
            dubins_path_t _path;
        };
        return new recorder(result, &paths, _tracker, _detector);
    }

    grpc::ServerReadReactor<dubins_path_t>* do_mission_kamikaze(
        grpc::CallbackServerContext* context, qr_result_t* qr_result) override
    {
        class recorder : public grpc::ServerReadReactor<dubins_path_t>
        {
        public:
            recorder(qr_result_t* qr_result, std::vector<dubins_path_t>* paths, path_tracker& tracker, qr_reader& reader) :
                _qr_result(qr_result),
                _paths(paths),
                _tracker(tracker),
                _reader(reader),
                _qr_text("")
                {
                    _reader.on_read([this](const std::string& result) {
                        _reader.end();
                        _qr_result->set_text(result);
                        Finish(grpc::Status::OK);
                    });
                    _reader.start();

                    StartRead(&_path);
                }

                void OnReadDone(bool ok) override {
                    if (ok) {
                        _paths->push_back(_path);
                        StartRead(&_path);
                    }
                    // else {
                    //     std::vector<DubinsPath> paths;
                    //     paths.reserve(_paths->size());

                    //     for (const auto& path : *_paths) {
                    //         paths.push_back({
                    //             {path.start().position().x(), path.start().position().y(), path.start().yaw()},
                    //             {path.segment_lengths().segment_1(), path.segment_lengths().segment_2(), path.segment_lengths().segment_3()},
                    //             path.radius(),
                    //             static_cast<DubinsPathType>(path.type() - 1)
                    //         });
                    //     }
                    //     _tracker.set_path(paths);
                    // }
                }

                void OnDone() override {
                    // {
                    //     std::vector<DubinsPath> paths;
                    //     paths.reserve(_paths->size());

                    //     for (const auto& path : *_paths) {
                    //         paths.push_back({
                    //             {path.start().position().x(), path.start().position().y(), path.start().yaw()},
                    //             {path.segment_lengths().segment_1(), path.segment_lengths().segment_2(), path.segment_lengths().segment_3()},
                    //             path.radius(),
                    //             static_cast<DubinsPathType>(path.type() - 1)
                    //         });
                    //     }

                    //     _tracker.set_path(std::move(paths));
                    // }
                    // _paths->clear();
                    _reader.end();
                    delete this;
                }

                void OnCancel() override {
                    _reader.end();
                    spdlog::info("RPC cancelled.");
                }
        private:
            qr_result_t* _qr_result;
            path_tracker& _tracker;
            qr_reader& _reader;
            std::vector<dubins_path_t>* _paths;
            dubins_path_t _path;
            std::string _qr_text;
        };
        return new recorder(qr_result, &paths, _tracker, _reader);
    }

private:
    std::vector<dubins_path_t> paths;
    path_tracker& _tracker;
    qr_reader& _reader;
    qr_result_t _qr_result;
    uav_detector& _detector;
};
}
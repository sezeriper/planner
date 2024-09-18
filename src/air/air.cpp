#include "path_tracker.hpp"
#include "uav_detector.hpp"
#include "qr_reader.hpp"
#include "ac_server.hpp"
#include "video_recorder.hpp"

#include <mavsdk.h>
#include <plugins/action/action.h>
#include <plugins/offboard/offboard.h>
#include <plugins/telemetry/telemetry.h>
#include <spdlog/spdlog.h>


using namespace rota;

static bool run_server() {
    plane_control pc;
    bool result = pc.init("udp://:14445");

    if (!result) {
        spdlog::error("Mavlink connection failed.");
        return 1;
    }

    path_tracker pt(pc);

    video_recorder vid(pc);

    qr_reader reader(vid);
    uav_detector detector(vid, pc);

    std::string server_address{"localhost:50051"};
    air_controller service(pt, reader, detector);
    grpc::ServerBuilder builder;

    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    spdlog::info("Server listening on {}", server_address);
    server->Wait();
    spdlog::info("Server shutting down.");
    return 0;
}

int main() {
    run_server();
    return 0;
}

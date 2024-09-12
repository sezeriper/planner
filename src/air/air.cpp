#include "ac_server.hpp"

#include <mavsdk.h>
#include <plugins/action/action.h>
#include <plugins/offboard/offboard.h>
#include <plugins/telemetry/telemetry.h>
#include <spdlog/spdlog.h>

using namespace rota;

static void run_server() {
    plane_control pc;
    pc.init("udp://:14540");

    path_tracker pt(pc);

    std::string server_address{"localhost:50051"};
    air_controller service(pt);
    grpc::ServerBuilder builder;

    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    spdlog::info("Server listening on {}", server_address);
    server->Wait();
    spdlog::info("Server shutting down.");
}

int main() {
    run_server();
    return 0;
}
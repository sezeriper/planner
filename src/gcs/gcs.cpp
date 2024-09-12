#define RL_CULL_DISTANCE_FAR 100000000.0 // not working

#include "referee_client.hpp"
#include "controller_mavlink.hpp"
#include "controller_grpc.hpp"
#include "ui.hpp"
#include "field.hpp"

using namespace rota;

int main() {

    obstacles_t obstacles {
        {{-10.0f, -10.0f}, 10.0f},
        {{-10.0f, 10.0f}, 10.0f},
        {{10.0f, -10.0f}, 10.0f},
        {{10.0f, 10.0f}, 10.0f},
        {{-20.0f, -20.0f}, 10.0f},
        {{-20.0f, 20.0f}, 10.0f},
        {{20.0f, -20.0f}, 10.0f},
        {{20.0f, 20.0f}, 10.0f},
    };

    border_t border {
        {-50.0f, -50.0f},
        {50.0f, -50.0f},
        {50.0f, 50.0f},
        {-50.0f, 50.0f},
    };

    field_t field {border, obstacles};

    controller_mavlink mav_ctrl;
    mav_ctrl.init_mavsdk("udp://:14550");

    controller_grpc grpc_ctrl("localhost:50051");

    referee_client referee("127.0.0.1", mav_ctrl);

    ui_t ui(field, mav_ctrl, grpc_ctrl);
    
    while (!WindowShouldClose()) {
        ui.draw();
        ui.handle_input();
    }

    return 0;
}
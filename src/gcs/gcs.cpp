#include "referee_client.hpp"
#include "controller_mavlink.hpp"
#include "controller_grpc.hpp"
#include "ui.hpp"
#include "field.hpp"

using namespace rota;

int main() {

    obstacles_t obstacles {
        // {{-10.0f, -10.0f}, 10.0f},
        // {{-10.0f, 10.0f}, 10.0f},
        // {{10.0f, -10.0f}, 10.0f},
        // {{10.0f, 10.0f}, 10.0f},
        // {{-20.0f, -20.0f}, 10.0f},
        // {{-20.0f, 20.0f}, 10.0f},
        // {{20.0f, -20.0f}, 10.0f},
        // {{20.0f, 20.0f}, 10.0f},
    };

    border_t border {
        {36.942314f, 35.563323f},
        {36.942673f, 35.553363f},
        {36.937683f, 35.553324f},
        {36.937864f, 35.562873f}
    };

    // border_t border {
    //     {36.938123f, 35.562996f},
    //     {36.938105f, 35.563552f},
    //     {36.937568f, 35.563549f},
    //     {36.937610f, 35.562892f}
    // };

    field_t field {border, obstacles};

    controller_mavlink mav_ctrl;
    bool result = mav_ctrl.init_mavsdk("udp://:14445");

    if (!result) {
        spdlog::error("Mavsdk initialization failed.");
        return 1;
    }

    controller_grpc grpc_ctrl("localhost:50051");

    referee_client referee("10.0.0.11:10001", "rotasscgm", "n55XFpQHmn", mav_ctrl);

    ui_t ui(field, mav_ctrl, grpc_ctrl, referee);

    while (!WindowShouldClose()) {
        ui.draw();
        ui.handle_input();
    }

    return 0;
}
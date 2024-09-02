#include "ui.hpp"
#include "field.hpp"

#include <imgui_internal.h>
#include <imgui.h>
#include <raylib.h>
#include <rlImGui.h>


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

    ui_t ui(field);
    
    bool is_first_run = true;
    while (!WindowShouldClose()) {
        ui.draw();
        ui.get_input();
    }

    return 0;
}
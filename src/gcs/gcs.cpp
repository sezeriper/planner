#include <imgui_internal.h>
#include <imgui.h>
#include <rlImGui.h>

constexpr int SCREEN_WIDTH = 1280;
constexpr int SCREEN_HEIGHT = 720;
constexpr auto TITLE = "Pars - GCS";
constexpr int FPS = 60;
constexpr auto BG_COLOR = DARKGRAY;


int main() {
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, TITLE);
    SetTargetFPS(FPS);
    rlImGuiSetup(true);
    // enable docking
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    bool is_first_run = true;

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(BG_COLOR);
        rlImGuiBegin();


        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_PassthruCentralNode;
        ImGuiID dockspace_id = ImGui::GetID("Dockspace");
        ImGui::DockSpaceOverViewport(dockspace_id, viewport, dockspace_flags);

        ImGui::Begin("DockSpace Demo");
        ImGui::End();

        rlImGuiEnd();
        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();

    return 0;
}
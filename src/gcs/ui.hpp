#pragma once

#include "field.hpp"
#include "visualizer.hpp"
#include "camera.hpp"

#include <raylib.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <rlImGui.h>


constexpr int SCREEN_WIDTH = 1920;
constexpr int SCREEN_HEIGHT = 1080;
constexpr auto TITLE = "Pars - GCS";
constexpr int FPS = 60;
constexpr auto BG_COLOR = DARKGRAY;
constexpr float FONT_SIZE = 24.0f;

namespace rota {
class ui_t {
public:
    ui_t(field_t field) :
        _cam(),
        _io(),
        _font(),
        _field(field),
        _vis(),
        _prev_region(0.0f, 0.0f),
        _render_texture()
    {
        InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, TITLE);
        SetTargetFPS(FPS);
        rlImGuiSetup(true);

        _vis.init(_field);

        // enable docking
        _io = &ImGui::GetIO();
        _io->ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        _font = _io->Fonts->AddFontFromFileTTF("resources/JetBrainsMonoNerdFont-Medium.ttf", FONT_SIZE);
        rlImGuiReloadFonts();
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    }

    ~ui_t() {
        rlImGuiShutdown();
        UnloadRenderTexture(_render_texture);
        CloseWindow();
    }

    void draw() {
        BeginDrawing();
        ClearBackground(BG_COLOR);
        rlImGuiBegin();
        ImGui::PushFont(_font);

        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_PassthruCentralNode;
        ImGuiID dockspace_id = ImGui::GetID("Dockspace");
        ImGui::DockSpaceOverViewport(dockspace_id, viewport, dockspace_flags);

        ImGuiWindowClass window_class;
        window_class.DockNodeFlagsOverrideSet = ImGuiDockNodeFlags_NoDockingOverMe | ImGuiDockNodeFlags_NoDockingOverOther | ImGuiDockNodeFlags_NoDockingSplit;

        ImGui::SetNextWindowClass(&window_class);
        ImGui::Begin("Field View");

        ImVec2 region = ImGui::GetContentRegionAvail();
        if (region.x != _prev_region.x || region.y != _prev_region.y) {
            UnloadRenderTexture(_render_texture);
            _render_texture = LoadRenderTexture(region.x, region.y);
        }
        _prev_region = region;

        BeginTextureMode(_render_texture);
        ClearBackground(BG_COLOR);
        BeginMode3D(_cam.get());
        _vis.draw_field(_field);
        EndMode3D();
        EndTextureMode();

        ImGui::Image((void*)&_render_texture.texture.id, region, ImVec2(0, 1), ImVec2(1, 0));
        ImGui::End();


        ImGui::SetNextWindowClass(&window_class);
        ImGui::Begin("Controls");
        ImGui::End();


        rlImGuiEnd();
        EndDrawing();
    }

    void get_input() {
        _cam.update();
    }
private:
    camera _cam;
    ImGuiIO* _io;
    ImFont* _font;
    field_t _field;
    visualizer _vis;
    ImVec2 _prev_region;
    RenderTexture2D _render_texture;
};
}
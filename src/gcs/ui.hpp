#pragma once

#include "controller_grpc.hpp"
#include "controller_mavlink.hpp"
#include "field.hpp"
#include "../math.hpp"
#include "referee_client.hpp"
#include "spdlog/spdlog.h"
#include "visualizer.hpp"
#include "camera.hpp"

#include <raylib.h>
#include <rlgl.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <rlImGui.h>
#include <unordered_map>


constexpr int SCREEN_WIDTH = 1280;
constexpr int SCREEN_HEIGHT = 720;
constexpr auto TITLE = "Pars - GCS";
constexpr int FPS = 60;
constexpr auto BG_COLOR = DARKGRAY;
constexpr float FONT_SIZE = 24.0f;
constexpr float FLY_AREA_SIZE = 2000.0f;
constexpr float GRID_CELL_SIZE = 10.0f;

namespace rota {
class ui_t {
public:
    ui_t(field_t field, controller_mavlink& mav_ctrl, controller_grpc& grpc_ctrl, referee_client& referee) :
        _cam(),
        _io(),
        _font(),
        _field(field),
        _vis(),
        _prev_region(0.0f, 0.0f),
        _render_texture(),
        _mav_ctrl(mav_ctrl),
        _grpc_ctrl(grpc_ctrl),
        _referee(referee),
        _cam_dist{100.0f}
    {
        InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, TITLE);
        SetTargetFPS(FPS);
        rlImGuiSetup(true);
        rlSetClipPlanes(0.001, 2000.0);

        for (auto& point : _field.border) {
            point = _mav_ctrl.coords_to_xz(point.x, point.y);
        }

        _vis.init(_field);

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

        _window_pos = ImGui::GetCursorScreenPos();

        ImVec2 region = ImGui::GetContentRegionAvail();
        if (region.x != _prev_region.x || region.y != _prev_region.y) {
            UnloadRenderTexture(_render_texture);
            _render_texture = LoadRenderTexture(region.x, region.y);
        }
        _prev_region = region;

        BeginTextureMode(_render_texture);
        ClearBackground(BG_COLOR);
        BeginMode3D(_cam.get());

        _vis.draw_field();

        _vis.draw_position_history(_mav_ctrl.get_position_history(), 0.0f, GREEN);

        auto pos = _mav_ctrl.get_position_xyz();
        DrawSphere({pos.x, pos.y, pos.z}, 2.0f, GREEN);

        _vis.draw_grid(FLY_AREA_SIZE / GRID_CELL_SIZE, FLY_AREA_SIZE / GRID_CELL_SIZE, GRID_CELL_SIZE, {0.0f, 0.0f, 0.0f});

        DrawSphere({_picked_point.x, 50.0f, _picked_point.y}, 5.0f, RED);
        DrawLine3D({_picked_point.x, 0.0f, _picked_point.y}, {_picked_point.x, 50.0f, _picked_point.y}, RED);

        if (!_picked_paths.empty()) {
            _vis.draw_paths(_picked_paths, 50.0f, PURPLE);
        }

        EndMode3D();
        EndTextureMode();

        ImGui::Image((void*)&_render_texture.texture.id, region, ImVec2(0, 1), ImVec2(1, 0));
        ImGui::End();

        ImGui::SetNextWindowClass(&window_class);
        ImGui::Begin("Controls");

        bool do_takeoff = button_once("Takeoff");
        bool do_land = button_once("Land");
        bool do_arm = button_once("Arm");
        bool do_hold = button_once("Hold");
        bool do_login = button_once("Login");
        bool do_kamikaze = button_once("Kamikaze");
        bool do_chase = button_once("Chase");
        bool abort_kamikaze = button_once("Abort Kamikaze");
        bool abort_chase = button_once("Abort Chase");
        bool get_hss = button_once("Get HSS");
        bool get_qr = button_once("Get QR");

        ImGui::SliderFloat("Camera Distance", &_cam_dist, 50.0f, 500.0f);
        ImGui::End();

        ImGui::PopFont();
        rlImGuiEnd();
        EndDrawing();

        if (do_takeoff) {
            _mav_ctrl.do_takeoff();
        }
        if (do_land) {
            _mav_ctrl.do_land();
        }
        if (do_arm) {
            _mav_ctrl.do_arm();
        }
        if (do_hold) {
            _mav_ctrl.do_hold();
        }
        if (do_login) {
            _referee.login();
        }
        if (do_kamikaze) {
            spdlog::info("Kamikaze mission started.");
            auto start = _referee.get_time();
            _grpc_ctrl.do_mission_kamikaze([this, start](const std::string& text) {
                auto end = _referee.get_time();
                _referee.send_kamikaze_info(start, end, text);
            });
        }
        if (do_chase) {
            spdlog::info("Chase mission started.");
            _grpc_ctrl.do_mission_chase([this](std::uint64_t start, std::uint64_t end){
                spdlog::info("Chase mission completed.");
                spdlog::info("Locked for {} seconds.", static_cast<double>(end - start) / 1000000.0);
                _referee.send_lock_info(referee_client::unix_to_time(start), referee_client::unix_to_time(end));
            });
        }
        if (abort_kamikaze) {
            _grpc_ctrl.abort_mission_kamikaze();
        }
        if (get_hss) {
            auto hss_arr = _referee.get_hss_coords();
            _field.obstacles.clear();
            for (const referee_client::hss_t& hss : hss_arr) {
                auto obstacle = _referee.hss_to_obstacle(hss);
                _field.obstacles.push_back(obstacle);
            }
            _vis.update_field(_field);
        }
        if (get_qr) {
            _referee.get_qr_coords();
        }
    }

    bool button_once(const std::string& name) {
        bool btn = ImGui::Button(name.c_str());
        bool ret = btn == false && _button_states[name] == true;
        _button_states[name] = btn;
        return ret;
    }

    void handle_input() {
        _cam.set_target(_mav_ctrl.get_position_xyz());
        _cam.set_distance(_cam_dist);
        _cam.handle_input();

        // if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
        //     _picked_point = pick_point();
        // }

        // update_path();
    }

    // void update_path() {
    //     if (GetTime() - _last_update < 4.0f) {
    //         return;
    //     }
    //     auto pos = _mav_ctrl.get_position_xyz();
    //     configuration_t from = {
    //         pos.x,
    //         pos.z,
    //         _mav_ctrl.get_heading()
    //     };
    //     configuration_t to = {
    //         _picked_point.x,
    //         _picked_point.y,
    //         0.0f
    //     };
    //     auto path = find_path(from, to, 25.0f);
    //     if (!path) {
    //         spdlog::error("No path found.");
    //     }
    //     else {
    //         _picked_paths.clear();
    //         _picked_paths.push_back(path.value());
    //         _grpc_ctrl.do_mission_chase(_picked_paths);
    //     }
    //     _last_update = GetTime();
    // }

    // Vector2 pick_point() const {
    //     Vector2 local_mouse = GetMousePosition();
    //     local_mouse.x -= _window_pos.x;
    //     local_mouse.y -= _window_pos.y;
    //     Ray ray = GetScreenToWorldRayEx(local_mouse, _cam.get(), _prev_region.x, _prev_region.y);

    //     Vector3 p1 {-FLY_AREA_SIZE / 2.0f, 0.0f, -FLY_AREA_SIZE / 2.0f};
    //     Vector3 p2 {-FLY_AREA_SIZE / 2.0f, 0.0f,  FLY_AREA_SIZE / 2.0f};
    //     Vector3 p3 { FLY_AREA_SIZE / 2.0f, 0.0f,  FLY_AREA_SIZE / 2.0f};
    //     Vector3 p4 { FLY_AREA_SIZE / 2.0f, 0.0f, -FLY_AREA_SIZE / 2.0f};

    //     RayCollision hit = GetRayCollisionQuad(ray, p1, p2, p3, p4);

    //     if (hit.hit) {
    //         spdlog::info("Picked point: ({}, {})", hit.point.x, hit.point.z);
    //         return {hit.point.x, hit.point.z};
    //     }
    //     return _picked_point;
    // }
private:
    camera _cam;
    ImGuiIO* _io;
    ImFont* _font;
    field_t _field;
    visualizer _vis;
    ImVec2 _prev_region;
    RenderTexture2D _render_texture;
    controller_mavlink& _mav_ctrl;
    controller_grpc& _grpc_ctrl;
    referee_client& _referee;

    real_t _cam_dist;
    Vector2 _picked_point;
    std::vector<DubinsPath> _picked_paths;
    ImVec2 _window_pos;
    double _last_update = 0.0;
    std::unordered_map<std::string, bool> _button_states;
};
}
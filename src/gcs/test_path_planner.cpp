#include "path_planner.hpp"
#include "visualizer.hpp"
#include "camera.hpp"

#include <raylib.h>
#include <rlgl.h>
#include <raymath.h>
#include <imgui.h>
#include <rlImGui.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <memory>
#include <chrono>

using namespace rota;

std::vector<path_planner::obstacle> gen_obstacles(std::size_t num)
{
    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<float> dis_uni(-38.0f, 38.0f);
    std::normal_distribution<float> dis_nor(10.0f, 1.0f);

    std::vector<path_planner::obstacle> obstacles;
    obstacles.reserve(num);

    for (std::size_t i = 0; i < num; ++i) {
        vec2_t position{dis_uni(gen), dis_uni(gen)};
        float radius = dis_nor(gen);

        obstacles.push_back(path_planner::obstacle{position, radius});
    }

    return obstacles;
}

int main() {
    constexpr int screenWidth = 1280;
    constexpr int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "RRT* Dubins Path Planning");

    SetTargetFPS(60);

    rlImGuiSetup(true);

    std::vector<vec2_t> border_vec2_ts {
        {-50.0f, -50.0f},
        {50.0f, -50.0f},
        {50.0f, 50.0f},
        {-50.0f, 50.0f},
    };

    auto vis = visualizer(border_vec2_ts);

    constexpr int NUM_OBSTACLES = 8;
    // auto obstacles = gen_obstacles(NUM_OBSTACLES);
    std::vector<obstacle_t> obstacles {
        {{-10.0f, -10.0f}, 10.0f},
        {{-10.0f, 10.0f}, 10.0f},
        {{10.0f, -10.0f}, 10.0f},
        {{10.0f, 10.0f}, 10.0f},
        {{-20.0f, -20.0f}, 10.0f},
        {{-20.0f, 20.0f}, 10.0f},
        {{20.0f, -20.0f}, 10.0f},
        {{20.0f, 20.0f}, 10.0f},
    };

    configuration_t start{-45.0f, 0.0f, PI * -0.5f};
    configuration_t goal{40.0f, 0.0f, PI * 0.5f};

    real_t RHO = 8.0f;
    real_t STEP_SIZE = 8.0f;
    real_t NEAR_RADIUS = 16.0f;
    real_t GOAL_RADIUS = 16.0f;
    auto planner = std::make_unique<rrtstar_dubins>(start, goal, STEP_SIZE, GOAL_RADIUS, NEAR_RADIUS, RHO, border_vec2_ts, obstacles);

    camera cam;

    int run_time = 1000;
    bool show_nodes = true;
    bool do_sample = false;
    bool do_reset = false;
    std::vector<DubinsPath> paths;
    trajectory_t traj;
    trajectory_t traj_simple;
    while (!WindowShouldClose())
    {
        cam.update();
        
        BeginDrawing();
        ClearBackground(DARKGRAY);

        BeginMode3D(cam.get());

        vis.draw_border();
        vis.draw_obstacles(obstacles);
        if (show_nodes) {
            if (planner->get_node_count() > 1)  {
                vis.draw_nodes(planner->get_nodes(), RHO);
            }
        }
        else {
            vis.draw_paths(paths, 0.0f, GREEN);
            // vis.draw_traj_simple(traj, 0.0f, BLUE);
            vis.draw_traj_smooth(traj_simple, RHO, 0.0f, RED);
        }
        vis.draw_configuration(start);
        vis.draw_configuration(goal);

        EndMode3D();

        {
            rlImGuiBegin();
            ImGui::Begin("Menu");

                ImGui::SliderInt("Node Count", &run_time, 1, 10000);
                do_sample = ImGui::Button("Sample", ImVec2(100, 20));
                do_reset = ImGui::Button("Reset", ImVec2(100, 20));
                auto added_nodes = std::to_string(planner->get_node_count());
                if (ImGui::Button("Toggle Nodes", ImVec2(100, 20))) {
                    show_nodes = !show_nodes;
                }
                ImGui::Text(added_nodes.c_str());
                ImGui::SliderFloat("Min Radius", &RHO, 1.0f, 64.0f);
                ImGui::SliderFloat("Step Size", &STEP_SIZE, 1.0f, 64.0f);
                ImGui::SliderFloat("Near Radius", &NEAR_RADIUS, 1.0f, 64.0f);
                ImGui::SliderFloat("Goal Radius", &GOAL_RADIUS, 1.0f, 64.0f);

            ImGui::End();
            rlImGuiEnd();
        }

        EndDrawing();

        if (IsKeyPressed(KEY_SPACE) || do_reset) {
            planner = std::make_unique<rrtstar_dubins>(start, goal, STEP_SIZE, GOAL_RADIUS, NEAR_RADIUS, RHO, border_vec2_ts, obstacles);
        }

        if (IsKeyPressed(KEY_ENTER) || do_sample) {
            spdlog::stopwatch sw;
            planner->run_for_ms(std::chrono::milliseconds(run_time));
            // planner->run_for_iter(run_time);
            spdlog::info("RRT* Dubins took {} ms", sw.elapsed_ms().count());

            sw.reset();

            paths = planner->get_paths_to_goal();
            traj = sample_paths(paths, 0.1f);
            traj_simple = planner->simplify_path(traj);
            spdlog::info("Simplification took {} ms", sw.elapsed_ms().count());
        }

    }

    rlImGuiShutdown();
    CloseWindow();

    return 0;
}
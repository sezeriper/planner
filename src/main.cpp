#include "path_planner.hpp"
#include "visualizer.hpp"
#include "utils.hpp"
#include "camera.hpp"

#include <raylib.h>
#include <rlgl.h>
#include <raymath.h>
#include <imgui.h>
#include <rlImGui.h>

#include <memory>
#include <chrono>
#include <numbers>

using namespace rota;

std::vector<path_planner::obstacle> gen_obstacles(std::size_t num)
{
    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<float> dis_uni(-38.0f, 38.0f);
    std::normal_distribution<float> dis_nor(10.0f, 1.0f);

    std::vector<path_planner::obstacle> obstacles;
    obstacles.reserve(num);

    for (std::size_t i = 0; i < num; ++i) {
        point position{dis_uni(gen), dis_uni(gen)};
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

    std::vector<point> border_points {
        {-50.0f, -50.0f},
        {50.0f, -50.0f},
        {50.0f, 50.0f},
        {-50.0f, 50.0f},
    };

    auto vis = visualizer(border_points);

    constexpr int NUM_OBSTACLES = 8;
    // auto obstacles = gen_obstacles(NUM_OBSTACLES);
    std::vector<path_planner::obstacle> obstacles {
        {{-10.0f, -10.0f}, 10.0f},
        {{-10.0f, 10.0f}, 10.0f},
        {{10.0f, -10.0f}, 10.0f},
        {{10.0f, 10.0f}, 10.0f},
        {{-20.0f, -20.0f}, 10.0f},
        {{-20.0f, 20.0f}, 10.0f},
        {{20.0f, -20.0f}, 10.0f},
        {{20.0f, 20.0f}, 10.0f},
    };

    configuration start{{-45.0f, 0.0f}, PI * -0.5f};
    configuration goal{{40.0f, 0.0f}, PI * 0.5f};

    constexpr real_t RHO = 8.0f;
    constexpr real_t STEP_SIZE = 16.0f;
    constexpr real_t NEAR_RADIUS = 16.0f;
    constexpr real_t GOAL_RADIUS = STEP_SIZE;
    auto planner = std::make_unique<path_planner_rrt_star_dubins>(start, goal, STEP_SIZE, GOAL_RADIUS, NEAR_RADIUS, RHO, border_points, obstacles);

    camera cam;

    int run_time = 1000;
    bool show_nodes = true;
    bool do_sample = false;
    bool do_reset = false;
    std::vector<configuration> path;
    while (!WindowShouldClose())
    {
        cam.update();
        
        BeginDrawing();
        ClearBackground(DARKGRAY);

        BeginMode3D(cam.get());

        vis.draw_border();
        vis.draw_obstacles(obstacles);
        if (show_nodes) {
            vis.draw_nodes(planner->get_nodes(), RHO);
        }
        else {
            vis.draw_path_smooth(path, RHO, 0.0f, GREEN);
        }
        vis.draw_grid(planner->get_grid());
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

            ImGui::End();
            rlImGuiEnd();
        }

        EndDrawing();

        if (IsKeyPressed(KEY_SPACE) || do_reset) {
            planner = std::make_unique<path_planner_rrt_star_dubins>(start, goal, STEP_SIZE, GOAL_RADIUS, NEAR_RADIUS, RHO, border_points, obstacles);
        }

        if (IsKeyPressed(KEY_ENTER) || do_sample) {
            {
                timer t("Path Planning");
                planner->run_for_ms(std::chrono::milliseconds(run_time));
            }
            {
                timer t("Path Sampling");
                path = planner->simplify_path(planner->get_path_to_goal(0.1f));
            }
        }

    }

    rlImGuiShutdown();
    CloseWindow();

    return 0;
}
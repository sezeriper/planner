#include "path_planner.hpp"
#include "visualizer.hpp"
#include "utils.hpp"

#include <raylib.h>
#include <rlgl.h>
extern "C" {
    #include <dubins.h>
}

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

    Camera camera{};
    camera.position = Vector3{ 50.0f, 50.0f, 50.0f };
    camera.target = Vector3{ 0.0f, 0.0f, 0.0f };
    camera.up = Vector3{ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    DisableCursor();
    SetTargetFPS(60);

    std::vector<point> border_points {
        {-50.0f, -50.0f},
        {-25.0f, -30.0f},
        {50.0f, -50.0f},
        {50.0f, 50.0f},
        {-50.0f, 50.0f},
    };

    auto vis = visualizer(border_points);

    constexpr int NUM_OBSTACLES = 16;
    auto obstacles = gen_obstacles(NUM_OBSTACLES);

    point startPos{-45.0f, -45.0f};
    point goalPos{45.0f, 45.0f};
    auto planner = std::make_unique<path_planner_rrt_star>(startPos, goalPos, 1.0f, 1.0f, border_points, obstacles);
    

    DubinsPath path;
    double q0[3] {0.0, 0.0, std::numbers::pi * 0.5};
    double q1[3] {12.0, 6.0, std::numbers::pi * 1.0};
    double rho = 4.0;
    dubins_shortest_path(&path, q0, q1, rho);

    double len = dubins_path_length(&path);
    double interval = 1.0;
    std::vector<point> path_points;
    path_points.reserve(std::ceill(len / interval));

    dubins_path_sample_many(&path, interval, [](double q[3], double t, void* user_data) {
        auto vec = reinterpret_cast<std::vector<point>*>(user_data);
        point p{static_cast<float>(q[0]), static_cast<float>(q[1])};
        vec->push_back(p);
        return 0;
    }, &path_points);

    while (!WindowShouldClose())
    {
        UpdateCamera(&camera, CAMERA_THIRD_PERSON);

        if (IsKeyPressed(KEY_SPACE)) {
            obstacles = gen_obstacles(NUM_OBSTACLES);
            planner = std::make_unique<path_planner_rrt_star>(startPos, goalPos, 1.0f, 1.0f, border_points, obstacles);
        }
        
        if (IsKeyPressed(KEY_ENTER)) {
            timer t("RRT*");
            planner->step(20000);
        }

        BeginDrawing();

            ClearBackground(RAYWHITE);

            BeginMode3D(camera);

                // vis.draw_border();
                // vis.draw_obstacles(obstacles);
                // vis.draw_nodes(planner->get_nodes(), planner->get_goal_node());
                vis.draw_path(path_points);

                DrawGrid(25, 2.0f);

            EndMode3D();
        EndDrawing();
    }

    CloseWindow();

    return 0;
}

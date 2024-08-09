#include "path_planner.hpp"

#include <raylib.h>
#include <rlgl.h>
#include <memory>
#include <chrono>


template <typename T>
void drawNodes(const Model& model, const T& nodes, std::size_t goal_node_index = 0) {

    if (goal_node_index != 0) {
        auto goal_node = nodes[goal_node_index];

        while (goal_node.parent != 0) {
            DrawModel(model, {goal_node.position.x, 0.0f, goal_node.position.y}, 3.0f, GREEN);

            DrawLine3D(Vector3{goal_node.position.x, 0.0f, goal_node.position.y},
                    Vector3{nodes[goal_node.parent].position.x, 0.0f, nodes[goal_node.parent].position.y},
                    GREEN);

            goal_node = nodes[goal_node.parent];
        }
    }

    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const auto& node = nodes[i];
        DrawModel(model, {node.position.x, 0.0f, node.position.y}, 0.2f, PURPLE);  

        DrawLine3D(Vector3{node.position.x, 0.0f, node.position.y},
                   Vector3{nodes[node.parent].position.x, 0.0f, nodes[node.parent].position.y},
                   PURPLE);
    }
}

Mesh GenMeshBorder(std::vector<Vector2> points) {
    Mesh mesh{};

    mesh.triangleCount = points.size() * 2;
    mesh.vertexCount = mesh.triangleCount * 3;

    mesh.vertices = (float *)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
    mesh.normals  = (float *)MemAlloc(mesh.vertexCount * 3 * sizeof(float));

    for (std::size_t i = 0; i < points.size(); ++i) {
        std::size_t next_idx = (i + 1) % points.size();

        mesh.vertices[i * 18] = points[i].x;
        mesh.vertices[i * 18 + 1] = 0.0f;
        mesh.vertices[i * 18 + 2] = points[i].y;

        mesh.vertices[i * 18 + 3] = points[i].x;
        mesh.vertices[i * 18 + 4] = 5.0f;
        mesh.vertices[i * 18 + 5] = points[i].y;

        mesh.vertices[i * 18 + 6] = points[next_idx].x;
        mesh.vertices[i * 18 + 7] = 5.0f;
        mesh.vertices[i * 18 + 8] = points[next_idx].y;

        mesh.vertices[i * 18 + 9] = points[next_idx].x;
        mesh.vertices[i * 18 + 10] = 5.0f;
        mesh.vertices[i * 18 + 11] = points[next_idx].y;

        mesh.vertices[i * 18 + 12] = points[next_idx].x;
        mesh.vertices[i * 18 + 13] = 0.0f;
        mesh.vertices[i * 18 + 14] = points[next_idx].y;

        mesh.vertices[i * 18 + 15] = points[i].x;
        mesh.vertices[i * 18 + 16] = 0.0f;
        mesh.vertices[i * 18 + 17] = points[i].y;
    }

    UploadMesh(&mesh, false);

    return mesh;
}

Model GenModelBorder(std::vector<Vector2> points) {
    Model border = LoadModelFromMesh(GenMeshBorder(points));
    border.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = RED;

    return border;
}

std::vector<path_planner::obstacle> GenObstacles(std::size_t num)
{
    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<float> dis_uni(-38.0f, 38.0f);
    std::normal_distribution<float> dis_nor(10.0f, 1.0f);

    std::vector<path_planner::obstacle> obstacles;
    obstacles.reserve(num);

    for (std::size_t i = 0; i < num; ++i) {
        Vector2 position{dis_uni(gen), dis_uni(gen)};
        float radius = dis_nor(gen);

        obstacles.push_back(path_planner::obstacle{position, radius});
    }

    return obstacles;
}

void DrawObstacles(const std::vector<path_planner::obstacle>& obstacles) {
    for (const auto& obstacle : obstacles) {
        DrawCylinder({obstacle.position.x, 0.0f, obstacle.position.y}, obstacle.radius - 2.0f, obstacle.radius - 2.0f, 5.0f, 16, RED);
    }
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

    // DisableCursor();
    SetTargetFPS(60);

    std::vector<Vector2> borderPoints {
        {-50.0f, -50.0f},
        {-25.0f, -30.0f},
        {50.0f, -50.0f},
        {50.0f, 50.0f},
        {-50.0f, 50.0f},
    };

    Model border = GenModelBorder(borderPoints);
    Model modelNode = LoadModelFromMesh(GenMeshSphere(0.25f, 4, 8));

    constexpr int NUM_OBSTACLES = 16;
    auto obstacles = GenObstacles(NUM_OBSTACLES);

    Vector2 startPos{-45.0f, -45.0f};
    Vector2 goalPos{45.0f, 45.0f};
    auto planner = std::make_unique<path_planner_rrt_star>(startPos, goalPos, 1.0f, 1.0f, borderPoints, obstacles);

    while (!WindowShouldClose())
    {
        UpdateCamera(&camera, CAMERA_THIRD_PERSON);

        if (IsKeyPressed(KEY_SPACE)) {
            obstacles = GenObstacles(NUM_OBSTACLES);
            planner = std::make_unique<path_planner_rrt_star>(startPos, goalPos, 1.0f, 1.0f, borderPoints, obstacles);
        }
        
        if (IsKeyPressed(KEY_ENTER)) {
            const auto start = std::chrono::high_resolution_clock::now();
            planner->step(20000);
            const auto diff = std::chrono::high_resolution_clock::now() - start;
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
            std::cout << "Time taken: " << ms << "ms" << std::endl;
        }

        BeginDrawing();

            ClearBackground(RAYWHITE);

            BeginMode3D(camera);

                rlDisableBackfaceCulling();
                DrawModel(border, {0.0f, 0.0f, 0.0f}, 1.0f, WHITE);
                rlEnableBackfaceCulling();

                DrawObstacles(obstacles);

                rlSetLineWidth(4.0f);
                drawNodes(modelNode, planner->getNodes(), planner->get_goal_node_index());

                DrawGrid(25, 2.0f);

            EndMode3D();
        EndDrawing();
    }

    CloseWindow();

    return 0;
}

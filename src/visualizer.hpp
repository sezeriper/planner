#pragma once

#include <span>

#include <raylib.h>
#include <rlgl.h>

#include "path_planner.hpp"

namespace rota {

class visualizer {
using node = path_planner::node;
using nodes_t = std::span<path_planner::node>;
public:
    visualizer(const std::vector<point>& border_points) {
        model_border = gen_model_border(border_points);
        model_node = LoadModelFromMesh(GenMeshSphere(0.25f, 4, 8));
    }

    void draw_border() const {
        rlDisableBackfaceCulling();
        DrawModel(model_border, {0.0f, 0.0f, 0.0f}, 1.0f, RED);
        rlEnableBackfaceCulling();
    }

    void draw_nodes(const nodes_t nodes, node goal_node) const {
        while (goal_node.parent != nullptr) {
            const auto& parent = *goal_node.parent;
            DrawModel(model_node, {goal_node.position.x, 0.0f, goal_node.position.y}, 3.0f, GREEN);

            DrawLine3D(Vector3{goal_node.position.x, 0.0f, goal_node.position.y},
                    Vector3{parent.position.x, 0.0f, parent.position.y},
                    GREEN);

            goal_node = parent;
        }

        for (std::size_t i = 0; i < nodes.size(); ++i) {
            const auto node = nodes[i];
            if (node.parent == nullptr) continue;

            const auto parent = *node.parent;
            DrawModel(model_node, {node.position.x, 0.0f, node.position.y}, 0.2f, PURPLE);  

            DrawLine3D(Vector3{node.position.x, 0.0f, node.position.y},
                    Vector3{parent.position.x, 0.0f, parent.position.y},
                    PURPLE);
        }
    }

    void draw_obstacles(const std::vector<path_planner::obstacle>& obstacles) const {
        for (const auto& obstacle : obstacles) {
            DrawCylinder({obstacle.position.x, 0.0f, obstacle.position.y}, obstacle.radius - 2.0f, obstacle.radius - 2.0f, 5.0f, 16, RED);
        }
    }

    void draw_path(const std::vector<point>& path) const {
        for (std::size_t i = 0; i < path.size() - 1; ++i) {
            DrawLine3D(Vector3{path[i].x, 0.0f, path[i].y},
                    Vector3{path[i + 1].x, 0.0f, path[i + 1].y},
                    BLUE);
        }
    }

private:
    Mesh gen_mesh_border(const std::vector<point>& points) {
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

    Model gen_model_border(const std::vector<point>& points) {
        Model border = LoadModelFromMesh(gen_mesh_border(points));
        border.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = RED;

        return border;
    }
    
    Model model_border;
    Model model_node;
};
}
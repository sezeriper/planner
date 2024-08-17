#pragma once

#include <span>

#include <raylib.h>
#include <rlgl.h>

#include "path_planner.hpp"

namespace rota {

class visualizer {
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

    void draw_nodes(const std::span<path_planner_rrt_star::node_t> nodes, path_planner_rrt_star::node_t goal_node) const {
        while (goal_node.parent != nullptr) {
            const auto& parent = *goal_node.parent;
            DrawModel(model_node, {goal_node.position.x, 0.0f, goal_node.position.y}, 3.0f, GREEN);

            DrawLine3D(Vector3{goal_node.position.x, 0.0f, goal_node.position.y},
                    Vector3{parent.position.x, 0.0f, parent.position.y},
                    GREEN);

            goal_node = parent;
        }

        // for (std::size_t i = 0; i < nodes.size(); ++i) {
        //     const auto node_t = nodes[i];
        //     if (node_t.parent == nullptr) continue;

        //     const auto parent = *node_t.parent;
        //     DrawModel(model_node, {node_t.position.x, 0.0f, node_t.position.y}, 0.2f, PURPLE);  

        //     DrawLine3D(Vector3{node_t.position.x, 0.0f, node_t.position.y},
        //             Vector3{parent.position.x, 0.0f, parent.position.y},
        //             PURPLE);
        // }
    }

    void draw_nodes(const std::span<path_planner_rrt_star_dubins::node_t> nodes,  real_t rho) const {

        for (std::size_t i = 0; i < nodes.size(); ++i) {
            const auto node_t = nodes[i];
            if (node_t.parent == nullptr) continue;

            const auto parent = *node_t.parent;

            DubinsPath path;
            double q0[] {parent.conf.pos.x, parent.conf.pos.y, parent.conf.angle};
            double q1[] {node_t.conf.pos.x, node_t.conf.pos.y, node_t.conf.angle};
            dubins_shortest_path(&path, q0, q1, rho);
            auto points = sample_path_to_end(path);

            draw_path(points, 0.0f, PURPLE);
        }
    }

    void draw_goal_path(path_planner_rrt_star_dubins::node_t goal_node, real_t rho) const {
        while (goal_node.parent != nullptr) {
            const auto& parent = *goal_node.parent;

            DubinsPath path;
            double q1[] {parent.conf.pos.x, parent.conf.pos.y, parent.conf.angle};
            double q2[] {goal_node.conf.pos.x, goal_node.conf.pos.y, goal_node.conf.angle};
            dubins_shortest_path(&path, q1, q2, rho);
            auto points = sample_path_to_end(path);

            draw_path(points, 0.0f, DARKGREEN);

            goal_node = parent;
        }
    }

    void draw_configuration(configuration conf) {
        DrawModel(model_node, {conf.pos.x, 0.0f, conf.pos.y}, 3.0f, BLUE);

        point p2 {std::cos(conf.angle), std::sin(conf.angle)};
        p2 = scale(p2, 5.0f);
        p2 = add(conf.pos, p2);

        DrawLine3D(Vector3{conf.pos.x, 0.0f, conf.pos.y},
                Vector3{p2.x, 0.0f, p2.y},
                BLUE);
    }

    void draw_obstacles(const std::vector<path_planner::obstacle>& obstacles) const {
        for (const auto& obstacle : obstacles) {
            DrawCylinder({obstacle.position.x, 0.0f, obstacle.position.y}, obstacle.radius - 2.0f, obstacle.radius - 2.0f, 5.0f, 32, BLACK);
        }
    }

    void draw_path(const std::vector<configuration>& path, real_t height = 0.0f, Color color = BLUE) const {
        for (std::size_t i = 0; i < path.size() - 1; ++i) {
            DrawLine3D(Vector3{path[i].pos.x, height, path[i].pos.y},
                    Vector3{path[i + 1].pos.x, height, path[i + 1].pos.y},
                    color);
        }
    }

    template <typename T>
    void draw_grid(const grid_spatial<T>& g) {
        real_t w = g.get_width();
        real_t h = g.get_height();
        real_t cell_size = g.get_cell_size();
        point tl = g.get_top_left();

        for (real_t i = 0.0f; i <= w; ++i) {
            Vector3 start {tl.x + i * cell_size, 0.0f, tl.y};
            Vector3 end {tl.x + i * cell_size, 0.0f, tl.y + h * cell_size};
            DrawLine3D(start, end,
                    BLACK);
        }

        for (real_t i = 0.0f; i <= h; ++i) {
            Vector3 start {tl.x, 0.0f, tl.y + i * cell_size};
            Vector3 end {tl.x + w * cell_size, 0.0f, tl.y + i * cell_size};
            DrawLine3D(start, end,
                    BLACK);
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
        border.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = BLACK;

        return border;
    }
    
    Model model_border;
    Model model_node;
};
}
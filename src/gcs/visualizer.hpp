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

    void draw_path(const DubinsPath& path) const {
        auto traj = sample_path_to_end(path);
        draw_traj_simple(traj, 0.0f, PURPLE);
    }

    void draw_nodes(const std::span<const rrtstar_dubins::node_t> nodes,  real_t rho) const {

        for (std::size_t i = 0; i < nodes.size(); ++i) {
            const auto node = nodes[i];
            draw_path(node.path);
        }
    }

    void draw_configuration(configuration_t conf) {
        DrawModel(model_node, {(conf.x), 0.0f, (conf.y)}, 3.0f, BLUE);

        point p2 {std::cos(conf.yaw), std::sin(conf.yaw)};
        p2 = scale(p2, 5.0f);
        p2 = add(conf.get_point(), p2);

        DrawLine3D(Vector3{conf.x, 0.0f, conf.y},
                Vector3{p2.x, 0.0f, p2.y},
                BLUE);
    }

    void draw_obstacles(const std::vector<path_planner::obstacle>& obstacles) const {
        for (const auto& obstacle : obstacles) {
            DrawCylinder({obstacle.position.x, 0.0f, obstacle.position.y}, obstacle.radius - 2.0f, obstacle.radius - 2.0f, 5.0f, 32, BLACK);
        }
    }

    void draw_traj_simple(const trajectory_t& traj, real_t height = 0.0f, Color color = BLUE) const {
        for (std::size_t i = 0; i < traj.size() - 1; ++i) {
            DrawLine3D(Vector3{traj[i].x, height, traj[i].y},
                    Vector3{traj[i + 1].x, height, traj[i + 1].y},
                    color);
        }
    }

    void draw_traj_smooth(const trajectory_t& path, real_t rho, real_t height = 0.0f, Color color = BLUE) const {
        if (path.size() < 2) return;

        for (std::size_t i = 0; i < path.size() - 1; ++i) {
            auto path_opt = find_path(path[i], path[i + 1], rho);
            if (!path_opt) return;
            const auto path_smooth = sample_path_to_end(path_opt.value());

            draw_traj_simple(path_smooth, height, color);
        }
    }

    void draw_paths(const std::vector<DubinsPath>& paths, real_t height = 0.0f, Color color = BLUE) const {
        for (const auto& path : paths) {
            auto traj = sample_path_to_end(path);
            draw_traj_simple(traj, height, color);
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
            DrawLine3D(start, end, BLACK);
        }

        for (real_t i = 0.0f; i <= h; ++i) {
            Vector3 start {tl.x, 0.0f, tl.y + i * cell_size};
            Vector3 end {tl.x + w * cell_size, 0.0f, tl.y + i * cell_size};
            DrawLine3D(start, end, BLACK);
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
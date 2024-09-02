#pragma once

#include "field.hpp"
#include "dubins/dubins.hpp"
#include "path_planner.hpp"
#include "grid.hpp"

#include <raylib.h>
#include <rlgl.h>

namespace rota {

class visualizer {
public:

    void init(field_t field) {
        model_border = gen_model_border(field.border);
        model_node = LoadModelFromMesh(GenMeshSphere(0.25f, 4, 8));

        for (const auto& obstacle : field.obstacles) {
            model_obstacles.push_back(LoadModelFromMesh(GenMeshCylinder(obstacle.radius, 5.0f, 32)));
        }
    }

    void draw_field(const field_t& field) const {
        draw_border();
        draw_obstacles(field.obstacles);
    }

    void draw_border() const {
        rlDisableBackfaceCulling();
        DrawModel(model_border, {0.0f, 0.0f, 0.0f}, 1.0f, BLACK);
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

        vec2_t p2 {std::cos(conf.yaw), std::sin(conf.yaw)};
        p2 = scale(p2, 5.0f);
        p2 = add(conf.get_point(), p2);

        DrawLine3D(Vector3{conf.x, 0.0f, conf.y},
                Vector3{p2.x, 0.0f, p2.y},
                BLUE);
    }

    void draw_obstacles(const obstacles_t& obstacles) const {
        for (const auto& obstacle : obstacles) {
            DrawCylinder({obstacle.position.x, 0.0f, obstacle.position.y}, obstacle.radius - 2.0f, obstacle.radius, 5.0f, 32, BLACK);
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
        vec2_t tl = g.get_top_left();

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
    Model model_border;
    Model model_node;
    std::vector<Model> model_obstacles;

    Mesh gen_mesh_border(const std::vector<vec2_t>& vec2_ts) {
        Mesh mesh{};

        mesh.triangleCount = vec2_ts.size() * 2;
        mesh.vertexCount = mesh.triangleCount * 3;

        mesh.vertices = (float *)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
        mesh.normals  = (float *)MemAlloc(mesh.vertexCount * 3 * sizeof(float));

        for (std::size_t i = 0; i < vec2_ts.size(); ++i) {
            std::size_t next_idx = (i + 1) % vec2_ts.size();

            mesh.vertices[i * 18] = vec2_ts[i].x;
            mesh.vertices[i * 18 + 1] = 0.0f;
            mesh.vertices[i * 18 + 2] = vec2_ts[i].y;

            mesh.vertices[i * 18 + 3] = vec2_ts[i].x;
            mesh.vertices[i * 18 + 4] = 5.0f;
            mesh.vertices[i * 18 + 5] = vec2_ts[i].y;

            mesh.vertices[i * 18 + 6] = vec2_ts[next_idx].x;
            mesh.vertices[i * 18 + 7] = 5.0f;
            mesh.vertices[i * 18 + 8] = vec2_ts[next_idx].y;

            mesh.vertices[i * 18 + 9] = vec2_ts[next_idx].x;
            mesh.vertices[i * 18 + 10] = 5.0f;
            mesh.vertices[i * 18 + 11] = vec2_ts[next_idx].y;

            mesh.vertices[i * 18 + 12] = vec2_ts[next_idx].x;
            mesh.vertices[i * 18 + 13] = 0.0f;
            mesh.vertices[i * 18 + 14] = vec2_ts[next_idx].y;

            mesh.vertices[i * 18 + 15] = vec2_ts[i].x;
            mesh.vertices[i * 18 + 16] = 0.0f;
            mesh.vertices[i * 18 + 17] = vec2_ts[i].y;
        }

        UploadMesh(&mesh, false);

        return mesh;
    }

    Model gen_model_border(const std::vector<vec2_t>& vec2_ts) {
        Model border = LoadModelFromMesh(gen_mesh_border(vec2_ts));
        border.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = BLACK;

        return border;
    }
};
}
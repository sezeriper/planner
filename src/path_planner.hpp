#pragma once

#include "grid.hpp"
#include "math.hpp"

#include <random>

namespace rota {

class path_planner {
public:
    struct obstacle {
        point position;
        float radius;
    };

    struct node {
        point position;
        const node* parent;
        float cost;
    };

    path_planner() = delete;

    path_planner(
        const point& start,
        const point& goal,
        const float step_size = 1.0f,
        const float goal_radius = 1.0f,
        const std::vector<point>& border = {},
        const std::vector<obstacle>& obstacles = {}
    ) :
        start(start),
        goal(goal),
        step_size(step_size),
        goal_radius(goal_radius),
        border(border),
        obstacles(obstacles),
        gen(std::random_device{}()),
        goal_node_index(0)
    {
        border_min = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        border_max = {std::numeric_limits<float>::min(), std::numeric_limits<float>::min()};

        for (const auto& point : border) {
            if (point.x < border_min.x) border_min.x = point.x;
            if (point.y < border_min.y) border_min.y = point.y;
            if (point.x > border_max.x) border_max.x = point.x;
            if (point.y > border_max.y) border_max.y = point.y;
        }

        dis_uni_x = std::uniform_real_distribution<float>(border_min.x, border_max.x);
        dis_uni_y = std::uniform_real_distribution<float>(border_min.y, border_max.y);
    }

protected:
    const point start;
    const point goal;
    float step_size;
    float goal_radius;
    std::vector<point> border;
    std::vector<obstacle> obstacles;
    std::mt19937 gen;
    std::size_t goal_node_index;

    point border_min;
    point border_max;
    std::uniform_real_distribution<float> dis_uni_x;
    std::uniform_real_distribution<float> dis_uni_y;


    bool is_colliding(const point& point) {
        bool collision = !check_collision(point, border);
        
        if (collision) return collision;

        for (const auto& obstacle : obstacles) {
            if (distance_sqr(point, obstacle.position) < (obstacle.radius * obstacle.radius)) {
                collision = true;
                break;
            }
        }

        return collision;
    }
};

class path_planner_rrt_star : public path_planner {
public:
    path_planner_rrt_star(
        const point& start,
        const point& goal,
        const float step_size = 2.0f,
        const float goal_radius = 1.0f,
        const std::vector<point>& border = {},
        const std::vector<obstacle>& obstacles = {},
        const float neighbour_radius = 4.0f
    ) :
        path_planner(start, goal, step_size, goal_radius, border, obstacles),
        neighbour_radius(neighbour_radius),
        root{start, nullptr, 0.0f},
        goal_node{},
        nodes(border_min, border_max, neighbour_radius*2.0f, 512)
    {
        nodes.add_data(start, root);
    }

    void step(std::size_t num_steps = 1) {
        for (std::size_t i = 0; i < num_steps;) {
            point random_point{dis_uni_x(gen), dis_uni_y(gen)};
            if (is_colliding(random_point)) { continue; }
            point nearest_position = nearest_node_position(random_point);
            point direction = normalize(subtract(random_point, nearest_position));
            point new_point = add(nearest_position, scale(direction, step_size));
            if (is_colliding(new_point)) { continue; }

            const auto neighbours = find_neighbours(new_point);
            const node* parent = find_best_parent(new_point, neighbours);

            node new_node;
            new_node.parent = parent;
            new_node.position = new_point;
            new_node.cost = cost(new_point, *parent);
            const node* new_node_ptr = nodes.add_data(new_node.position, new_node);
            ++i;

            if (distance_sqr(new_point, goal) < goal_radius * goal_radius) {
                if (goal_node.parent == nullptr || new_node.cost < goal_node.cost) {
                    goal_node = new_node;
                }
            }

            for (const auto& neighbour : neighbours) {
                float new_cost = cost(neighbour->position, new_node);

                if (new_cost < neighbour->cost) {
                    neighbour->parent = new_node_ptr;
                    neighbour->cost = new_cost;
                }
            }
        }
    }

    static constexpr real_t cost(const point& p, const node& parent) {
        return parent.cost + distance(p, parent.position);
    }

    const std::span<node> get_nodes() const {
        return nodes.get_data();
    }

    const node& get_goal_node() const {
        return goal_node;
    }

private:
    float neighbour_radius;
    node root;
    node goal_node;
    grid_spatial<node> nodes;
    
    
    point nearest_node_position(point p) const {
        float min_distance = std::numeric_limits<float>::max();
        point nearest_pos;

        int dist = 0;
        const auto center = nodes.get_coord(p);
        while (min_distance == std::numeric_limits<float>::max()) {
            for (int y = -dist; y <= dist; ++y) {
                for (int x = -dist; x <= dist; ++x) {
                    if (x == -dist || x == dist || y == -dist || y == dist) {
                        grid_spatial<node>::coord cell_coord{center.x + x, center.y + y};
                        if (cell_coord.x < 0 || cell_coord.x >= nodes.get_width() || cell_coord.y < 0 || cell_coord.y >= nodes.get_height()) {
                            continue;
                        }
                        auto cell = nodes.get_cell(cell_coord.x, cell_coord.y);
                        for (int i = 0; i < *cell.size; ++i) {
                            point pos = cell.data[i].position;
                            float distance = distance_sqr(pos, p);
                            if (distance < min_distance) {
                                min_distance = distance;
                                nearest_pos = pos;
                            }
                        }
                    }
                }
            }
            ++dist;
        }
        return nearest_pos;
    }

    std::vector<node*> find_neighbours(point p) const {
        std::vector<node*> neighbours;
        neighbours.reserve(nodes.get_capacity() * 9);

        const auto center = nodes.get_coord(p);
        for (int y = -1; y <= 1; ++y) {
            for (int x = -1; x <= 1; ++x) {
                const grid_spatial<node>::coord cell_coord{center.x + x, center.y + y};
                if (cell_coord.x < 0 || cell_coord.x >= nodes.get_width() || cell_coord.y < 0 || cell_coord.y >= nodes.get_height()) {
                    continue;
                }
                auto cell = nodes.get_cell(center.x + x, center.y + y);

                for (int i = 0; i < *cell.size; ++i) {
                    point pos = cell.data[i].position;
                    if (distance_sqr(pos, p) < neighbour_radius * neighbour_radius) {
                        neighbours.push_back(cell.data + i);
                    }
                }
            }
        }

        return neighbours;
    }

    node* find_best_parent(const point& point, const std::vector<node*>& neighbours) const {
        node* best_parent;
        float min_cost = std::numeric_limits<float>::max();

        for (const auto& neighbour : neighbours) {
            float dist = distance(neighbour->position, point);
            float cost = neighbour->cost + dist;

            if (cost < min_cost) {
                best_parent = neighbour;
                min_cost = cost;
            }
        }

        return best_parent;
    }
};
}

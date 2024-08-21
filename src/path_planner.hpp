#pragma once

#include "grid.hpp"
#include "math.hpp"
#include "dubins/dubins.hpp"

#include <random>
#include <iostream>
#include <utility>
#include <tuple>
#include <algorithm>
#include <numbers>
#include <stack>
#include <cstdlib>

namespace rota {

struct configuration {
    point pos;
    real_t angle;
};

constexpr bool operator==(const configuration& lhs, const configuration& rhs) {
    return lhs.pos == rhs.pos && lhs.angle == rhs.angle;
}

static std::optional<DubinsPath> find_path(const configuration from, const configuration to, real_t rho) {
    DubinsPath path;

    real_t q0[] {from.pos.x, from.pos.y, from.angle};
    real_t q1[] {to.pos.x, to.pos.y, to.angle};
    bool result = dubins_shortest_path(path, q0, q1, rho);
    if (result == 0) {
        return path;
    }
    return std::nullopt;
}

static real_t find_path_length(const configuration from, const configuration to, real_t rho) {
    const auto path_opt = find_path(from, to, rho);
    if (!path_opt) return std::numeric_limits<real_t>::infinity();
    const auto path = path_opt.value();
    return dubins_path_length(path);
}

static std::vector<configuration> sample_path_to_end(DubinsPath& path, real_t interval = 0.1f) {
    std::vector<configuration> path_points;

    real_t q[3];
    real_t length = dubins_path_length(path);
    for (real_t t = 0.0f; t < length; t += interval) {
        int result = dubins_path_sample(path, t, q);
        if (result != 0) break;
        path_points.push_back({ {q[0], q[1]}, q[2] });
    }

    dubins_path_endpoint(path, q);
    path_points.push_back({ {q[0], q[1]}, q[2] });

    return path_points;
}

static std::vector<configuration> sample_path_to_length(DubinsPath& path, real_t length, real_t interval = 0.1f) {
    std::vector<configuration> path_points;

    real_t path_length = dubins_path_length(path);

    if ((path_length - length) < EPSILON) {
        return sample_path_to_end(path, interval);
    }

    real_t q[3];
    for (real_t t = 0.0f; t < length; t += interval) {
        int result = dubins_path_sample(path, t, q);
        if (result != 0) break;
        path_points.push_back({ {q[0], q[1]}, q[2] });
    }

    dubins_path_sample(path, length, q);

    return path_points;
}

class path_planner {
public:
    struct obstacle {
        point position;
        real_t radius;
    };

    path_planner() = delete;

    path_planner(
        point start,
        point goal,
        real_t step_size,
        real_t goal_radius,
        const std::vector<point>& border,
        const std::vector<obstacle>& obstacles
    ) :
        start(start),
        goal(goal),
        step_size(step_size),
        goal_radius(goal_radius),
        border(border),
        obstacles(obstacles),
        border_min(find_border_min()),
        border_max(find_border_max()),
        dis_real(0.0f, 1.0f),
        dis_int(),
        gen(std::random_device{}())
    {}

protected:
    const point start;
    const point goal;
    const real_t step_size;
    const real_t goal_radius;
    const std::vector<point> border;
    const std::vector<obstacle> obstacles;
    const point border_min;
    const point border_max;
    std::uniform_real_distribution<real_t> dis_real;
    std::uniform_int_distribution<int> dis_int;
    std::mt19937 gen;

    constexpr bool is_colliding(const point& point) const {
        if(!is_inside(point, border))
            return true;

        for (const auto& obstacle : obstacles) {
            if (distance_sqr(point, obstacle.position) < (obstacle.radius * obstacle.radius)) {
                return true;
            }
        }
        return false;
    }

    constexpr int get_rand_int(int start, int end) {
        dis_int.param(std::uniform_int_distribution<int>::param_type(start, end));
        return dis_int(gen);
    }

    constexpr real_t get_rand_real() { return dis_real(gen); }

    constexpr real_t get_rand_between(real_t start, real_t end) {
        return (get_rand_real() * (end - start)) + start;
    }

    constexpr real_t get_rand_angle() { return get_rand_between(0.0, std::numbers::pi_v<real_t> * 2.0); }

    constexpr point get_rand_point() {
        return {get_rand_between(border_min.x, border_max.x),
                get_rand_between(border_min.y, border_max.y)};
    }

private:
    constexpr point find_border_min() const {
        point min {std::numeric_limits<real_t>::max(), std::numeric_limits<real_t>::max()};

        for (const auto& point : border) {
            if (point.x < min.x) min.x = point.x;
            if (point.y < min.y) min.y = point.y;
        }

        return min;
    }

    constexpr point find_border_max() const {
        point max {std::numeric_limits<real_t>::min(), std::numeric_limits<real_t>::min()};

        for (const auto& point : border) {
            if (point.x > max.x) max.x = point.x;
            if (point.y > max.y) max.y = point.y;
        }

        return max;
    }
};

class path_planner_rrt_star : public path_planner {
public:

    struct node_t {
        const node_t* parent;
        real_t cost;
        point position;
    };

    path_planner_rrt_star(
        const point& start,
        const point& goal,
        const real_t step_size = 2.0f,
        const real_t goal_radius = 1.0f,
        const std::vector<point>& border = {},
        const std::vector<obstacle>& obstacles = {},
        const real_t near_radius = 4.0f
    ) :
        path_planner(start, goal, step_size, goal_radius, border, obstacles),
        near_radius(near_radius),
        root{nullptr, 0.0f, start},
        goal_node{},
        nodes(border_min, border_max, near_radius*2.0f, 512)
    {
        nodes.add_data(start, root);
    }

    void step(std::size_t num_steps = 1) {
        for (std::size_t i = 0; i < num_steps;) {
            point rand_conf{get_rand_point()};
            if (is_colliding(rand_conf)) { continue; }
            point nearest_position = nearest_node_position(rand_conf);
            point direction = normalize(subtract(rand_conf, nearest_position));
            point new_p = add(nearest_position, scale(direction, step_size));
            if (is_colliding(new_p)) { continue; }

            const auto neighbours = find_neighbours(new_p);
            const node_t* parent = find_best_parent(new_p, neighbours);

            const node_t& new_node = nodes.add_data(new_p, {
                parent,
                cost(new_p, *parent),
                new_p
            });
            ++i;

            if (distance_sqr(new_p, goal) < goal_radius * goal_radius) {
                if (goal_node.parent == nullptr || new_node.cost < goal_node.cost) {
                    goal_node = new_node;
                }
            }

            for (const auto& neighbour : neighbours) {
                real_t new_cost = cost(neighbour->position, new_node);

                if (new_cost < neighbour->cost) {
                    neighbour->parent = &new_node;
                    neighbour->cost = new_cost;
                }
            }
        }
    }

    static constexpr real_t cost(const point& p, const node_t& parent) {
        return parent.cost + distance(p, parent.position);
    }

    const std::span<node_t> get_nodes() const {
        return nodes.get_data();
    }

    const node_t& get_goal_node() const {
        return goal_node;
    }

private:
    real_t near_radius;
    node_t root;
    node_t goal_node;
    grid_spatial<node_t> nodes;
    
    
    point nearest_node_position(point p) const {
        real_t min_distance = std::numeric_limits<real_t>::max();
        point nearest_pos;

        int dist = 0;
        const auto center = nodes.get_coord(p);
        while (min_distance == std::numeric_limits<real_t>::max()) {
            for (int y = -dist; y <= dist; ++y) {
                for (int x = -dist; x <= dist; ++x) {
                    if (x == -dist || x == dist || y == -dist || y == dist) {
                        grid_spatial<node_t>::coord cell_coord{center.x + x, center.y + y};
                        if (cell_coord.x < 0 || cell_coord.x >= nodes.get_width() || cell_coord.y < 0 || cell_coord.y >= nodes.get_height()) {
                            continue;
                        }
                        auto cell = nodes.get_cell(cell_coord.x, cell_coord.y);
                        for (int i = 0; i < *cell.size; ++i) {
                            point pos = cell.data[i].position;
                            real_t distance = distance_sqr(pos, p);
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

    std::vector<node_t*> find_neighbours(point p) const {
        std::vector<node_t*> neighbours;
        neighbours.reserve(nodes.get_capacity() * 9);

        const auto center = nodes.get_coord(p);
        for (int y = -1; y <= 1; ++y) {
            for (int x = -1; x <= 1; ++x) {
                const grid_spatial<node_t>::coord cell_coord{center.x + x, center.y + y};
                if (cell_coord.x < 0 || cell_coord.x >= nodes.get_width() || cell_coord.y < 0 || cell_coord.y >= nodes.get_height()) {
                    continue;
                }
                auto cell = nodes.get_cell(center.x + x, center.y + y);

                for (int i = 0; i < *cell.size; ++i) {
                    point pos = cell.data[i].position;
                    if (distance_sqr(pos, p) < near_radius * near_radius) {
                        neighbours.push_back(cell.data + i);
                    }
                }
            }
        }

        return neighbours;
    }

    node_t* find_best_parent(const point& point, const std::vector<node_t*>& neighbours) const {
        node_t* best_parent;
        real_t min_cost = std::numeric_limits<real_t>::max();

        for (const auto& neighbour : neighbours) {
            real_t dist = distance(neighbour->position, point);
            real_t cost = neighbour->cost + dist;

            if (cost < min_cost) {
                best_parent = neighbour;
                min_cost = cost;
            }
        }

        return best_parent;
    }
};

class path_planner_rrt_star_dubins : public path_planner {
public:

    struct node_t {
        const node_t* parent;
        real_t cost;
        configuration conf;
    };

    path_planner_rrt_star_dubins(
        const configuration& start,
        const configuration& goal,
        real_t step_size,
        real_t goal_radius,
        real_t near_radius,
        real_t rho,
        const std::vector<point>& border,
        const std::vector<obstacle>& obstacles
    ) :
        path_planner(start.pos, goal.pos, step_size, goal_radius, border, obstacles),
        rho{rho},
        goal_node{nullptr, std::numeric_limits<real_t>::infinity(), goal},
        nodes(border_min, border_max, 4.0f, 1024),
        root_ptr{&nodes.add_data(start.pos, {nullptr, 0.0f, start})},
        node_list{root_ptr},
        near_radius{near_radius}
    {}

    void step() {
        auto [node_ptr, neighbours] = add_node();
        if (!node_ptr) return;

        const node_t& node_t = *node_ptr;
        update_neighbours(node_t, neighbours);
    }

    void run_for_ms(std::chrono::milliseconds ms, int batch_size = 200) {
        auto start = std::chrono::high_resolution_clock::now();
        while (true) {
            for (int i = 0; i < batch_size; ++i) {
                step();
            }
            auto now = std::chrono::high_resolution_clock::now();
            if (now - start > ms) break;
        }
        try_reach_goal();
    }

    std::vector<configuration> get_path_to_goal(real_t interval) const {
        std::vector<configuration> confs;
        const node_t* current = &goal_node;

        while (current != nullptr) {
            confs.push_back(current->conf);
            current = current->parent;
        }

        std::reverse(confs.begin(), confs.end());

        real_t rem = 0.0f;
        std::vector<configuration> path;
        for (std::size_t i = 0; i < confs.size() - 1; ++i) {
            const auto path_opt = find_path(confs[i], confs[i + 1], rho);
            if (!path_opt) return {};

            DubinsPath dubins_path = path_opt.value();
            const real_t path_length = dubins_path_length(dubins_path);

            real_t t = rem;
            while (t < path_length) {
                real_t q[3];
                dubins_path_sample(dubins_path, t, q);
                path.push_back({{q[0], q[1]}, q[2]});
                t += interval;
            }

            rem = t - path_length;
        }

        path.push_back(confs.back());

        return path;
    }

    std::vector<configuration> simplify_path(std::vector<configuration> path, int max_iter = 10000) {
        for (int i = 0; i < max_iter; ++i) {
            if (path.size() < 3) return path;

            int a = get_rand_int(0, path.size() - 3);
            int b = get_rand_int(2, path.size() - 1);

            while (b - a < 2) {
                b = get_rand_int(2, path.size() - 1);
            }

            const auto conf1 = path[a];
            const auto conf2 = path[b];

            const auto path_opt = find_path(conf1, conf2, rho);
            if (!path_opt) return {};

            DubinsPath dubins_path = path_opt.value();
            const auto sub_path = sample_path_to_end(dubins_path);
            if (is_colliding(sub_path)) continue;

            path.erase(path.begin() + a + 1, path.begin() + b);
        }
        return path;
    }

    int get_node_count() const { return node_list.size(); }

    const std::span<node_t> get_nodes() const {
        return nodes.get_data();
    }

    const node_t& get_goal_node() const {
        return goal_node;
    }

    const grid_spatial<node_t>& get_grid() const {
        return nodes;
    }

private:
    const real_t rho;
    node_t goal_node;
    grid_spatial<node_t> nodes;
    node_t* root_ptr;
    std::vector<node_t*> node_list;
    const real_t near_radius;

    void try_reach_goal() {
        const auto cells = nodes.get_cells_in_radius(goal_node.conf.pos, near_radius);

        for (const auto cell : cells) {
            for (int i = 0; i < *cell.size; ++i) {
                const node_t& n = cell.data[i];

                const auto path_opt = find_path(n.conf, goal_node.conf, rho);
                if (!path_opt) continue;

                DubinsPath path = path_opt.value();
                real_t path_length = dubins_path_length(path);
                
                if (path_length > goal_radius) continue;

                const auto path_points = sample_path_to_end(path);
                if (is_colliding(path_points)) continue;

                real_t cost = n.cost + path_length;
                if (cost < goal_node.cost) {
                    goal_node.parent = &n;
                    goal_node.cost = cost;
                }
            }
        }
    }

    std::tuple<const node_t*, std::vector<node_t*>> add_node() {
        configuration rand_conf {get_rand_point(), get_rand_angle()};
        if (path_planner::is_colliding(rand_conf.pos)) return {};

        const node_t* nearest = nearest_node(rand_conf);

        const auto path_opt = find_path(nearest->conf, rand_conf, rho);
        if (!path_opt) return {};

        auto path = path_opt.value();
        auto path_points = sample_path_to_length(path, step_size);
        if (is_colliding(path_points)) return {};

        auto new_conf = path_points.back();
        const auto neighbours = find_neighbours(new_conf.pos);
        if (is_colliding_with_neighbours(new_conf, neighbours)) return {};

        const auto [parent, cost] = find_best_parent(new_conf, neighbours);

        if (parent == nullptr ||
            cost == std::numeric_limits<real_t>::infinity()) return {};

        node_t& new_node = nodes.add_data(new_conf.pos, {
            parent,
            cost,
            new_conf
        });
        node_list.push_back(&new_node);

        return {&new_node, neighbours};
    }

    bool is_colliding_with_neighbours(const configuration& conf, const std::vector<node_t*>& neighbours) const {
        for (const auto neighbour : neighbours) {
            if (neighbour->conf == conf) {
                return true;
            }
        }
        return false;
    }

    void update_neighbours(const node_t& new_n, const std::vector<node_t*> neighbours) {
        for (const auto neighbour : neighbours) {
            auto path_opt = find_path(new_n.conf, neighbour->conf, rho);
            if (!path_opt) continue;

            DubinsPath path = path_opt.value();

            real_t path_length = dubins_path_length(path);
            if (path_length > step_size) continue;

            real_t new_cost = new_n.cost + path_length;
            if (new_cost >= neighbour->cost) continue;

            const auto path_points = sample_path_to_end(path);
            if (is_colliding(path_points)) continue;

            propagate_cost(neighbour, neighbour->cost - new_cost);

            neighbour->parent = &new_n;
            neighbour->cost = new_cost;
        }
    }

    void propagate_cost(const node_t* parent, real_t cost_dif) {
        auto children = get_children(parent);
        for (node_t* child : children) {
            child->cost -= cost_dif;
        }
    }

    std::vector<node_t*> get_children(const node_t* parent_node) const {
        std::vector<node_t*> children;
        std::stack<const node_t*> stack;
        stack.push(parent_node);

        while (!stack.empty()) {
            const node_t* parent = stack.top();
            stack.pop();

            for (node_t* node : node_list) {
                if (node->parent == parent) {
                    children.push_back(node);
                    stack.push(node);
                }
            }
        }

        return children;
    }

    constexpr bool is_colliding(const std::vector<configuration>& path_points) const {
        for (const auto config : path_points) {
            bool collision = path_planner::is_colliding(config.pos);
            if (collision) {
                return true;
            }
        }
        return false;
    }

    const node_t* nearest_node(const configuration& c) const {
        real_t min_dist = std::numeric_limits<real_t>::infinity();
        const node_t* nearest;

        const int min_search_dist = std::ceil(near_radius / nodes.get_cell_size()) + 1;

        int dist = 0;
        const auto center = nodes.get_coord(c.pos);
        while (min_dist == std::numeric_limits<real_t>::infinity() || dist < min_search_dist) {
            for (int y = -dist; y <= dist; ++y) {
                for (int x = -dist; x <= dist; ++x) {
                    grid_spatial<node_t>::coord cell_coord {center.x + x, center.y + y};

                    cell_coord.x = std::clamp(cell_coord.x, 0ul, nodes.get_width() - 1ul);
                    cell_coord.y = std::clamp(cell_coord.y, 0ul, nodes.get_height() - 1ul);
                    if (cell_coord.x != center.x - dist &&
                        cell_coord.x != center.x + dist &&
                        cell_coord.y != center.y - dist &&
                        cell_coord.y != center.y + dist) continue;

                    auto cell = nodes.get_cell(cell_coord.x, cell_coord.y);
                    for (int i = 0; i < *cell.size; ++i) {
                        const node_t& n = cell.data[i];
                        real_t dist = find_path_length(n.conf, c, rho);
                        // real_t dist = distance_sqr(n.conf.pos, c.pos);
                        if (dist < min_dist) {
                            min_dist = dist;
                            nearest = &n;
                        }
                    }
                }
            }
            ++dist;
        }
        return nearest;
    }

    std::vector<node_t*> find_neighbours(point p) const {
        std::vector<node_t*> neighbours;

        const auto cells = nodes.get_cells_in_radius(p, near_radius);
        for (const auto cell : cells) {
            for (int i = 0; i < *cell.size; ++i) {
                point pos = cell.data[i].conf.pos;
                if (distance_sqr(pos, p) < (near_radius * near_radius)) {
                    neighbours.push_back(cell.data + i);
                }
            }
        }

        return neighbours;
    }

    std::tuple<const node_t*, real_t> find_best_parent(const configuration& conf, const std::vector<node_t*>& neighbours) const {
        const node_t* best_parent = nullptr;
        real_t min_cost = std::numeric_limits<real_t>::infinity();

        for (const node_t* neighbour : neighbours) {
            const auto path_opt = find_path(neighbour->conf, conf, rho);
            if (!path_opt) continue;

            DubinsPath path = path_opt.value();
            real_t path_length = dubins_path_length(path);
            if (path_length > near_radius) continue;

            const auto path_points = sample_path_to_end(path);
            if (is_colliding(path_points)) continue;

            real_t cost = neighbour->cost + path_length;

            if (cost < min_cost) {
                best_parent = neighbour;
                min_cost = cost;
            }
        }

        
        return {best_parent, min_cost};
    }
};
}

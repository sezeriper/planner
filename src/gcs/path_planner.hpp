#pragma once

#include "../math.hpp"
#include "field.hpp"
#include "../dubins/dubins.hpp"

#include <nanoflann.hpp>
#include <spdlog/spdlog.h>

#include <random>
#include <algorithm>
#include <cstdlib>
#include <span>

namespace rota {
template <
    class T, class DataSource, typename _DistanceType = T,
    typename IndexType = uint32_t>
struct custom_metric_adaptor
{
    using ElementType  = T;
    using DistanceType = _DistanceType;

    const DataSource& data_source;

    custom_metric_adaptor(const DataSource& _data_source)
        : data_source(_data_source)
    {
    }

    DistanceType evalMetric(
        const T* a, const IndexType b_idx, size_t size) const
    {
        DistanceType result = DistanceType();
        for (size_t i = 0; i < 3; ++i)
        {
            result += accum_dist(a[i], data_source.kdtree_get_pt(b_idx, i), i);
        }

        return result;
    }

    template <typename U, typename V>
    DistanceType accum_dist(const U a, const V b, const size_t i) const
    {
        return (a - b) * (a - b);
    }
};

template <typename T>
struct point_cloud {
    std::vector<configuration_t> points;
    std::vector<T> nodes;

    point_cloud() {
        points.reserve(50000);
        nodes.reserve(50000);
    }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return points.size(); }

    // Returns the dim'th component of the idx'th vec2_t in the class:
    // Since this is inlined and the "dim" argument is typically an immediate
    // value, the
    //  "if/else's" are actually solved at compile time.
    inline real_t kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return points[idx].x;
        else if (dim == 1)
            return points[idx].y;
        else
            return points[idx].yaw;
    }

    // Optional bounding-box computation: return false to default to a standard
    // bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned
    //   in "bb" so it can be avoided to redo it again. Look at bb.size() to
    //   find out the expected dimensionality (e.g. 2 or 3 for vec2_t clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const
    {
        return false;
    }

    T& add_data(const configuration_t& p, T data) {
        points.push_back({p.x, p.y, p.yaw});
        nodes.push_back(data);
        return nodes.back();
    }
};

class path_planner {
public:
    path_planner() = delete;

    path_planner(
        vec2_t start,
        vec2_t goal,
        real_t step_size,
        real_t goal_radius,
        const std::vector<vec2_t>& border,
        const std::vector<obstacle_t>& obstacles
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
    const vec2_t start;
    const vec2_t goal;
    const real_t step_size;
    const real_t goal_radius;
    const std::vector<vec2_t> border;
    const std::vector<obstacle_t> obstacles;
    const vec2_t border_min;
    const vec2_t border_max;
    std::uniform_real_distribution<real_t> dis_real;
    std::uniform_int_distribution<int> dis_int;
    std::mt19937 gen;

    constexpr bool is_colliding(const vec2_t& vec2_t) const {
        if(!is_inside(vec2_t, border))
            return true;

        for (const auto& obstacle_t : obstacles) {
            if (distance_sqr(vec2_t, obstacle_t.position) < (obstacle_t.radius * obstacle_t.radius)) {
                return true;
            }
        }
        return false;
    }

    int get_rand_int(int start, int end) {
        dis_int.param(std::uniform_int_distribution<int>::param_type(start, end));
        return dis_int(gen);
    }

    real_t get_rand_real() { return dis_real(gen); }

    real_t get_rand_between(real_t start, real_t end) {
        return (get_rand_real() * (end - start)) + start;
    }

    real_t get_rand_angle() { return get_rand_between(0.0f, std::numbers::pi_v<real_t> * 2.0f); }

    vec2_t get_rand_vec2_t() {
        return {get_rand_vec2_t_x(), get_rand_vec2_t_y()};
    }

    real_t get_rand_vec2_t_x() {
        return get_rand_between(border_min.x, border_max.x);
    }

    real_t get_rand_vec2_t_y() {
        return get_rand_between(border_min.y, border_max.y);
    }

private:
    constexpr vec2_t find_border_min() const {
        vec2_t min {std::numeric_limits<real_t>::max(), std::numeric_limits<real_t>::max()};

        for (const auto& vec2_t : border) {
            if (vec2_t.x < min.x) min.x = vec2_t.x;
            if (vec2_t.y < min.y) min.y = vec2_t.y;
        }

        return min;
    }

    constexpr vec2_t find_border_max() const {
        vec2_t max {std::numeric_limits<real_t>::min(), std::numeric_limits<real_t>::min()};

        for (const auto& vec2_t : border) {
            if (vec2_t.x > max.x) max.x = vec2_t.x;
            if (vec2_t.y > max.y) max.y = vec2_t.y;
        }

        return max;
    }
};


class rrtstar_dubins : public path_planner {
public:

    struct node_t {
        const node_t* parent;
        real_t cost;
        configuration_t conf;
        DubinsPath path;
    };

    using node_list_t = std::vector<node_t*>;

    using kd_tree_t = nanoflann::KDTreeSingleIndexDynamicAdaptor<
        custom_metric_adaptor<real_t, point_cloud<node_t>>,
        point_cloud<node_t>, 3 /* dim */
    >;

    rrtstar_dubins(
        const configuration_t& start,
        const configuration_t& goal,
        real_t step_size,
        real_t goal_radius,
        real_t near_radius,
        real_t rho,
        const std::vector<vec2_t>& border,
        const std::vector<obstacle_t>& obstacles
    ) :
        path_planner(start.get_point(), goal.get_point(), step_size, goal_radius, border, obstacles),
        rho{rho},
        goal_node{nullptr, std::numeric_limits<real_t>::max(), goal},
        cloud{},
        index(3, cloud, { 10 }),
        root_ptr{&cloud.add_data(start, {nullptr, 0.0f, start})},
        near_radius{near_radius},
        near_radius_sqr{near_radius * near_radius}
    {
        index.addPoints(0, 0);
    }

    bool step() {
        auto [node_ptr, neighbours] = add_node();
        if (!node_ptr) return false;

        const node_t& node_t = *node_ptr;
        update_neighbours(node_t, neighbours);
        return true;
    }

    void run_for_iter(int iter) {
        for (int i = 0; i < iter;) {
            if (step()) ++i;
        }
        try_reach_goal();
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

    std::vector<DubinsPath> get_paths_to_goal() const {
        std::vector<DubinsPath> paths;
        const node_t* current = &goal_node;

        while (current != nullptr) {
            paths.push_back(current->path);
            current = current->parent;
        }

        std::reverse(paths.begin(), paths.end());
        return paths;
    }

    trajectory_t simplify_path(trajectory_t path, int max_iter = 10000) {
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

    int get_node_count() const { return cloud.nodes.size(); }

    const std::span<const node_t> get_nodes() const {
        return {cloud.nodes.data(), cloud.nodes.size()};
    }

    const node_t& get_goal_node() const {
        return goal_node;
    }

private:
    const real_t rho;
    node_t goal_node;
    point_cloud<node_t> cloud;
    kd_tree_t index;
    node_t* root_ptr;
    const real_t near_radius;
    const real_t near_radius_sqr;

    void try_reach_goal() {
        std::vector<nanoflann::ResultItem<size_t, real_t>> indices;
        nanoflann::RadiusResultSet<real_t> results(goal_radius * goal_radius, indices);

        index.findNeighbors(results, &goal_node.conf.x);

        for (const auto& result : indices) {
            const node_t& n = cloud.nodes[result.first];

            const auto path_opt = find_path(n.conf, goal_node.conf, rho);
            if (!path_opt) continue;

            DubinsPath path = path_opt.value();
            real_t path_length = dubins_path_length(path);
            if (path_length > goal_radius) continue;

            real_t cost = n.cost + path_length;
            if (cost >= goal_node.cost) continue;

            const auto path_points = sample_path_to_end(path);
            if (is_colliding(path_points)) continue;

            goal_node.parent = &n;
            goal_node.cost = cost;
            goal_node.path = path;
        }
    }

    std::tuple<const node_t*, node_list_t> add_node() {
        configuration_t rand_conf {get_rand_vec2_t_x(), get_rand_vec2_t_y(), get_rand_angle()};
        if (path_planner::is_colliding(rand_conf.get_point())) return {};

        const node_t* nearest = nearest_node(rand_conf);

        auto path_opt = steer(nearest->conf, rand_conf);
        if (!path_opt) return {};

        auto path = path_opt.value();
        auto path_points = sample_path_to_end(path);
        if (is_colliding(path_points)) return {};

        configuration_t new_conf = path_points.back();

        const auto neighbours = find_neighbours(new_conf);

        auto [parent, cost, new_path] = find_best_parent(new_conf, neighbours, nearest, path, nearest->cost + dubins_path_length(path));

        node_t& new_node = cloud.add_data(new_conf, {
            parent,
            cost,
            new_conf,
            new_path
        });

        index.addPoints(cloud.nodes.size()-1, cloud.nodes.size()-1);

        return {&new_node, neighbours};
    }

    std::optional<DubinsPath> steer(const configuration_t& from, const configuration_t& to) {
        const auto path_opt = find_path(from, to, rho);
        if (!path_opt) return std::nullopt;

        DubinsPath path = path_opt.value();
        if (dubins_path_length(path) <= step_size) return path;

        DubinsPath subpath;
        int result = dubins_extract_subpath(path, step_size, &subpath);
        if (result != 0) return std::nullopt;
        return subpath;
    }

    void update_neighbours(const node_t& new_n, const node_list_t neighbours) {
        for (const auto neighbour : neighbours) {
            auto path_opt = find_path(new_n.conf, neighbour->conf, rho);
            if (!path_opt) continue;

            DubinsPath path = path_opt.value();

            real_t path_length = dubins_path_length(path);
            real_t new_cost = new_n.cost + path_length;
            if (new_cost >= neighbour->cost) continue;

            const auto path_points = sample_path_to_end(path);
            if (is_colliding(path_points)) continue;

            propagate_cost(neighbour, neighbour->cost - new_cost);

            neighbour->parent = &new_n;
            neighbour->cost = new_cost;
            neighbour->path = path;
        }
    }

    void propagate_cost(const node_t* parent, real_t cost_dif) {
        auto children = get_children(parent);
        for (node_t* child : children) {
            child->cost -= cost_dif;
        }
    }

    node_list_t get_children(const node_t* parent_node) {
        node_list_t children;
        std::stack<const node_t*> stack;
        stack.push(parent_node);

        while (!stack.empty()) {
            const node_t* parent = stack.top();
            stack.pop();

            for (node_t& node : cloud.nodes) {
                if (node.parent == parent) {
                    children.push_back(&node);
                    stack.push(&node);
                }
            }
        }

        return children;
    }

    constexpr bool is_colliding(const trajectory_t& path_points) const {
        for (const auto config : path_points) {
            bool collision = path_planner::is_colliding(config.get_point());
            if (collision) {
                return true;
            }
        }
        return false;
    }

    const node_t* nearest_node(const configuration_t& c) const {
        size_t ret_index;
        real_t dist_sqr;
        nanoflann::KNNResultSet<real_t> results(1 /* number of results */);
        results.init(&ret_index, &dist_sqr);
        index.findNeighbors(results, &c.x);

        return &cloud.nodes[ret_index];
    }

    node_list_t find_neighbours(const configuration_t& c) {
        node_list_t neighbours;

        std::vector<nanoflann::ResultItem<size_t, real_t>> indices;
        nanoflann::RadiusResultSet<real_t> results(near_radius_sqr, indices);

        index.findNeighbors(results, &c.x);

        for (const auto& result : indices) {
            neighbours.push_back(&cloud.nodes[result.first]);
        }

        return neighbours;
    }

    std::tuple<const node_t*, real_t, DubinsPath> find_best_parent(const configuration_t& conf, const node_list_t& neighbours, const node_t* nearest, const DubinsPath& path_nearest, const real_t cost_nearest) const {
        const node_t* best_parent = nearest;
        real_t min_cost = cost_nearest;
        DubinsPath best_path = path_nearest;

        for (const node_t* neighbour : neighbours) {
            const auto path_opt = find_path(neighbour->conf, conf, rho);
            if (!path_opt) continue;

            DubinsPath path = path_opt.value();
            real_t path_length = dubins_path_length(path);

            real_t cost = neighbour->cost + path_length;
            if (path_length > near_radius) continue;
            if (neighbour->cost + path_length >= min_cost) continue;

            const auto path_points = sample_path_to_end(path);
            if (is_colliding(path_points)) continue;

            best_parent = neighbour;
            min_cost = cost;
            best_path = path;
        }

        
        return {best_parent, min_cost, best_path};
    }
};
}

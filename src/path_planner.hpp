#pragma once

#include <raylib.h>
#include <raymath.h>

#include <vector>
#include <random>
#include <iostream>
#include <memory>


class path_planner {
public:
    struct obstacle {
        Vector2 position;
        float radius;
    };

    path_planner() = delete;

    path_planner(
        const Vector2& start,
        const Vector2& goal,
        const float step_size = 1.0f,
        const float goal_radius = 1.0f,
        const std::vector<Vector2>& border = {},
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
        Vector2 min = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        Vector2 max = {std::numeric_limits<float>::min(), std::numeric_limits<float>::min()};

        for (const auto& point : border) {
            if (point.x < min.x) min.x = point.x;
            if (point.y < min.y) min.y = point.y;
            if (point.x > max.x) max.x = point.x;
            if (point.y > max.y) max.y = point.y;
        }

        dis_uni_x = std::uniform_real_distribution<float>(min.x, max.x);
        dis_uni_y = std::uniform_real_distribution<float>(min.y, max.y);
    }

    virtual void step(std::size_t num_steps = 1) = 0;

    std::size_t get_goal_node_index() const {
        return goal_node_index;
    }

protected:
    const Vector2 start;
    const Vector2 goal;
    float step_size;
    float goal_radius;
    std::vector<Vector2> border;
    std::vector<obstacle> obstacles;
    std::mt19937 gen;
    std::size_t goal_node_index;

    std::uniform_real_distribution<float> dis_uni_x;
    std::uniform_real_distribution<float> dis_uni_y;


    bool isColliding(const Vector2& point) {
        bool collision = !CheckCollisionPointPoly(point, border.data(), border.size());
        
        if (collision) return collision;

        for (const auto& obstacle : obstacles) {
            if (Vector2DistanceSqr(point, obstacle.position) < (obstacle.radius * obstacle.radius)) {
                collision = true;
                break;
            }
        }

        return collision;
    }
};



class PathPlannerRRT : public path_planner {
public:

    struct node {
        Vector2 position;
        std::size_t parent;
    };

    PathPlannerRRT(
        const Vector2& start,
        const Vector2& goal,
        const float step_size = 1.0f,
        const float goal_radius = 1.0f,
        const std::vector<Vector2>& border = {},
        const std::vector<obstacle>& obstacles = {}
    ) :
        path_planner(start, goal, step_size, goal_radius, border, obstacles),
        root{start, 0}
    {
        nodes.push_back(root);
    }


    void step(std::size_t num_steps = 1) override {
        nodes.reserve(nodes.size() + num_steps);

        for (std::size_t i = 0; i < num_steps;) {
            Vector2 random_point{dis_uni_x(gen), dis_uni_y(gen)};
            std::size_t index = nearestNodeIndex(random_point, nodes);
            node nearest = nodes[index];
            Vector2 direction = Vector2Normalize(Vector2Subtract(random_point, nearest.position));
            Vector2 new_point = Vector2Add(nearest.position, Vector2Scale(direction, step_size));

            if (isColliding(new_point))
                continue;

            if (Vector2DistanceSqr(new_point, goal) < (goal_radius * goal_radius)) {
                goal_node_index = nodes.size();
            }

            node new_node;
            new_node.parent = index;
            new_node.position = new_point;

            nodes.push_back(new_node);
            ++i;
        }
    }

    std::vector<node> getNodes() const {
        return nodes;
    }

private:
    node root;
    std::vector<node> nodes;

    std::size_t nearestNodeIndex(const Vector2& point, const std::vector<node>& nodes) {
        std::size_t nearest = 0;
        float min_distance = std::numeric_limits<float>::max();

        for (std::size_t i = 0; i < nodes.size(); ++i) {
            float distance = Vector2DistanceSqr(nodes[i].position, point);
            if (distance < min_distance) {
                nearest = i;
                min_distance = distance;
            }
        }

        return nearest;
    }
};

class path_planner_rrt_star : public path_planner {
public:
    using index_t = std::uint32_t;

    struct node {
        Vector2 position;
        index_t parent;
        float cost;
    };

    class node_container_1 {
    public:
        using index_t = std::uint32_t;
        using pos_t = Vector2;
        using cost_t = float;

        node_container_1() = delete;

        node_container_1(std::size_t capacity) :
            _size(0),
            _capacity(capacity),
            _nodes(std::make_unique<node[]>(_capacity))
        {}

        std::size_t size() const { return _size; }

        pos_t& position(index_t index) { return _nodes[index].position; }
        const pos_t& position(index_t index) const { return _nodes[index].position; }

        index_t& parent(index_t index) { return _nodes[index].parent; }
        const index_t& parent(index_t index) const { return _nodes[index].parent; }

        cost_t& cost(index_t index) { return _nodes[index].cost; }
        const cost_t& cost(index_t index) const { return _nodes[index].cost; }

        void push_back(const pos_t& position, index_t parent, cost_t cost) {
            _nodes[_size] = node{position, parent, cost};
            ++_size;
        }

        void push_back(const node& n) {
            push_back(n.position, n.parent, n.cost);
        }

        node operator[](index_t index) const {
            return _nodes[index];
        }

        void clear() {
            _size = 0;
        }


    private:
        std::size_t _size;
        std::size_t _capacity;

        std::unique_ptr<node[]> _nodes;
    };

    class node_container_2 {
    public:
        using index_t = std::uint32_t;
        using pos_t = Vector2;
        using cost_t = float;

        node_container_2() = delete;

        node_container_2(std::size_t capacity) :
            _size(0),
            _capacity(capacity),
            _positions(std::make_unique<pos_t[]>(_capacity)),
            _parents(std::make_unique<index_t[]>(_capacity)),
            _costs(std::make_unique<cost_t[]>(_capacity))
        {}

        std::size_t size() const { return _size; }

        pos_t& position(index_t index) { return _positions[index]; }
        const pos_t& position(index_t index) const { return _positions[index]; }

        index_t& parent(index_t index) { return _parents[index]; }
        const index_t& parent(index_t index) const { return _parents[index]; }

        cost_t& cost(index_t index) { return _costs[index]; }
        const cost_t& cost(index_t index) const { return _costs[index]; }

        void push_back(const pos_t& position, index_t parent, cost_t cost) {
            _positions[_size] = position;
            _parents[_size] = parent;
            _costs[_size] = cost;
            ++_size;
        }

        void push_back(const node& n) {
            push_back(n.position, n.parent, n.cost);
        }

        node operator[](index_t index) const {
            return node{_positions[index], _parents[index], _costs[index]};
        }

        void clear() {
            _size = 0;
        }

    private:
        std::size_t _size;
        std::size_t _capacity;

        std::unique_ptr<pos_t[]> _positions;
        std::unique_ptr<index_t[]> _parents;
        std::unique_ptr<cost_t[]> _costs;
    };

    path_planner_rrt_star(
        const Vector2& start,
        const Vector2& goal,
        const float step_size = 2.0f,
        const float goal_radius = 1.0f,
        const std::vector<Vector2>& border = {},
        const std::vector<obstacle>& obstacles = {},
        const float neighbour_radius = 4.0f
    ) :
        path_planner(start, goal, step_size, goal_radius, border, obstacles),
        neighbour_radius(neighbour_radius),
        root{start, 0, 0.0f},
        nodes(20001)
    {
        nodes.push_back(root);
    }

    void step(std::size_t num_steps = 1) override {
        for (std::size_t i = 0; i < num_steps;) {
            Vector2 random_point{dis_uni_x(gen), dis_uni_y(gen)};
            Vector2 nearest_position = nearest_node_position(random_point);

            Vector2 direction = Vector2Normalize(Vector2Subtract(random_point, nearest_position));
            Vector2 new_point = Vector2Add(nearest_position, Vector2Scale(direction, step_size));

            if (isColliding(new_point)) {
                continue;
            }
            else {
                ++i;
            }

            const auto neighbour_indices = find_neighbours(new_point);
            const auto parent_index = find_best_parent(new_point, neighbour_indices);
            const auto parent = nodes[parent_index];

            node new_node;
            new_node.parent = parent_index;
            new_node.position = new_point;
            new_node.cost = parent.cost + Vector2Distance(new_node.position, parent.position);

            if (Vector2DistanceSqr(new_point, goal) < goal_radius * goal_radius) {
                if (goal_node_index == 0 || new_node.cost < nodes.cost(goal_node_index)) {
                    goal_node_index = nodes.size();
                }
            }

            for (const auto& neighbour_index : neighbour_indices) {
                node neighbour = nodes[neighbour_index];
                float new_cost = new_node.cost + Vector2Distance(new_node.position, neighbour.position);

                if (new_cost < neighbour.cost) {
                    nodes.parent(neighbour_index) = nodes.size();
                    nodes.cost(neighbour_index) = new_cost;
                }
            }

            nodes.push_back(new_node);
        }
    }

    const node_container& getNodes() const {
        return nodes;
    }

private:
    float neighbour_radius;
    node root;
    node_container nodes;

    Vector2 nearest_node_position(const Vector2& point) const {
        Vector2 pos{0.0f, 0.0f};
        float min_distance = std::numeric_limits<float>::max();

        for (std::size_t i = 0; i < nodes.size(); ++i) {
            Vector2 n_pos = nodes.position(i);
            float distance = Vector2DistanceSqr(n_pos, point);
            if (distance < min_distance) {
                pos = n_pos;
                min_distance = distance;
            }
        }

        return pos;
    }

    std::vector<index_t> find_neighbours(const Vector2& point) const {
        std::vector<index_t> neighbours;

        for (std::size_t i = 0; i < nodes.size(); ++i) {
            float distance = Vector2DistanceSqr(nodes.position(i), point);
            if (distance < neighbour_radius * neighbour_radius) {
                neighbours.push_back(i);
            }
        }

        return neighbours;
    }

    index_t find_best_parent(const Vector2& point, const std::vector<index_t>& neighbour_indices) const {
        index_t best_parent_index = 0;
        float min_cost = std::numeric_limits<float>::max();

        for (const auto& neighbour_index : neighbour_indices) {
            node neighbour = nodes[neighbour_index];
            float distance = Vector2Distance(neighbour.position, point);
            float cost = neighbour.cost + distance;

            if (cost < min_cost) {
                best_parent_index = neighbour_index;
                min_cost = cost;
            }
        }

        return best_parent_index;
    }
};

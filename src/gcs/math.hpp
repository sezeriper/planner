#pragma once

#include "dubins/dubins.hpp"

#include <cmath>
#include <vector>
#include <numbers>
#include <optional>

namespace rota {

using real_t = float;

constexpr real_t PI = std::numbers::pi_v<real_t>;
constexpr real_t TWO_PI = 2.0f * PI;
constexpr real_t INF = std::numeric_limits<real_t>::max();

struct vec2_t {
    real_t x;
    real_t y;

    constexpr bool operator==(vec2_t other) const {
        return x == other.x && y == other.y;
    }
};

constexpr real_t distance_sqr(const vec2_t a, const vec2_t b) {
    return ((a.x - b.x) * (a.x - b.x)) + ((a.y - b.y) * (a.y - b.y));
}

constexpr real_t distance(const vec2_t a, const vec2_t b) {
    return std::sqrt(distance_sqr(a, b));
}

constexpr vec2_t add(const vec2_t a, const vec2_t b) {
    return {a.x + b.x, a.y + b.y};
}

constexpr vec2_t subtract(const vec2_t a, const vec2_t b) {
    return {a.x - b.x, a.y - b.y};
}

constexpr vec2_t normalize(const vec2_t p) {
    real_t length = std::sqrt(p.x * p.x + p.y * p.y);
    return {p.x / length, p.y / length};
}

constexpr vec2_t scale(const vec2_t p, real_t s) {
    return {p.x * s, p.y * s};
}

// Taken from raylib/src/rshapes.c
// Check if vec2_t is within a polygon described by array of vertices
// NOTE: Based on http://jeffreythompson.org/collision-detection/poly-vec2_t.php
constexpr bool is_inside(const vec2_t p, const std::vector<vec2_t>& poly)
{
    bool inside = false;

    if (poly.size() > 2)
    {
        for (int i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
        {
            if ((poly[i].y > p.y) != (poly[j].y > p.y) &&
                (p.x < (poly[j].x - poly[i].x)*(p.y - poly[i].y)/(poly[j].y - poly[i].y) + poly[i].x))
            {
                inside = !inside;
            }
        }
    }

    return inside;
}

struct configuration_t {
    real_t x;
    real_t y;
    real_t yaw;

    constexpr vec2_t get_point() const { return vec2_t{x, y}; }
};

constexpr bool operator==(configuration_t lhs, configuration_t rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.yaw == rhs.yaw;
}

using trajectory_t = std::vector<configuration_t>;

static std::optional<DubinsPath> find_path(const configuration_t from, const configuration_t to, real_t rho) {
    DubinsPath path;

    bool result = dubins_shortest_path(path, &from.x, &to.x, rho);
    if (result == 0) {
        return path;
    }
    return std::nullopt;
}

static real_t find_path_length(const configuration_t from, const configuration_t to, real_t rho) {
    const auto path_opt = find_path(from, to, rho);
    if (!path_opt) return std::numeric_limits<real_t>::max();
    const auto path = path_opt.value();
    return dubins_path_length(path);
}

static trajectory_t sample_path_to_end(const DubinsPath& path, real_t interval = 0.1f) {
    trajectory_t path_points;

    real_t q[3];
    real_t length = dubins_path_length(path);
    for (real_t t = 0.0f; t < length; t += interval) {
        int result = dubins_path_sample(path, t, q);
        path_points.push_back({ q[0], q[1], q[2] });
    }

    dubins_path_endvec2_t(path, q);
    path_points.push_back({ q[0], q[1], q[2] });

    return path_points;
}

static trajectory_t sample_path_to_length(const DubinsPath& path, real_t length, real_t interval = 0.1f) {
    trajectory_t path_points;

    real_t path_length = dubins_path_length(path);

    if ((path_length - length) < 0.0f) {
        return sample_path_to_end(path, interval);
    }

    real_t q[3];
    for (real_t t = 0.0f; t < length; t += interval) {
        int result = dubins_path_sample(path, t, q);
        path_points.push_back({ q[0], q[1], q[2] });
    }

    dubins_path_sample(path, length - EPSILON, q);
    path_points.push_back({ q[0], q[1], q[2] });

    return path_points;
}

constexpr trajectory_t sample_paths(const std::vector<DubinsPath>& paths, real_t interval = 0.1f) {
    trajectory_t path_points;

    for (const auto& path : paths) {
        const auto points = sample_path_to_end(path, interval);
        path_points.insert(path_points.end(), points.begin(), points.end());
    }

    return path_points;
}
}
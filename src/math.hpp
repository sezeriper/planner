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

constexpr real_t magnitude(const vec2_t vec) {
    return std::sqrt(vec.x * vec.x + vec.y * vec.y);
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

    dubins_path_endpoint(path, q);
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

struct vec3_t {
    real_t x;
    real_t y;
    real_t z;

    constexpr bool operator==(vec3_t other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

constexpr real_t deg_to_rad(real_t deg) {
    return deg / 180.0f * PI ;
}

constexpr real_t rad_to_deg(real_t rad) {
    return rad / PI * 180.0f;
}

struct line_t {
    vec2_t p1;
    vec2_t p2;
};

struct circle_t {
    vec2_t center;
    real_t r;
};

struct intersection_t {
  enum class solution_case_t : int { NO_SOLUTION, ONE_SOLUTION, TWO_SOLUTIONS };
  vec2_t p1, p2;
  solution_case_t solution_case;
};

constexpr intersection_t compute_intersection(const line_t& line, const circle_t& circle) {
    constexpr real_t zero_threshold = 1e-5;
    constexpr auto is_almost_zero = [](real_t val) { return std::fabs(val) < zero_threshold; };
    constexpr auto sign = [](real_t x) { return x < 0.0f ? -1.0f : 1.0f; };


    intersection_t intersection{};

    /*need to shift everything to the origin so that the formulas for the intersection are valid
    * those are taken from: http://mathworld.wolfram.com/Circle-LineIntersection.html
    * they're valid for the circle centered around the origin.
    */

    const real_t x1 = line.p1.x - circle.center.x;
    const real_t x2 = line.p2.x - circle.center.x;
    const real_t y1 = line.p1.y - circle.center.y;
    const real_t y2 = line.p2.y - circle.center.y;

    const real_t dx = x2 - x1;
    const real_t dy = y2 - y1;
    const real_t dr2 = dx * dx + dy * dy;
    const real_t D = x1 * y2 - x2 * y1;
    const real_t discriminant = circle.r * circle.r * dr2 - D * D;

    if (discriminant < -zero_threshold) {
        intersection.solution_case = intersection_t::solution_case_t::NO_SOLUTION;
        return intersection;
    }

    const real_t sqrt_discriminant = std::sqrt(discriminant);
    vec2_t solution1{(D * dy + sign(dy) * dx * sqrt_discriminant) / dr2, (-D * dx + std::abs(dy) * sqrt_discriminant) / dr2};
    vec2_t solution2{(D * dy - sign(dy) * dx * sqrt_discriminant) / dr2, (-D * dx - std::abs(dy) * sqrt_discriminant) / dr2};

    // translate solution back from the origin
    solution1 = add(circle.center, solution1);
    solution2 = add(circle.center, solution2);

    intersection.p1 = solution1;
    intersection.p2 = solution2;

    if (is_almost_zero(discriminant)) {
        intersection.solution_case = intersection_t::solution_case_t::ONE_SOLUTION;
    }
    else {
        intersection.solution_case = intersection_t::solution_case_t::TWO_SOLUTIONS;
    }
    return intersection;
}


constexpr vec3_t ned_to_xyz(real_t north, real_t east, real_t down) {
    return {north, -down, east};
}

constexpr vec3_t xyz_to_ned(real_t x, real_t y, real_t z) {
    return {x, z, -y};
}

} // namespace rota
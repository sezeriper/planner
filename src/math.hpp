#pragma once

#include <cstdint>
#include <cmath>
#include <vector>

namespace rota {

using real_t = float;

struct point {
    real_t x;
    real_t y;

    constexpr bool operator==(point other) const {
        return x == other.x && y == other.y;
    }
};

constexpr real_t distance_sqr(const point a, const point b) {
    return ((a.x - b.x) * (a.x - b.x)) + ((a.y - b.y) * (a.y - b.y));
}

constexpr real_t distance(const point a, const point b) {
    return std::sqrt(distance_sqr(a, b));
}

constexpr point add(const point a, const point b) {
    return {a.x + b.x, a.y + b.y};
}

constexpr point subtract(const point a, const point b) {
    return {a.x - b.x, a.y - b.y};
}

constexpr point normalize(const point p) {
    real_t length = std::sqrt(p.x * p.x + p.y * p.y);
    return {p.x / length, p.y / length};
}

constexpr point scale(const point p, real_t s) {
    return {p.x * s, p.y * s};
}

// Taken from raylib/src/rshapes.c
// Check if point is within a polygon described by array of vertices
// NOTE: Based on http://jeffreythompson.org/collision-detection/poly-point.php
constexpr bool is_inside(const point p, const std::vector<point>& poly)
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
}
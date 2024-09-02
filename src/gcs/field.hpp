#pragma once

#include "math.hpp"

#include <vector>

namespace rota {
struct obstacle_t {
    vec2_t position;
    real_t radius;
};


using border_t = std::vector<vec2_t>;
using obstacles_t = std::vector<obstacle_t>;

struct field_t {
    border_t border;
    obstacles_t obstacles;
};
}
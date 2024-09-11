#pragma once

#include "math.hpp"

namespace rota {
namespace messages {
struct coords_t {
    real_t lat; // -90.0f to 90.0f
    real_t lon; // -180.0f to 180.0f
    real_t alt; // altitude above ground in meters
};

struct position_t {
    real_t north;
    real_t east;
    real_t down;
};

struct attitude_t {
    real_t roll; // radians
    real_t pitch; // radians
    real_t yaw; // radians
};

using heading_t = real_t; // radians

struct distance_sensor_t {
    real_t distance; // meters
    real_t min_distance; // meters
    real_t max_distance; // meters
};

struct battery_t {
    real_t voltage;
    real_t remaining_percent;
};

struct metrics_t {
    real_t airspeed; // meters per second
    real_t throttle; // 0.0f - 100.0f
    real_t climb_rate; // meters per second
};

struct gps_origin_t {
    real_t lat;
    real_t lon;
    real_t alt; // altitude above sea level in meters
};

enum class flight_mode_t {
    Unknown,
    Ready,
    Takeoff,
    Hold,
    Mission,
    ReturnToLaunch,
    Land,
    Offboard,
    FollowMe,
    Manual,
    Altctl,
    Posctl,
    Acro,
    Stabilized,
    Rattitude,
};
}
}
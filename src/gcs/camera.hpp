#pragma once

#include "../math.hpp"

#include <raylib.h>
#include <raymath.h>

namespace rota {
class camera {
public:
    camera() :
        cam{{0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            45.0f,
            CAMERA_PERSPECTIVE},
        cam_angle{0.0f, PI / 4.0f},
        cam_dist{10.0f} {}
    
    void handle_input() {
        Vector2 mouse_dt{};
        mouse_dt.x += IsKeyDown(KEY_A);
        mouse_dt.x -= IsKeyDown(KEY_D);
        mouse_dt.y += IsKeyDown(KEY_W);
        mouse_dt.y -= IsKeyDown(KEY_S);
        mouse_dt = Vector2Scale(mouse_dt, GetFrameTime() * 2.0f);

        cam_dist -= GetMouseWheelMove() * 0.3f;

        cam_angle.x -= mouse_dt.x;
        cam_angle.y += mouse_dt.y;
        cam_angle = {Wrap(cam_angle.x, 0.0f, PI * 2.0f), Clamp(cam_angle.y, 0.1f, (PI / 2.0f) - 0.0001f)};

        update();
    }

    void update() {
        cam.position = {0.0f, 0.0f, -cam_dist};
        cam.position = Vector3RotateByAxisAngle(cam.position, Vector3{1.0f, 0.0f, 0.0f}, cam_angle.y);
        cam.position = Vector3RotateByAxisAngle(cam.position, Vector3{0.0f, 1.0f, 0.0f}, cam_angle.x);
        cam.position.x += cam.target.x;
        cam.position.y += cam.target.y;
        cam.position.z += cam.target.z;

        Vector3 camera_dir = Vector3Normalize(Vector3Subtract(cam.position, cam.target));
        Vector3 camera_right = Vector3Normalize(Vector3CrossProduct({0.0f, 1.0f, 0.0f}, camera_dir));
        cam.up = Vector3CrossProduct(camera_dir, camera_right);
    }

    Camera3D get() const {
        return cam;
    }

    void set_target(vec3_t pos) {
        cam.target = {pos.x, pos.y, pos.z};
    }

    void set_distance(float dist) {
        cam_dist = dist;
    }

private:
    Camera3D cam;
    Vector2 cam_angle;
    float cam_dist;
};
}
#pragma once

#include "Math/MyEigen.h"

struct Ray {
    Vec3 origin;
    Vec3 dir;
};

class Camera {
public:
    Camera(Vec3 target, double distance);

    // Drag gestures: deltas in pixels.
    void Rotate(double dx, double dy);
    void Pan(double dx, double dy);
    void Zoom(double scrollDelta);

    Vec3 Position() const;

    // Ray through pixel center (x, y) of a w x h image; y = 0 is the bottom row.
    Ray PixelRay(double x, double y, int w, int h) const;

    // Project a world point to NDC ([-1, 1]^2, y up); false if it is behind the camera.
    bool ProjectToNdc(const Vec3& p, int w, int h, Vec2& ndcOut) const;

private:
    Vec3 Forward() const;
    Vec3 Right() const;
    Vec3 Up() const;

    Vec3 target_;
    double distance_;
    double yaw_ = 0.0;
    double pitch_ = 0.0;
    double fovY_;
};

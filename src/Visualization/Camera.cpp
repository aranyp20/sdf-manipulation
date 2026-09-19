#include "Visualization/Camera.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace {
constexpr double kRotateSpeed = 0.01; // radians per pixel
constexpr double kPanSpeed = 0.002; // world units per pixel, scaled by distance
constexpr double kZoomSpeed = 0.1;
constexpr double kMinDistance = 0.05;
} // namespace

Camera::Camera(Vec3 target, double distance)
: target_(std::move(target))
, distance_(distance)
, fovY_(60.0 * M_PI / 180.0) {}

void Camera::Rotate(double dx, double dy) {
    yaw_ += dx * kRotateSpeed;
    pitch_ += dy * kRotateSpeed;
    constexpr double limit = M_PI / 2.0 - 0.01;
    pitch_ = std::clamp(pitch_, -limit, limit);
}

void Camera::Pan(double dx, double dy) {
    target_ += (-dx * Right() + dy * Up()) * kPanSpeed * distance_;
}

void Camera::Zoom(double scrollDelta) {
    distance_ = std::max(kMinDistance, distance_ * std::exp(-scrollDelta * kZoomSpeed));
}

Vec3 Camera::Forward() const {
    return Vec3(std::cos(pitch_) * std::sin(yaw_), std::sin(pitch_), -std::cos(pitch_) * std::cos(yaw_)).normalized() *
           -1.0;
}

Vec3 Camera::Right() const {
    return Forward().cross(Vec3::UnitY()).normalized();
}

Vec3 Camera::Up() const {
    return Right().cross(Forward()).normalized();
}

Vec3 Camera::Position() const {
    return target_ - Forward() * distance_;
}

Ray Camera::PixelRay(double x, double y, int w, int h) const {
    const double aspect = static_cast<double>(w) / h;
    const double tanHalfFov = std::tan(fovY_ / 2.0);
    const double ndcX = (2.0 * (x + 0.5) / w - 1.0) * aspect * tanHalfFov;
    const double ndcY = (2.0 * (y + 0.5) / h - 1.0) * tanHalfFov;
    const Vec3 dir = (Forward() + ndcX * Right() + ndcY * Up()).normalized();
    return {Position(), dir};
}

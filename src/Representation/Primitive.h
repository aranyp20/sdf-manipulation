#pragma once

#include "Representation/Implicit.h"

class Sphere : public Implicit {
public:
    Sphere(Vec3 center, double radius)
    : center_(std::move(center))
    , radius_(radius) {}

    double Sdf(const Vec3& p) const override {
        return (p - center_).norm() - radius_;
    }

    Vec3 Grad(const Vec3& p) const override {
        const double dist = (p - center_).norm();
        if (dist == 0.0) {
            return Vec3::Zero(); // the distance field has no gradient at the center
        }
        return (p - center_) / dist;
    }

    Box3 GetBoundingBox() const override {
        return {center_ - Vec3::Constant(radius_), center_ + Vec3::Constant(radius_)};
    }

private:
    Vec3 center_;
    double radius_;
};

// Half-space bounded by the plane normal . p = offset; the inside (negative
// SDF) is the side the normal points away from.
class Plane : public Implicit {
public:
    Plane(Vec3 normal, double offset)
    : normal_(normal.normalized())
    , offset_(offset) {}

    double Sdf(const Vec3& p) const override {
        return normal_.dot(p) - offset_;
    }

    Vec3 Grad(const Vec3& /*p*/) const override {
        return normal_;
    }

    Box3 GetBoundingBox() const override {
        return Box3(); // unbounded, so no finite bounding box; empty merges as a no-op
    }

private:
    Vec3 normal_;
    double offset_;
};

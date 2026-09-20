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

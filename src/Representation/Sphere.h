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

private:
    Vec3 center_;
    double radius_;
};

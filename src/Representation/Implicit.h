#pragma once

#include "Math/MyEigen.h"

class Implicit {
public:
    virtual ~Implicit() = default;
    virtual double Sdf(const Vec3& p) const = 0;
};

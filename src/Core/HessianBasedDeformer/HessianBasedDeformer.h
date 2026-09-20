#pragma once

#include "Core/HessianBasedDeformer/HBDebugData.h"
#include "Representation/ImplicitBspline.h"

// Hessian-based field deformer for topology-aware shape editing
// (Zhang et al., SIGGRAPH Asia 2023). For now only the exhaustive
// saddle point search (paper sec. 3.2) is implemented.
class HessianBasedDeformer {
public:
    explicit HessianBasedDeformer(const ImplicitBspline& spline);

    // Subdivide every grid cell once (8 subcells), run Newton's method on
    // grad F = 0 from each subcell center, and classify the converged critical
    // points by the eigenvalue signs of the Hessian (paper Tab. 1).
    HBDebugData SearchSaddlePoints() const;

private:
    const ImplicitBspline& spline_;
};

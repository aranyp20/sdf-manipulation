#pragma once

#include "Core/HessianBasedDeformer/HBDebugData.h"
#include "Representation/ImplicitBspline.h"

// One deformer D_s(q) = beta * B(W^-1 Q^T (q - s)) rooted at a saddle point s
// (paper eq. 10), with the local frame Q aligned to the Hessian eigenvectors
// and per-axis widths W derived from the tunable weights (paper sec. 3.3-3.4).
struct SaddleDeformer {
    // Tunable weights (paper fig. 6): W1 = mu |F(s)|, W2 = phi w, beta = -rho F(s).
    // rho starts at 0 (paper default: 5) so a freshly added deformer is a no-op
    // until the user dials the strength in.
    float mu = 2.0f;
    float phi = 4.0f;
    float rho = 0.0f;

    Vec3 s = Vec3::Zero();
    Mat3 Q = Mat3::Identity(); // eigenvector columns, ordered per paper sec. 3.3
    Vec3 lambda = Vec3::Zero(); // eigenvalues matching the columns of Q
    double fs = 0.0; // F(s)
    double cellWidth = 0.0;

    // Derived; call UpdateWeights() after changing mu/phi/rho.
    Vec3 invW = Vec3::Ones();
    double beta = 0.0;

    void UpdateWeights();
    double Eval(const Vec3& q) const;
    Vec3 Grad(const Vec3& q) const;
};

// F_new(q) = F(q) + sum_i D_i(q) (paper eq. 8), renderable like any implicit.
// Holds references only; keep the base spline and the deformer list alive.
class DeformedImplicit : public Implicit {
public:
    DeformedImplicit(const ImplicitBspline& base, const std::vector<SaddleDeformer>& deformers);

    double Sdf(const Vec3& p) const override;
    Vec3 Grad(const Vec3& p) const override;

    Box3 GetBoundingBox() const override {
        return base_.GetBoundingBox();
    }

private:
    const ImplicitBspline& base_;
    const std::vector<SaddleDeformer>& deformers_;
    // F_new is no longer a distance function; sphere-tracing steps are scaled
    // down by a bound on the deformers' gradient so the tracer cannot overshoot.
    double invLipschitz_ = 1.0;
};

// Hessian-based field deformer for topology-aware shape editing
// (Zhang et al., SIGGRAPH Asia 2023).
class HessianBasedDeformer {
public:
    explicit HessianBasedDeformer(const ImplicitBspline& spline);

    // Subdivide every grid cell once (8 subcells), run Newton's method on
    // grad F = 0 from each subcell center, and classify the converged critical
    // points by the eigenvalue signs of the Hessian (paper Tab. 1).
    HBDebugData SearchSaddlePoints() const;

    // Build a deformer rooted at the given saddle point with default weights
    // (paper sec. 3.3): eigen-decompose the Hessian and order the eigenvectors
    // by the direction toward the surface and the eigenvalue signs.
    SaddleDeformer MakeDeformer(const Vec3& saddle) const;

private:
    const ImplicitBspline& spline_;
};

#pragma once

#include "Representation/Implicit.h"

#include <functional>
#include <vector>

// Cubic trivariate tensor-product B-spline implicit function, as used in
// "A Hessian-Based Field Deformer for Real-Time Topology-Aware Shape Editing"
// (Zhang et al., SIGGRAPH Asia 2023): F(q) = sum_ijk alpha_ijk * B((q - g_ijk) / w),
// with B(q) = b(x) b(y) b(z) the univariate cubic B-spline basis product and
// g_ijk the vertices of a regular grid over the domain box.
class ImplicitBspline : public Implicit {
public:
    ImplicitBspline(Vec3 domainMin, double domainSize, int cellsPerAxis);

    double Sdf(const Vec3& p) const override;
    Vec3 Grad(const Vec3& p) const override;

    Box3 GetBoundingBox() const override {
        return {domainMin_, domainMin_ + Vec3::Constant(size_)};
    }

    // Analytic gradient and (symmetric) Hessian of F at p.
    void EvalDerivatives(const Vec3& p, Vec3& grad, Mat3& hess) const;

    const Vec3& DomainMin() const {
        return domainMin_;
    }
    double DomainSize() const {
        return size_;
    }
    double CellWidth() const {
        return w_;
    }
    int CellsPerAxis() const {
        return n_;
    }

    // Solve A alpha = c so that F interpolates the given SDF at the grid vertices (paper eq. 6-7).
    void Fit(const std::function<double(const Vec3&)>& sdf);

    static ImplicitBspline MakePaca();
    static ImplicitBspline MakeTorus();

private:
    double Eval(const Vec3& p) const;
    Vec3 Vertex(int i, int j, int k) const;
    int Index(int i, int j, int k) const;

    Vec3 domainMin_;
    double size_;
    double w_; // grid cell width
    int n_; // cells per axis; n_ + 1 vertices per axis
    std::vector<double> alpha_;
    // Lower bound of the surface distance on the domain boundary; lets the tracer
    // step through the box wall instead of stalling on it.
    double boundaryMargin_ = 0.0;
};

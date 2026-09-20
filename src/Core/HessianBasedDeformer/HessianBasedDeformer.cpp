#include "Core/HessianBasedDeformer/HessianBasedDeformer.h"

#include <Eigen/Eigenvalues>
#include <cmath>

namespace {

constexpr int kMaxNewtonIterations = 20; // paper sec. 3.2
constexpr double kGradientTol = 1e-7;
// Flat regions of the spline (alpha ~ 0 away from the surface) satisfy
// grad F ~ 0 everywhere; a near-zero eigenvalue marks such degenerate points.
constexpr double kEigenvalueTol = 1e-5;
constexpr double kMergeTol = 1e-4;
// The fit oscillates near the domain boundary (truncated basis support), which
// spawns hundreds of spurious critical points there; skip the outer cell layers.
constexpr int kBoundarySkipCells = 3;

bool NearAny(const std::vector<Vec3>& points, const Vec3& p) {
    for (const Vec3& q : points) {
        if ((q - p).norm() < kMergeTol) {
            return true;
        }
    }
    return false;
}

} // namespace

HessianBasedDeformer::HessianBasedDeformer(const ImplicitBspline& spline)
: spline_(spline) {}

HBDebugData HessianBasedDeformer::SearchSaddlePoints() const {
    HBDebugData data;
    const int n = spline_.CellsPerAxis();
    const double w = spline_.CellWidth();
    const int c0 = kBoundarySkipCells;
    const int c1 = n - kBoundarySkipCells;
    const Vec3 origin = spline_.DomainMin();
    const Vec3 lo = origin + Vec3::Constant(c0 * w);
    const Vec3 hi = origin + Vec3::Constant(c1 * w);

    Vec3 grad;
    Mat3 hess;
    for (int k = c0; k < c1; ++k) {
        for (int j = c0; j < c1; ++j) {
            for (int i = c0; i < c1; ++i) {
                for (int s = 0; s < 8; ++s) {
                    Vec3 q = origin + w * Vec3(i + 0.25 + 0.5 * (s & 1),
                                               j + 0.25 + 0.5 * ((s >> 1) & 1),
                                               k + 0.25 + 0.5 * ((s >> 2) & 1));
                    bool converged = false;
                    for (int iter = 0; iter < kMaxNewtonIterations; ++iter) {
                        spline_.EvalDerivatives(q, grad, hess);
                        if (grad.norm() < kGradientTol) {
                            converged = true;
                            break;
                        }
                        if (std::abs(hess.determinant()) < 1e-12) {
                            break;
                        }
                        Vec3 step = hess.inverse() * grad;
                        // Keep the search local: a huge step means Newton is diverging here.
                        if (step.norm() > w) {
                            step *= w / step.norm();
                        }
                        q -= step;
                        if ((q.array() < lo.array()).any() || (q.array() > hi.array()).any()) {
                            break;
                        }
                    }
                    if (!converged) {
                        continue;
                    }

                    const Eigen::SelfAdjointEigenSolver<Mat3> eig(hess);
                    const Vec3 lambda = eig.eigenvalues();
                    if (lambda.cwiseAbs().minCoeff() < kEigenvalueTol) {
                        continue; // degenerate critical point (flat region)
                    }
                    const int negative = static_cast<int>((lambda.array() < 0.0).count());
                    std::vector<Vec3>* bucket[] = {&data.minimum, &data.oneSaddle, &data.twoSaddle, &data.maximum};
                    if (!NearAny(*bucket[negative], q)) {
                        bucket[negative]->push_back(q);
                    }
                }
            }
        }
    }
    return data;
}

#include "Core/HessianBasedDeformer/HessianBasedDeformer.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>

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

// |grad B(v)| <= sqrt(3) * max|b'| * (max b)^2 = sqrt(3) * (2/3)^3, rounded up.
constexpr double kBasisGradBound = 0.52;

// Floors for the per-axis widths W: a saddle sitting almost on the surface
// (F(s) ~ 0) would otherwise collapse W1 (and W3) to zero.
double MinWidth(double cellWidth) {
    return 1e-3 * cellWidth;
}

} // namespace

void SaddleDeformer::UpdateWeights() {
    // Paper sec. 3.4: W1 = mu |F(s)|, W2 = phi w; W3 follows W1 if lambda3 and
    // lambda1 share their sign, otherwise W2, scaled by the eigenvalue ratio.
    const double w1 = std::max(static_cast<double>(mu) * std::abs(fs), MinWidth(cellWidth));
    const double w2 = std::max(static_cast<double>(phi) * cellWidth, MinWidth(cellWidth));
    const bool sameSign = lambda.z() * lambda.x() > 0.0;
    const double w3 = std::max(sameSign ? w1 * std::abs(lambda.z() / lambda.x()) : w2 * std::abs(lambda.z() / lambda.y()),
                               MinWidth(cellWidth));
    invW = Vec3(1.0 / w1, 1.0 / w2, 1.0 / w3);
    beta = -static_cast<double>(rho) * fs;
}

double SaddleDeformer::Eval(const Vec3& q) const {
    const Vec3 v = invW.cwiseProduct(Q.transpose() * (q - s));
    if (v.cwiseAbs().maxCoeff() >= 2.0) {
        return 0.0;
    }
    return beta * ImplicitBspline::Basis(v.x()) * ImplicitBspline::Basis(v.y()) * ImplicitBspline::Basis(v.z());
}

Vec3 SaddleDeformer::Grad(const Vec3& q) const {
    const Vec3 v = invW.cwiseProduct(Q.transpose() * (q - s));
    if (v.cwiseAbs().maxCoeff() >= 2.0) {
        return Vec3::Zero();
    }
    const Vec3 b(ImplicitBspline::Basis(v.x()), ImplicitBspline::Basis(v.y()), ImplicitBspline::Basis(v.z()));
    const Vec3 db(ImplicitBspline::BasisPrime(v.x()),
                  ImplicitBspline::BasisPrime(v.y()),
                  ImplicitBspline::BasisPrime(v.z()));
    const Vec3 gradV(db.x() * b.y() * b.z(), b.x() * db.y() * b.z(), b.x() * b.y() * db.z());
    return beta * Q * invW.cwiseProduct(gradV);
}

DeformedImplicit::DeformedImplicit(const ImplicitBspline& base, const std::vector<SaddleDeformer>& deformers)
: base_(base)
, deformers_(deformers) {
    double lipschitz = 1.0;
    for (const SaddleDeformer& d : deformers_) {
        lipschitz += std::abs(d.beta) * kBasisGradBound * d.invW.maxCoeff();
    }
    invLipschitz_ = 1.0 / lipschitz;
}

double DeformedImplicit::Sdf(const Vec3& p) const {
    double f = base_.Sdf(p);
    for (const SaddleDeformer& d : deformers_) {
        f += d.Eval(p);
    }
    return f * invLipschitz_;
}

Vec3 DeformedImplicit::Grad(const Vec3& p) const {
    Vec3 g = base_.Grad(p);
    for (const SaddleDeformer& d : deformers_) {
        g += d.Grad(p);
    }
    return g;
}

HessianBasedDeformer::HessianBasedDeformer(const ImplicitBspline& spline)
: spline_(spline) {}

SaddleDeformer HessianBasedDeformer::MakeDeformer(const Vec3& saddle) const {
    SaddleDeformer d;
    d.s = saddle;
    d.cellWidth = spline_.CellWidth();
    d.fs = spline_.Sdf(saddle);

    Vec3 grad;
    Mat3 hess;
    spline_.EvalDerivatives(saddle, grad, hess);
    const Eigen::SelfAdjointEigenSolver<Mat3> eig(hess);
    const Mat3 vectors = eig.eigenvectors();
    const Vec3 values = eig.eigenvalues();

    // Project s onto the surface (paper sec. 3.3). grad F = 0 at s, so gradient
    // descent cannot leave the saddle; instead march along the +-eigenvector
    // directions until F changes sign and keep the nearest crossing.
    const double step = 0.5 * spline_.CellWidth();
    const double maxT = 0.5 * spline_.DomainSize();
    double bestT = std::numeric_limits<double>::max();
    Vec3 toSurface = vectors.col(0);
    for (int e = 0; e < 3; ++e) {
        for (const double sign : {-1.0, 1.0}) {
            const Vec3 dir = sign * vectors.col(e);
            double prev = 0.0;
            for (double t = step; t < maxT; t += step) {
                if (spline_.Sdf(saddle + t * dir) * d.fs < 0.0) {
                    double lo = prev, hi = t;
                    for (int it = 0; it < 30; ++it) {
                        const double mid = 0.5 * (lo + hi);
                        (spline_.Sdf(saddle + mid * dir) * d.fs < 0.0 ? hi : lo) = mid;
                    }
                    if (hi < bestT) {
                        bestT = hi;
                        toSurface = dir;
                    }
                    break;
                }
                prev = t;
            }
        }
    }

    // First eigenvector: the one best aligned with the direction s -> s'.
    int first = 0;
    for (int e = 1; e < 3; ++e) {
        if (std::abs(vectors.col(e).dot(toSurface)) > std::abs(vectors.col(first).dot(toSurface))) {
            first = e;
        }
    }
    // Second: among the remaining two, the one whose eigenvalue sign is opposite
    // to lambda1; if both (or neither) are opposite, the larger absolute value.
    const int a = (first + 1) % 3;
    const int b = (first + 2) % 3;
    const bool aOpposite = values[a] * values[first] < 0.0;
    const bool bOpposite = values[b] * values[first] < 0.0;
    int second;
    if (aOpposite != bOpposite) {
        second = aOpposite ? a : b;
    } else {
        second = std::abs(values[a]) >= std::abs(values[b]) ? a : b;
    }
    const int third = a + b - second;

    d.Q.col(0) = vectors.col(first);
    d.Q.col(1) = vectors.col(second);
    d.Q.col(2) = vectors.col(third);
    d.lambda = Vec3(values[first], values[second], values[third]);
    d.UpdateWeights();
    return d;
}

HBDebugData HessianBasedDeformer::SearchSaddlePoints() const {
    HBDebugData data;
    const int n = spline_.CellsPerAxis();
    const double w = spline_.CellWidth();
    const int c0 = kBoundarySkipCells;
    const int c1 = n - kBoundarySkipCells;
    const Vec3 origin = spline_.DomainMin();
    const Vec3 lo = origin + Vec3::Constant(c0 * w);
    const Vec3 hi = origin + Vec3::Constant(c1 * w);
    data.searchBox = Box3(lo, hi);

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

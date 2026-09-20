#include "Representation/ImplicitBspline.h"

#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {

// Univariate cubic B-spline basis b(t) (paper eq. 4); symmetric, supported on [-2, 2].
double CubicB(double t) {
    t = std::abs(t);
    if (t < 1.0) {
        return 0.5 * t * t * t - t * t + 2.0 / 3.0;
    }
    if (t < 2.0) {
        return -t * t * t / 6.0 + t * t - 2.0 * t + 4.0 / 3.0;
    }
    return 0.0;
}

} // namespace

ImplicitBspline::ImplicitBspline(Vec3 domainMin, double domainSize, int cellsPerAxis)
: domainMin_(std::move(domainMin))
, size_(domainSize)
, w_(domainSize / cellsPerAxis)
, n_(cellsPerAxis)
, alpha_(static_cast<size_t>(cellsPerAxis + 1) * (cellsPerAxis + 1) * (cellsPerAxis + 1), 0.0) {}

int ImplicitBspline::Index(int i, int j, int k) const {
    const int m = n_ + 1;
    return (k * m + j) * m + i;
}

Vec3 ImplicitBspline::Vertex(int i, int j, int k) const {
    return domainMin_ + w_ * Vec3(i, j, k);
}

double ImplicitBspline::Eval(const Vec3& p) const {
    const Vec3 u = (p - domainMin_) / w_;
    int base[3];
    double weight[3][4];
    for (int a = 0; a < 3; ++a) {
        base[a] = static_cast<int>(std::floor(u[a])) - 1;
        for (int t = 0; t < 4; ++t) {
            weight[a][t] = CubicB(u[a] - (base[a] + t));
        }
    }
    double sum = 0.0;
    for (int dk = 0; dk < 4; ++dk) {
        const int k = base[2] + dk;
        if (k < 0 || k > n_) {
            continue;
        }
        for (int dj = 0; dj < 4; ++dj) {
            const int j = base[1] + dj;
            if (j < 0 || j > n_) {
                continue;
            }
            const double wjk = weight[2][dk] * weight[1][dj];
            for (int di = 0; di < 4; ++di) {
                const int i = base[0] + di;
                if (i < 0 || i > n_) {
                    continue;
                }
                sum += alpha_[Index(i, j, k)] * weight[0][di] * wjk;
            }
        }
    }
    return sum;
}

double ImplicitBspline::Sdf(const Vec3& p) const {
    // Outside the domain box the spline decays to zero, so march to the box first.
    // The box distance is a safe lower bound of the surface distance (the surface lies inside);
    // adding the boundary spline value would overestimate and make the tracer overshoot.
    const Vec3 lo = domainMin_;
    const Vec3 hi = domainMin_ + Vec3::Constant(size_);
    const Vec3 q = p.cwiseMax(lo).cwiseMin(hi);
    const double outside = (p - q).norm();
    if (outside > 0.0) {
        return outside + boundaryMargin_;
    }
    return Eval(p);
}

void ImplicitBspline::Fit(const std::function<double(const Vec3&)>& sdf) {
    const int m = n_ + 1;
    const int total = m * m * m;

    Eigen::VectorXd c(total);
    double minBoundary = std::numeric_limits<double>::max();
    for (int k = 0; k < m; ++k) {
        for (int j = 0; j < m; ++j) {
            for (int i = 0; i < m; ++i) {
                const double d = sdf(Vertex(i, j, k));
                c(Index(i, j, k)) = d;
                const bool onBoundary = i == 0 || i == n_ || j == 0 || j == n_ || k == 0 || k == n_;
                if (onBoundary) {
                    minBoundary = std::min(minBoundary, d);
                }
            }
        }
    }
    boundaryMargin_ = std::max(0.0, 0.9 * minBoundary);

    // A_(row, col) = B_col(g_row); the basis at integer offsets is b(0) = 2/3, b(+-1) = 1/6.
    const double b1d[3] = {1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0};
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<size_t>(total) * 27);
    for (int k = 0; k < m; ++k) {
        for (int j = 0; j < m; ++j) {
            for (int i = 0; i < m; ++i) {
                const int row = Index(i, j, k);
                for (int dk = -1; dk <= 1; ++dk) {
                    for (int dj = -1; dj <= 1; ++dj) {
                        for (int di = -1; di <= 1; ++di) {
                            const int ii = i + di, jj = j + dj, kk = k + dk;
                            if (ii < 0 || ii > n_ || jj < 0 || jj > n_ || kk < 0 || kk > n_) {
                                continue;
                            }
                            triplets.emplace_back(row, Index(ii, jj, kk), b1d[di + 1] * b1d[dj + 1] * b1d[dk + 1]);
                        }
                    }
                }
            }
        }
    }
    Eigen::SparseMatrix<double> A(total, total);
    A.setFromTriplets(triplets.begin(), triplets.end());

    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> cg(A);
    const Eigen::VectorXd a = cg.solve(c);
    alpha_.assign(a.data(), a.data() + total);
}

ImplicitBspline ImplicitBspline::MakePaca() {
    auto sphere = [](const Vec3& p, const Vec3& c, double r) {
        return (p - c).norm() - r;
    };
    auto smin = [](double a, double b, double k) {
        const double h = std::clamp(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
        return b + h * (a - b) - k * h * (1.0 - h);
    };

    ImplicitBspline spline(Vec3(-1.0, -1.0, -1.0), 2.0, 32);
    spline.Fit([&](const Vec3& p) {
        double d = sphere(p, Vec3(0.0, -0.15, 0.0), 0.45); // body
        d = smin(d, sphere(p, Vec3(0.35, 0.25, 0.0), 0.25), 0.1); // head
        d = smin(d, sphere(p, Vec3(-0.42, 0.05, 0.1), 0.18), 0.12); // tail
        d = smin(d, sphere(p, Vec3(0.5, 0.45, 0.08), 0.07), 0.05); // ear
        d = smin(d, sphere(p, Vec3(0.45, 0.45, -0.12), 0.07), 0.05); // ear
        return d;
    });
    return spline;
}

ImplicitBspline ImplicitBspline::MakeTorus() {
    const double major = 0.5;
    const double minor = 0.2;
    ImplicitBspline spline(Vec3(-1.0, -1.0, -1.0), 2.0, 32);
    spline.Fit([&](const Vec3& p) {
        const double ring = std::sqrt(p.x() * p.x() + p.z() * p.z()) - major;
        return std::sqrt(ring * ring + p.y() * p.y()) - minor;
    });
    return spline;
}

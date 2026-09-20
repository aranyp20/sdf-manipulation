#include "Visualization/Renderer.h"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <thread>

namespace {
constexpr int kMaxSteps = 128;
constexpr double kMaxDistance = 100.0;
constexpr double kHitEpsilon = 1e-4;

bool SphereTrace(const Implicit& sdf, const Ray& ray, double& tOut) {
    double t = 0.0;
    for (int i = 0; i < kMaxSteps && t < kMaxDistance; ++i) {
        const double d = sdf.Sdf(ray.origin + t * ray.dir);
        if (d < kHitEpsilon) {
            tOut = t;
            return true;
        }
        t += d;
    }
    return false;
}

} // namespace

void Renderer::ComputeNormalMap(const Implicit& sdf, const Camera& camera, int w, int h) {
    normalMap_.w = w;
    normalMap_.h = h;
    normalMap_.normals.assign(static_cast<size_t>(w) * h, Vec3::Zero());
    normalMap_.hit.assign(static_cast<size_t>(w) * h, 0);

    const unsigned numThreads = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::thread> threads;
    threads.reserve(numThreads);
    for (unsigned tid = 0; tid < numThreads; ++tid) {
        threads.emplace_back([&, tid] {
            for (int y = static_cast<int>(tid); y < h; y += static_cast<int>(numThreads)) {
                for (int x = 0; x < w; ++x) {
                    const Ray ray = camera.PixelRay(x, y, w, h);
                    double t;
                    if (SphereTrace(sdf, ray, t)) {
                        const size_t i = static_cast<size_t>(y) * w + x;
                        normalMap_.normals[i] = sdf.Grad(ray.origin + t * ray.dir).normalized();
                        normalMap_.hit[i] = 1;
                    }
                }
            }
        });
    }
    for (std::thread& t : threads) {
        t.join();
    }
}

void Renderer::ShadeDiffuse() {
    const Vec3 lightDir = Vec3(0.5, 0.7, 0.5).normalized();
    constexpr double ambient = 0.25;
    const size_t n = normalMap_.normals.size();
    pixels_.assign(n * 3, 0.0f);

    for (size_t i = 0; i < n; ++i) {
        if (!normalMap_.hit[i]) {
            pixels_[i * 3 + 0] = 0.1f;
            pixels_[i * 3 + 1] = 0.1f;
            pixels_[i * 3 + 2] = 0.1f;
            continue;
        }
        // Half-Lambert: wraps the light around, so no side goes fully dark.
        const double d = 0.5 + 0.5 * normalMap_.normals[i].dot(lightDir);
        const float v = static_cast<float>(std::min(1.0, ambient + (1.0 - ambient) * d));
        pixels_[i * 3 + 0] = v * 0.2f;
        pixels_[i * 3 + 1] = v * 0.4f;
        pixels_[i * 3 + 2] = v * 0.75f;
    }
}

void Renderer::Draw(int fbw, int fbh) const {
    if (normalMap_.w == 0 || normalMap_.h == 0 || pixels_.empty()) {
        return;
    }
    glRasterPos2d(-1.0, -1.0);
    glPixelZoom(static_cast<float>(fbw) / normalMap_.w, static_cast<float>(fbh) / normalMap_.h);
    glDrawPixels(normalMap_.w, normalMap_.h, GL_RGB, GL_FLOAT, pixels_.data());
    glPixelZoom(1.0f, 1.0f);
}

void Renderer::DrawPoints(const std::vector<Vec3>& points,
                          const Vec3& color,
                          float pointSize,
                          const Camera& camera,
                          int w,
                          int h) const {
    glPointSize(pointSize);
    glColor3d(color.x(), color.y(), color.z());
    glBegin(GL_POINTS);
    for (const Vec3& p : points) {
        Vec2 ndc;
        if (camera.ProjectToNdc(p, w, h, ndc)) {
            glVertex2d(ndc.x(), ndc.y());
        }
    }
    glEnd();
}

void Renderer::DrawLine(const Vec3& a, const Vec3& b, const Vec3& color, const Camera& camera, int w, int h) const {
    Vec2 na, nb;
    if (!camera.ProjectToNdc(a, w, h, na) || !camera.ProjectToNdc(b, w, h, nb)) {
        return;
    }
    glLineWidth(1.5f);
    glColor3d(color.x(), color.y(), color.z());
    glBegin(GL_LINES);
    glVertex2d(na.x(), na.y());
    glVertex2d(nb.x(), nb.y());
    glEnd();
}

// Straight 3D segments project to straight 2D segments under perspective, so
// drawing the corner-to-corner lines is exact.
void Renderer::DrawWireBox(const Box3& box, const Vec3& color, const Camera& camera, int w, int h) const {
    if (box.isEmpty()) {
        return;
    }
    // Corner c has max.x if bit 0 is set, max.y if bit 1, max.z if bit 2; edges
    // join corners differing in exactly one bit.
    constexpr int kEdges[12][2] = {
        {0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3}, {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    const Vec3& lo = box.min();
    const Vec3& hi = box.max();
    Vec3 corners[8];
    for (int c = 0; c < 8; ++c) {
        corners[c] = Vec3(c & 1 ? hi.x() : lo.x(), c & 2 ? hi.y() : lo.y(), c & 4 ? hi.z() : lo.z());
    }
    glLineWidth(1.5f);
    glColor3d(color.x(), color.y(), color.z());
    glBegin(GL_LINES);
    for (const auto& edge : kEdges) {
        Vec2 a, b;
        if (camera.ProjectToNdc(corners[edge[0]], w, h, a) && camera.ProjectToNdc(corners[edge[1]], w, h, b)) {
            glVertex2d(a.x(), a.y());
            glVertex2d(b.x(), b.y());
        }
    }
    glEnd();
}

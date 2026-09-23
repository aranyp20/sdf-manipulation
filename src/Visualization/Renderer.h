#pragma once

#include "Representation/Implicit.h"
#include "Visualization/Camera.h"

#include <vector>

struct NormalMap {
    std::vector<Vec3> normals; // per pixel, row-major from the bottom row
    std::vector<char> hit;
    int w = 0, h = 0;
};

class Renderer {
public:
    // Part 1: sphere trace the implicit and build a normal map.
    void ComputeNormalMap(const Implicit& sdf, const Camera& camera, int w, int h, int maxSteps = 128);

    // Part 2: shade the normal map with simple diffuse lighting into an RGB buffer.
    void ShadeDiffuse();

    // Draw the shaded buffer into a fbw x fbh framebuffer.
    void Draw(int fbw, int fbh) const;

    // Debug overlays, drawn on top of the image (no depth test).
    void
    DrawPoints(const std::vector<Vec3>& points, const Vec3& color, float pointSize, const Camera& camera, int w, int h) const;
    void DrawWireBox(const Box3& box, const Vec3& color, const Camera& camera, int w, int h) const;
    void DrawLine(const Vec3& a, const Vec3& b, const Vec3& color, const Camera& camera, int w, int h) const;

private:
    NormalMap normalMap_;
    std::vector<float> pixels_; // RGB float, same layout as the normal map
};

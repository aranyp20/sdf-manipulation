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
    void ComputeNormalMap(const Implicit& sdf, const Camera& camera, int w, int h);

    // Part 2: shade the normal map with simple diffuse lighting into an RGB buffer.
    void ShadeDiffuse();

    // Draw the shaded buffer into a fbw x fbh framebuffer.
    void Draw(int fbw, int fbh) const;

private:
    NormalMap normalMap_;
    std::vector<float> pixels_; // RGB float, same layout as the normal map
};

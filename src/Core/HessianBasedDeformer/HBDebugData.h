#pragma once

#include "Math/MyEigen.h"

#include <vector>

// Debug output of the Hessian-based deformer pipeline.
// Critical points grouped by the number of negative Hessian eigenvalues (paper Tab. 1).
struct HBDebugData {
    std::vector<Vec3> minimum; // 0 negative eigenvalues
    std::vector<Vec3> oneSaddle; // 1 negative eigenvalue
    std::vector<Vec3> twoSaddle; // 2 negative eigenvalues
    std::vector<Vec3> maximum; // 3 negative eigenvalues
};

#ifndef MULTINET_SURFACE_PROJECTION_H
#define MULTINET_SURFACE_PROJECTION_H

#include "multinet/core/coordinates.h"
#include <cstdint>

namespace Multinet {

constexpr uint32_t PROJECTION_VERSION_SIX_PARAMETER_COBE_V1 = 1;

namespace ProjectionCOBE {
    constexpr double LAMBDA = 0.7240;
    constexpr double GAMMA_10 = -0.0941;
    constexpr double GAMMA_01 = 0.0276;
    constexpr double GAMMA_20 = -0.0623;
    constexpr double GAMMA_11 = 0.0409;
    constexpr double GAMMA_02 = 0.0342;

    // Evaluates f(a, b) mapping 2D face coordinates to X or Z.
    double f_forward(double a, double b) noexcept;

    // Forward conversion: face index (0-5) and U,V in [-1, 1] to normalized 3D sphere coordinate.
    FramePosition64 map_forward(int face, double u, double v) noexcept;

    // Differential of map_forward with respect to normalized face U and V.
    // This is presentation support, not a second projection authority.
    bool map_forward_differential(
        int face,
        double u,
        double v,
        FramePosition64& out_du,
        FramePosition64& out_dv
    ) noexcept;

    // Validated inverse conversion: 3D sphere coordinate to canonical face and U,V.
    // Returns true if converged within tolerances.
    bool map_inverse(const FramePosition64& p, int expected_canonical_face, double& out_u, double& out_v, int& out_face) noexcept;
}

} // namespace Multinet

#endif // MULTINET_SURFACE_PROJECTION_H

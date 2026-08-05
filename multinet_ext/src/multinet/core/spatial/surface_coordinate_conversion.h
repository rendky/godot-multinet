#ifndef MULTINET_SURFACE_COORDINATE_CONVERSION_H
#define MULTINET_SURFACE_COORDINATE_CONVERSION_H

#include "surface_address.h"
#include "surface_frame.h"
#include "multinet/core/coordinates.h"

namespace Multinet {

struct WorldScaleManifest;

[[nodiscard]] bool try_surface_to_frame(
    const SurfacePosition64& position,
    const SurfaceFrame& frame,
    const WorldScaleManifest& scale,
    FramePosition64& out_position
) noexcept;

[[nodiscard]] bool try_frame_to_surface(
    const FramePosition64& position,
    const SurfaceFrame& frame,
    const WorldScaleManifest& scale,
    SurfacePosition64& out_position
) noexcept;

} // namespace Multinet

#endif // MULTINET_SURFACE_COORDINATE_CONVERSION_H

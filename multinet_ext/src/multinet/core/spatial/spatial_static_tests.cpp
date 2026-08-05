#include "surface_face.h"
#include "surface_address.h"
#include "surface_frame.h"
#include "surface_topology.h"
#include "surface_projection.h"
#include "world_manifests.h"
#include "multinet/core/coordinates.h"

#include <type_traits>

namespace Multinet {
namespace Tests {

// Exact SurfaceFace numeric values
static_assert(static_cast<uint8_t>(SurfaceFace::PositiveX) == 0);
static_assert(static_cast<uint8_t>(SurfaceFace::NegativeX) == 1);
static_assert(static_cast<uint8_t>(SurfaceFace::PositiveY) == 2);
static_assert(static_cast<uint8_t>(SurfaceFace::NegativeY) == 3);
static_assert(static_cast<uint8_t>(SurfaceFace::PositiveZ) == 4);
static_assert(static_cast<uint8_t>(SurfaceFace::NegativeZ) == 5);

// Structure field widths (address coordinates use signed 64-bit millimetres)
static_assert(sizeof(SurfaceAddress::u_mm) == 8);
static_assert(std::is_signed_v<decltype(SurfaceAddress::u_mm)>);

// Canonical constants
static_assert(TOPOLOGY_VERSION_SIX_FACE == 1);

// Explicit-conversion enforcement (No implicit canonical/local conversions)
static_assert(!std::is_convertible_v<FramePosition64, SurfacePosition64>);
static_assert(!std::is_convertible_v<FramePosition64, SurfaceAddress>);
static_assert(!std::is_convertible_v<SurfacePosition64, SurfaceAddress>);

// Trivially copyable
static_assert(std::is_trivially_copyable_v<SurfaceAddress>);
static_assert(std::is_trivially_copyable_v<SurfacePosition64>);
static_assert(std::is_trivially_copyable_v<FramePosition64>);
static_assert(std::is_trivially_copyable_v<LocalPosition32>);

} // namespace Tests
} // namespace Multinet

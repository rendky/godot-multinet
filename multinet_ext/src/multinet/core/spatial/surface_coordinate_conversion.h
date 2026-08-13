#ifndef MULTINET_SURFACE_COORDINATE_CONVERSION_H
#define MULTINET_SURFACE_COORDINATE_CONVERSION_H

#include "surface_address.h"
#include "surface_frame.h"
#include "surface_topology.h"
#include "multinet/core/coordinates.h"

namespace Multinet {

struct WorldScaleManifest;
struct WorldDomainManifest;

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

[[nodiscard]] bool try_domain_surface_to_frame(
	const SurfacePosition64& position,
	const SurfaceFrame& frame,
	const WorldDomainManifest& domain,
	FramePosition64& out_position
) noexcept;

[[nodiscard]] bool try_frame_to_domain_surface(
	const FramePosition64& position,
	const SurfaceFrame& frame,
	const WorldDomainManifest& domain,
	SurfacePosition64& out_position
) noexcept;

// Advances an observer using presentation-local components (X=surface U,
// Y=altitude, Z=surface V). Closed-surface edge transport and canonical face
// changes are owned by this topology/conversion layer; callers receive the
// resulting canonical position and accumulated flat frame.
[[nodiscard]] bool try_advance_domain_surface_frame(
	const FramePosition64& local_delta,
	const WorldDomainManifest& domain,
	const SurfaceFrame& initial_frame,
	SurfaceFrame& out_frame,
	SurfacePosition64& out_position,
	uint32_t& out_transition_count,
	SurfaceFace* out_last_source_face = nullptr,
	SurfaceFace* out_last_destination_face = nullptr,
	SurfaceEdge* out_last_edge = nullptr
) noexcept;

} // namespace Multinet

#endif // MULTINET_SURFACE_COORDINATE_CONVERSION_H

#ifndef MULTINET_SURFACE_ADDRESS_H
#define MULTINET_SURFACE_ADDRESS_H

#include "surface_face.h"
#include <cstdint>

namespace Multinet {

struct SurfaceAddress {
	SurfaceFace face{ SurfaceFace::PositiveX };

	int64_t u_mm{ 0 };
	int64_t v_mm{ 0 };
	int64_t altitude_mm{ 0 };

	uint32_t topology_version{ 1 };
	uint32_t projection_version{ 1 };

	[[nodiscard]] constexpr bool is_valid() const noexcept {
		return is_valid_surface_face(face);
	}

	[[nodiscard]] constexpr bool operator==(const SurfaceAddress &p_other) const noexcept {
		return face == p_other.face &&
			   u_mm == p_other.u_mm &&
			   v_mm == p_other.v_mm &&
			   altitude_mm == p_other.altitude_mm &&
			   topology_version == p_other.topology_version &&
			   projection_version == p_other.projection_version;
	}
};

struct SurfacePosition64 {
	SurfaceFace face{ SurfaceFace::PositiveX };

	double u_m{ 0.0 };
	double v_m{ 0.0 };
	double altitude_m{ 0.0 };

	uint32_t topology_version{ 1 };
	uint32_t projection_version{ 1 };

	[[nodiscard]] constexpr bool is_valid() const noexcept {
		return is_valid_surface_face(face);
	}
};

} // namespace Multinet

#include "world_manifests.h"

namespace Multinet {
SurfaceAddress canonicalize_surface_address(SurfaceAddress address, const WorldScaleManifest &scale) noexcept;
}

#endif // MULTINET_SURFACE_ADDRESS_H

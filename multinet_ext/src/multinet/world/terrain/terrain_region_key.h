#ifndef MULTINET_TERRAIN_REGION_KEY_H
#define MULTINET_TERRAIN_REGION_KEY_H

#include "multinet/core/spatial/surface_projection.h"
#include <cstdint>
#include <functional>

namespace Multinet {

struct SurfaceRegionID {
	SurfaceFace face{ SurfaceFace::PositiveX };

	uint16_t cell_u{ 0 };
	uint16_t cell_v{ 0 };

	uint16_t vertical_layer{ 0 };
	uint16_t reserved{ 0 };

	uint32_t topology_version{ 1 };

	[[nodiscard]] bool operator==(const SurfaceRegionID& other) const noexcept {
		return face == other.face &&
		       cell_u == other.cell_u &&
		       cell_v == other.cell_v &&
		       vertical_layer == other.vertical_layer &&
		       topology_version == other.topology_version;
	}

	[[nodiscard]] bool operator!=(const SurfaceRegionID& other) const noexcept {
		return !(*this == other);
	}
};

} // namespace Multinet

template <>
struct std::hash<Multinet::SurfaceRegionID> {
	std::size_t operator()(const Multinet::SurfaceRegionID& id) const noexcept {
		std::size_t h = static_cast<std::size_t>(id.face);
		h ^= (static_cast<std::size_t>(id.cell_u) << 8) + 0x9e3779b9 + (h << 6) + (h >> 2);
		h ^= (static_cast<std::size_t>(id.cell_v) << 16) + 0x9e3779b9 + (h << 6) + (h >> 2);
		h ^= (static_cast<std::size_t>(id.vertical_layer) << 24) + 0x9e3779b9 + (h << 6) + (h >> 2);
		h ^= (static_cast<std::size_t>(id.topology_version) << 32) + 0x9e3779b9 + (h << 6) + (h >> 2);
		return h;
	}
};

#endif // MULTINET_TERRAIN_REGION_KEY_H

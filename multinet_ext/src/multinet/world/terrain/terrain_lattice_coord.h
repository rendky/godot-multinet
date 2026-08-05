#ifndef MULTINET_TERRAIN_LATTICE_COORD_H
#define MULTINET_TERRAIN_LATTICE_COORD_H

#include <cstdint>
#include <functional>

namespace Multinet {

struct TerrainLatticeCoord {
	int64_t x{ 0 };
	int64_t z{ 0 };

	uint32_t channel{ 0 };
	uint32_t recipe_version{ 1 };

	[[nodiscard]] bool operator==(const TerrainLatticeCoord& other) const noexcept {
		return x == other.x &&
		       z == other.z &&
		       channel == other.channel &&
		       recipe_version == other.recipe_version;
	}

	[[nodiscard]] bool operator!=(const TerrainLatticeCoord& other) const noexcept {
		return !(*this == other);
	}
};

} // namespace Multinet

template <>
struct std::hash<Multinet::TerrainLatticeCoord> {
	std::size_t operator()(const Multinet::TerrainLatticeCoord& coord) const noexcept {
		std::size_t h = static_cast<std::size_t>(coord.x);
		h ^= (static_cast<std::size_t>(coord.z) << 16) + 0x9e3779b9 + (h << 6) + (h >> 2);
		h ^= (static_cast<std::size_t>(coord.channel) << 24) + 0x9e3779b9 + (h << 6) + (h >> 2);
		h ^= (static_cast<std::size_t>(coord.recipe_version) << 32) + 0x9e3779b9 + (h << 6) + (h >> 2);
		return h;
	}
};

#endif // MULTINET_TERRAIN_LATTICE_COORD_H

#ifndef MULTINET_BLOCK_CLIPMAP_IDS_H
#define MULTINET_BLOCK_CLIPMAP_IDS_H

#include "multinet/core/spatial/surface_face.h"
#include <cstdint>
#include <functional>

namespace multinet::rendering {

constexpr uint8_t ORDINARY_BCCM_V1_PROFILE = 0;

struct TerrainRenderBlockKey {
	Multinet::SurfaceFace face{ Multinet::SurfaceFace::PositiveX };

	int32_t block_u{0};
	int32_t block_v{0};

	uint8_t lod{0};
	uint8_t profile{0};
	uint16_t reserved{0};

	bool operator==(const TerrainRenderBlockKey &other) const {
		return face == other.face && block_u == other.block_u && block_v == other.block_v && lod == other.lod && profile == other.profile;
	}

	bool operator!=(const TerrainRenderBlockKey &other) const {
		return !(*this == other);
	}
};

} // namespace multinet::rendering

namespace Multinet {
	using multinet::rendering::ORDINARY_BCCM_V1_PROFILE;
}

namespace std {
template <>
struct hash<multinet::rendering::TerrainRenderBlockKey> {
	std::size_t operator()(const multinet::rendering::TerrainRenderBlockKey &k) const noexcept {
		std::size_t h1 = std::hash<uint8_t>{}(static_cast<uint8_t>(k.face));
		std::size_t h2 = std::hash<int32_t>{}(k.block_u);
		std::size_t h3 = std::hash<int32_t>{}(k.block_v);
		std::size_t h4 = std::hash<uint8_t>{}(k.lod);
		std::size_t h5 = std::hash<uint8_t>{}(k.profile);
		return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3) ^ (h5 << 4);
	}
};
} // namespace std

#endif // MULTINET_BLOCK_CLIPMAP_IDS_H

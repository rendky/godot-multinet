#ifndef MULTINET_BLOCK_CLIPMAP_IDS_H
#define MULTINET_BLOCK_CLIPMAP_IDS_H

#include <cstdint>
#include <functional>

namespace multinet::rendering {

struct TerrainRenderBlockKey {
	int64_t block_x{0};
	int64_t block_z{0};
	uint8_t lod{0};
	uint8_t profile{0};
	uint16_t reserved{0};

	bool operator==(const TerrainRenderBlockKey &other) const {
		return block_x == other.block_x && block_z == other.block_z && lod == other.lod && profile == other.profile;
	}

	bool operator!=(const TerrainRenderBlockKey &other) const {
		return !(*this == other);
	}
};

} // namespace multinet::rendering

namespace std {
template <>
struct hash<multinet::rendering::TerrainRenderBlockKey> {
	std::size_t operator()(const multinet::rendering::TerrainRenderBlockKey &k) const noexcept {
		std::size_t h1 = std::hash<int64_t>{}(k.block_x);
		std::size_t h2 = std::hash<int64_t>{}(k.block_z);
		std::size_t h3 = std::hash<uint8_t>{}(k.lod);
		return h1 ^ (h2 << 1) ^ (h3 << 2);
	}
};
} // namespace std

#endif // MULTINET_BLOCK_CLIPMAP_IDS_H

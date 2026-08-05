#ifndef MULTINET_BLOCK_CLIPMAP_PROFILE_H
#define MULTINET_BLOCK_CLIPMAP_PROFILE_H

#include <cstdint>

namespace multinet::rendering {

struct BlockClipmapProfile {
	static constexpr uint32_t QUADS_PER_EDGE = 16;
	static constexpr uint32_t VERTS_PER_EDGE = QUADS_PER_EDGE + 1; // 17
	static constexpr uint32_t TOTAL_VERTS = VERTS_PER_EDGE * VERTS_PER_EDGE; // 289
	static constexpr uint32_t TOTAL_TRIANGLES = QUADS_PER_EDGE * QUADS_PER_EDGE * 2; // 512
	static constexpr uint32_t TOTAL_INDICES = TOTAL_TRIANGLES * 3; // 1536

	uint8_t level_count{ 8 }; // Ordinary WP5 default
	float finest_spacing{ 2.0f }; // 2.0m per quad
	float lod0_block_size{ 32.0f }; // 16 * 2.0m = 32m per block edge

	int32_t candidate_grid_radius{ 4 }; // 8x8 grid around camera (-4 to +3)
	int32_t inner_hole_radius{ 2 }; // 4x4 hole for coarser levels (-2 to +1)

	static constexpr uint32_t MAX_CANDIDATES = 256;
	static constexpr uint32_t MAX_VISIBLE_INSTANCES = 256;
	static constexpr uint8_t MAX_LEVELS = 16; // hard limit for array allocation
	static constexpr uint32_t MAX_GRID_OFFSETS = 1024; // (2*16)^2 — worst-case grid radius 16

	float default_min_elevation{ -10.0f };
	float default_max_elevation{ 10.0f };

	float get_lod_spacing(uint8_t lod) const {
		return finest_spacing * static_cast<float>(1 << lod);
	}

	float get_lod_block_size(uint8_t lod) const {
		return lod0_block_size * static_cast<float>(1 << lod);
	}
};

} // namespace multinet::rendering

#endif // MULTINET_BLOCK_CLIPMAP_PROFILE_H

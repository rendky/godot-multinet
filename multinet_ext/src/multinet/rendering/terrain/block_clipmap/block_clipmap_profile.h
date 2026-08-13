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
	// The finest level has no hole, so its full square must fit the instance
	// buffers. Raising this means resizing the renderer buffers as one change.
	static constexpr int32_t MAX_SUPPORTED_CANDIDATE_GRID_RADIUS = 8;
	static_assert((2 * MAX_SUPPORTED_CANDIDATE_GRID_RADIUS) *
		(2 * MAX_SUPPORTED_CANDIDATE_GRID_RADIUS) == MAX_CANDIDATES,
		"candidate radius limit must match the instance capacity");
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

struct BlockClipmapLimits {
	size_t max_source_requests{ 64 };
	size_t max_page_commits{ 24 };
	static constexpr size_t MAX_SOURCE_REQUESTS = 64;
	static constexpr size_t MAX_PAGE_COMMITS = 24;
	static constexpr size_t MAX_FRAME_DEMAND = 512;
};

} // namespace multinet::rendering

#endif // MULTINET_BLOCK_CLIPMAP_PROFILE_H

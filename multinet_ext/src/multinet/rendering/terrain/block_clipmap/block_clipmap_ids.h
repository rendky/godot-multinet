#ifndef MULTINET_BLOCK_CLIPMAP_IDS_H
#define MULTINET_BLOCK_CLIPMAP_IDS_H

#include "multinet/core/spatial/surface_face.h"
#include <cstdint>
#include <functional>

namespace multinet::rendering {

constexpr uint8_t ORDINARY_BCCM_V1_PROFILE = 0;
constexpr uint8_t TERRAIN_SAMPLE_PATCH_MAPPING_V1 = 1;
constexpr uint8_t TERRAIN_SAMPLE_PATCH_MAPPING_COHERENT_V2 = 2;
constexpr uint8_t TERRAIN_SAMPLE_PATCH_MAPPING_LOGICAL_CHART_V3 = 3;
constexpr uint8_t TERRAIN_SAMPLE_PATCH_MAPPING_BOUNDED_CHART_V4 = 4;
constexpr uint8_t TERRAIN_SAMPLE_PATCH_MAPPING_LOCAL_EXP_CHART_V5 = 5;

struct TerrainRenderBlockKey {
	Multinet::SurfaceFace face{ Multinet::SurfaceFace::PositiveX };

	int32_t block_u{0};
	int32_t block_v{0};

	uint8_t lod{0};
	uint8_t profile{0};
	uint16_t reserved{0};

	[[nodiscard]] bool is_valid() const noexcept {
		return Multinet::is_valid_surface_face(face) && lod < 16;
	}

	bool operator==(const TerrainRenderBlockKey &other) const {
		return face == other.face && block_u == other.block_u && block_v == other.block_v && lod == other.lod && profile == other.profile;
	}

	bool operator!=(const TerrainRenderBlockKey &other) const {
		return !(*this == other);
	}
};

// A cell in the unwrapped, flat presentation lattice. Closed-surface
// canonicalization may map several of these cells to the same content block;
// they still remain distinct draw instances.
struct TerrainPresentationBlockKey {
	int64_t block_u{ 0 };
	int64_t block_v{ 0 };
	uint64_t unfolding_generation{ 0 };
	uint8_t lod{ 0 };
	uint8_t profile{ ORDINARY_BCCM_V1_PROFILE };

	[[nodiscard]] bool is_valid() const noexcept {
		return unfolding_generation != 0 && lod < 16;
	}

	bool operator==(const TerrainPresentationBlockKey& other) const noexcept {
		return block_u == other.block_u && block_v == other.block_v &&
			unfolding_generation == other.unfolding_generation &&
			lod == other.lod && profile == other.profile;
	}

	bool operator!=(const TerrainPresentationBlockKey& other) const noexcept {
		return !(*this == other);
	}
};

	// Exact CPU-owned mapping for one presentation block. V1 anchors each block at
	// its centre. V2 shares one cube-net path. V3 is the legacy exponential
	// chart. V4 is the retired bounded tangent experiment. V5 uses an
	// observer-local exponential chart with a separately enforced metric radius.
struct TerrainSamplePatchKey {
	Multinet::SurfaceFace anchor_face{ static_cast<Multinet::SurfaceFace>(255) };
	int64_t anchor_u_mm{ 0 };
	int64_t anchor_v_mm{ 0 };
	int64_t presentation_center_dx_mm{ 0 };
	int64_t presentation_center_dz_mm{ 0 };
	uint64_t unfolding_generation{ 0 };
	int8_t u_axis_x{ 1 };
	int8_t u_axis_z{ 0 };
	int8_t v_axis_x{ 0 };
	int8_t v_axis_z{ 1 };
	uint8_t lod{ 0 };
	uint8_t profile{ ORDINARY_BCCM_V1_PROFILE };
	uint8_t mapping_version{ TERRAIN_SAMPLE_PATCH_MAPPING_V1 };

	[[nodiscard]] bool is_valid() const noexcept {
		if (!Multinet::is_valid_surface_face(anchor_face) || lod >= 16 ||
			(mapping_version != TERRAIN_SAMPLE_PATCH_MAPPING_V1 &&
			 mapping_version != TERRAIN_SAMPLE_PATCH_MAPPING_COHERENT_V2 &&
			 mapping_version != TERRAIN_SAMPLE_PATCH_MAPPING_LOGICAL_CHART_V3 &&
			 mapping_version != TERRAIN_SAMPLE_PATCH_MAPPING_BOUNDED_CHART_V4 &&
			 mapping_version != TERRAIN_SAMPLE_PATCH_MAPPING_LOCAL_EXP_CHART_V5) ||
			(mapping_version != TERRAIN_SAMPLE_PATCH_MAPPING_V1 && unfolding_generation == 0)) return false;
		const int det = static_cast<int>(u_axis_x) * static_cast<int>(v_axis_z) -
			static_cast<int>(u_axis_z) * static_cast<int>(v_axis_x);
		const int u_len = static_cast<int>(u_axis_x) * static_cast<int>(u_axis_x) +
			static_cast<int>(u_axis_z) * static_cast<int>(u_axis_z);
		const int v_len = static_cast<int>(v_axis_x) * static_cast<int>(v_axis_x) +
			static_cast<int>(v_axis_z) * static_cast<int>(v_axis_z);
		return det == 1 && u_len == 1 && v_len == 1;
	}

	bool operator==(const TerrainSamplePatchKey& other) const noexcept {
		return anchor_face == other.anchor_face && anchor_u_mm == other.anchor_u_mm &&
			anchor_v_mm == other.anchor_v_mm &&
			presentation_center_dx_mm == other.presentation_center_dx_mm &&
			presentation_center_dz_mm == other.presentation_center_dz_mm &&
			unfolding_generation == other.unfolding_generation &&
			u_axis_x == other.u_axis_x &&
			u_axis_z == other.u_axis_z && v_axis_x == other.v_axis_x &&
			v_axis_z == other.v_axis_z && lod == other.lod &&
			profile == other.profile && mapping_version == other.mapping_version;
	}

	bool operator!=(const TerrainSamplePatchKey& other) const noexcept {
		return !(*this == other);
	}
};

} // namespace multinet::rendering

namespace Multinet {
	using multinet::rendering::ORDINARY_BCCM_V1_PROFILE;
	using multinet::rendering::TerrainPresentationBlockKey;
	using multinet::rendering::TerrainSamplePatchKey;
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

#ifndef MULTINET_REGION_TILE_H
#define MULTINET_REGION_TILE_H

#include "multinet/core/coordinates.h"
#include "multinet/core/dirty_bounds.h"
#include "multinet/core/span.h"
#include "multinet/core/io/bundle_io.h"
#include "multinet/core/memory/arena_allocator.h"
#include "multinet/world/terrain/terrain_queries.h"
#include "multinet/world/terrain/terrain_region_key.h"

#include <cstdint>
#include <cstring>
#include <cmath>

namespace Multinet {

struct TerrainVertex {
	float x{ 0.0f };
	float y{ 0.0f };
	float z{ 0.0f };
	float nx{ 0.0f };
	float ny{ 1.0f };
	float nz{ 0.0f };
};

class TerrainRegionTile {
private:
	SurfaceRegionID region_id{};
	static constexpr size_t GRID_DIM = 33;
	float heights[GRID_DIM * GRID_DIM]{};
	SurfaceNormal normals[GRID_DIM * GRID_DIM]{};
	SurfaceBounds dirty_bounds{};
	bool is_generated{ false };
	float actual_spacing{ 0.0f };

public:
	TerrainRegionTile() = default;

	explicit TerrainRegionTile(SurfaceRegionID p_region) : region_id(p_region) {}

	bool generate(const TerrainFieldEvaluator &p_evaluator, const WorldScaleManifest &p_scale) noexcept {
		if (region_id.cell_u >= p_scale.regions_per_face_axis || region_id.cell_v >= p_scale.regions_per_face_axis) {
			return false;
		}

		double H = static_cast<double>(p_scale.chart_half_extent_mm) * 0.001;
		double R = p_scale.actual_region_extent_m;
		double start_u = -H + static_cast<double>(region_id.cell_u) * R;
		double start_v = -H + static_cast<double>(region_id.cell_v) * R;

		double sample_spacing = R / static_cast<double>(GRID_DIM - 1);
		actual_spacing = static_cast<float>(sample_spacing);

		for (size_t z = 0; z < GRID_DIM; ++z) {
			for (size_t x = 0; x < GRID_DIM; ++x) {
				double u_offset = static_cast<double>(x) * sample_spacing;
				double v_offset = static_cast<double>(z) * sample_spacing;

				// The last region must terminate at the positive chart boundary within declared floating tolerance
				if (region_id.cell_u == p_scale.regions_per_face_axis - 1 && x == GRID_DIM - 1) {
					u_offset = H - start_u;
				}
				if (region_id.cell_v == p_scale.regions_per_face_axis - 1 && z == GRID_DIM - 1) {
					v_offset = H - start_v;
				}

				SurfacePosition64 pos{};
				pos.face = region_id.face;
				pos.u_m = start_u + u_offset;
				pos.v_m = start_v + v_offset;
				pos.altitude_m = 0.0;

				TerrainHeightEvaluation result = p_evaluator.evaluate(pos, TerrainQueryFlags::Normals);
				size_t idx = z * GRID_DIM + x;
				heights[idx] = static_cast<float>(result.height);
				normals[idx] = result.normal;
			}
		}

		SurfaceAddress corners[4];
		corners[0].face = region_id.face;
		corners[0].u_mm = static_cast<int64_t>(std::round(start_u * 1000.0));
		corners[0].v_mm = static_cast<int64_t>(std::round(start_v * 1000.0));

		corners[1].face = region_id.face;
		corners[1].u_mm = static_cast<int64_t>(std::round((start_u + R) * 1000.0));
		corners[1].v_mm = static_cast<int64_t>(std::round(start_v * 1000.0));

		corners[2].face = region_id.face;
		corners[2].u_mm = static_cast<int64_t>(std::round(start_u * 1000.0));
		corners[2].v_mm = static_cast<int64_t>(std::round((start_v + R) * 1000.0));

		corners[3].face = region_id.face;
		corners[3].u_mm = static_cast<int64_t>(std::round((start_u + R) * 1000.0));
		corners[3].v_mm = static_cast<int64_t>(std::round((start_v + R) * 1000.0));

		for (int i = 0; i < 4; ++i) {
			dirty_bounds.expand(canonicalize_surface_address(corners[i], p_scale));
		}
		
		is_generated = true;
		return true;
	}

	[[nodiscard]] float get_height_at_sample(size_t p_x, size_t p_z) const noexcept {
		if (p_x >= GRID_DIM || p_z >= GRID_DIM) return 0.0f;
		return heights[p_z * GRID_DIM + p_x];
	}

	bool export_jolt_collision_buffer(ArenaAllocator &p_arena, Span<const float> &r_jolt_buffer) noexcept {
		if (!is_generated) return false;

		size_t count = GRID_DIM * GRID_DIM;
		float *dst = p_arena.allocate<float>(count);
		if (!dst) return false;

		std::memcpy(dst, heights, count * sizeof(float));
		r_jolt_buffer = Span<const float>(dst, count);
		return true;
	}

	bool export_render_vertices(ArenaAllocator &p_arena, Span<const TerrainVertex> &r_vertex_buffer) noexcept {
		if (!is_generated) return false;

		size_t count = GRID_DIM * GRID_DIM;
		TerrainVertex *vertices = p_arena.allocate<TerrainVertex>(count);
		if (!vertices) return false;

		for (size_t z = 0; z < GRID_DIM; ++z) {
			for (size_t x = 0; x < GRID_DIM; ++x) {
				size_t idx = z * GRID_DIM + x;
				vertices[idx].x = static_cast<float>(x) * actual_spacing;
				vertices[idx].y = heights[idx];
				vertices[idx].z = static_cast<float>(z) * actual_spacing;
				vertices[idx].nx = normals[idx].nx;
				vertices[idx].ny = normals[idx].ny;
				vertices[idx].nz = normals[idx].nz;
			}
		}

		r_vertex_buffer = Span<const TerrainVertex>(vertices, count);
		return true;
	}

	[[nodiscard]] const SurfaceBounds &get_dirty_bounds() const noexcept { return dirty_bounds; }
	[[nodiscard]] bool has_valid_data() const noexcept { return is_generated; }
};

} // namespace Multinet

#endif // MULTINET_REGION_TILE_H

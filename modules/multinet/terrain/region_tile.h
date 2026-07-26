#ifndef MULTINET_REGION_TILE_H
#define MULTINET_REGION_TILE_H

#include "core/coordinates.h"
#include "core/dirty_bounds.h"
#include "core/span.h"
#include "io/bundle_io.h"
#include "memory/arena_allocator.h"
#include "terrain/heightfield_generator.h"

#include <cstdint>
#include <cstring>

namespace Multinet {

// ============================================================================
// Gate: TERRAIN-COLLISION-01 (Asynchronous Jolt Heightfield Tile)
// Gate: TERRAIN-STREAM-01 (HDD Streaming Locality Bundle Staging)
// Gate: TERRAIN-RENDER-01 (LivingWorldRendering Coarse LOD Handoff)
// ============================================================================

struct RegionID {
	int64_t cell_x{ 0 };
	int64_t cell_y{ 0 };
	int64_t cell_z{ 0 };
};

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
	RegionID region_id{};
	static constexpr size_t GRID_DIM = 33; // 33x33 sample grid for 1024m region (32 meter grid spacing)
	float heights[GRID_DIM * GRID_DIM]{};
	DirtyBounds3D dirty_bounds{};
	bool is_generated{ false };

public:
	TerrainRegionTile() = default;

	explicit TerrainRegionTile(RegionID p_region) : region_id(p_region) {}

	bool generate(const HeightfieldGenerator &p_generator) noexcept {
		RegionPosition base_pos{ region_id.cell_x, region_id.cell_y, region_id.cell_z, 0.0f, 0.0f, 0.0f };
		WorldPosition64 base_w = base_pos.to_world();

		constexpr float spacing = 1024.0f / static_cast<float>(GRID_DIM - 1);

		for (size_t z = 0; z < GRID_DIM; ++z) {
			for (size_t x = 0; x < GRID_DIM; ++x) {
				double wx = base_w.x + static_cast<double>(static_cast<float>(x) * spacing);
				double wz = base_w.z + static_cast<double>(static_cast<float>(z) * spacing);

				double h = p_generator.evaluate_height(wx, wz);
				heights[z * GRID_DIM + x] = static_cast<float>(h);
			}
		}

		dirty_bounds.expand(base_pos);
		is_generated = true;
		return true;
	}

	[[nodiscard]] float get_height_at_sample(size_t p_x, size_t p_z) const noexcept {
		if (p_x >= GRID_DIM || p_z >= GRID_DIM) return 0.0f;
		return heights[p_z * GRID_DIM + p_x];
	}

	// TERRAIN-COLLISION-01: Export heightfield data buffer for Jolt heightfield shape creation
	bool export_jolt_collision_buffer(ArenaAllocator &p_arena, Span<const float> &r_jolt_buffer) noexcept {
		if (!is_generated) return false;

		size_t count = GRID_DIM * GRID_DIM;
		float *dst = p_arena.allocate<float>(count);
		if (!dst) return false;

		std::memcpy(dst, heights, count * sizeof(float));
		r_jolt_buffer = Span<const float>(dst, count);
		return true;
	}

	// TERRAIN-RENDER-01: Export coarse mesh vertices for LivingWorldRendering LOD handoff
	bool export_render_vertices(ArenaAllocator &p_arena, Span<const TerrainVertex> &r_vertex_buffer) noexcept {
		if (!is_generated) return false;

		size_t count = GRID_DIM * GRID_DIM;
		TerrainVertex *vertices = p_arena.allocate<TerrainVertex>(count);
		if (!vertices) return false;

		constexpr float spacing = 1024.0f / static_cast<float>(GRID_DIM - 1);

		for (size_t z = 0; z < GRID_DIM; ++z) {
			for (size_t x = 0; x < GRID_DIM; ++x) {
				size_t idx = z * GRID_DIM + x;
				vertices[idx].x = static_cast<float>(x) * spacing;
				vertices[idx].y = heights[idx];
				vertices[idx].z = static_cast<float>(z) * spacing;
				vertices[idx].nx = 0.0f;
				vertices[idx].ny = 1.0f;
				vertices[idx].nz = 0.0f;
			}
		}

		r_vertex_buffer = Span<const TerrainVertex>(vertices, count);
		return true;
	}

	[[nodiscard]] const DirtyBounds3D &get_dirty_bounds() const noexcept { return dirty_bounds; }
	[[nodiscard]] bool has_valid_data() const noexcept { return is_generated; }
};

} // namespace Multinet

#endif // MULTINET_REGION_TILE_H

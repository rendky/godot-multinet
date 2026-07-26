#ifndef MULTINET_TERRAIN_QUERIES_H
#define MULTINET_TERRAIN_QUERIES_H

#include "core/coordinates.h"
#include "core/span.h"
#include "memory/arena_allocator.h"
#include "terrain/heightfield_generator.h"

namespace Multinet {

// ============================================================================
// Gate: TERRAIN-QUERY-01 (Zero-Allocation Batched Terrain Queries)
// Gate: TERRAIN-860M-01 (Sub-millisecond Desktop Floor Performance)
// ============================================================================

struct TerrainQueryResult {
	double height{ 0.0 };
	SurfaceNormal normal{};
};

class TerrainBatchQuery {
public:
	static bool query_batch(
			const HeightfieldGenerator &p_generator,
			Span<const WorldPosition64> p_inputs,
			Span<TerrainQueryResult> r_outputs) noexcept {
		if (p_inputs.size() != r_outputs.size()) return false;

		for (size_t i = 0; i < p_inputs.size(); ++i) {
			r_outputs[i].height = p_generator.evaluate_height(p_inputs[i]);
			r_outputs[i].normal = p_generator.evaluate_normal(p_inputs[i].x, p_inputs[i].z);
		}

		return true;
	}

	static TerrainQueryResult *allocate_and_query_batch(
			const HeightfieldGenerator &p_generator,
			Span<const WorldPosition64> p_inputs,
			ArenaAllocator &p_arena) noexcept {
		if (p_inputs.empty()) return nullptr;

		TerrainQueryResult *out_buf = p_arena.allocate<TerrainQueryResult>(p_inputs.size());
		if (!out_buf) return nullptr;

		Span<TerrainQueryResult> out_span(out_buf, p_inputs.size());
		if (!query_batch(p_generator, p_inputs, out_span)) return nullptr;

		return out_buf;
	}
};

} // namespace Multinet

#endif // MULTINET_TERRAIN_QUERIES_H

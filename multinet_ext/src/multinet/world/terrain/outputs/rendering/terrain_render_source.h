#ifndef MULTINET_TERRAIN_RENDER_SOURCE_H
#define MULTINET_TERRAIN_RENDER_SOURCE_H

#include "multinet/world/terrain/terrain_recipe_identity.h"
#include "multinet/world/terrain/terrain_queries.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_ids.h"

#include <cstdint>
#include <array>

namespace Multinet {

struct TerrainFallbackBounds {
	float minimum_height{ 0.0f };
	float maximum_height{ 0.0f };
	float residual_bound{ 1.0f };
	float morph_allowance{ 0.0f };
};

enum class TerrainPageGenerationMode : uint8_t {
	AsynchronousProduction,
	SynchronousDiagnostic
};

struct TerrainRenderSourceSnapshot {
	TerrainRecipeIdentity recipe_identity;

	uint64_t world_manifest_hash{ 0 };

	uint32_t topology_version{ 1 };
	uint32_t projection_version{ 1 };
	uint32_t terrain_version{ 1 };
	uint32_t source_version{ 1 };

	TerrainFallbackBounds fallback_bounds;
};

enum class TerrainSourceState : uint8_t {
	Pending,
	Ready,
	Missing,
	Invalid  // Evaluation failed; do not re-enqueue without a version change
};

struct TerrainHeightPage {
	// 19x19 R32F. Texels 1..17 are actual BCCM vertices. Texel 0 and 18 are canonical one-sample apron.
	std::array<float, 19 * 19> heights{};
};

struct TerrainSourceRecord {
	multinet::rendering::TerrainRenderBlockKey canonical_key;

	uint64_t cpu_page_handle{ 0 };
	uint32_t cpu_page_generation{ 0 };

	TerrainSourceState state{ TerrainSourceState::Pending };

	uint32_t terrain_version{ 1 };
	uint32_t source_version{ 1 };

	float min_height{ 0.0f };
	float max_height{ 0.0f };
	float residual_bound{ 0.0f };
	float morph_allowance{ 0.0f };
	float gradient_bound{ 0.0f };

	uint64_t previous_fallback_handle{ 0 };
};

class TerrainRenderSource {
public:
	virtual ~TerrainRenderSource() = default;

	virtual TerrainRenderSourceSnapshot get_snapshot() const noexcept = 0;
	virtual TerrainSourceRecord get_or_request_record(const multinet::rendering::TerrainRenderBlockKey& key) noexcept = 0;
	virtual void commit_pending_requests(const multinet::rendering::TerrainRenderBlockKey& camera_key) noexcept = 0;

	// Reads the page for a given handle only if the expected_generation matches.
	// Rejects: out-of-range handle, inactive slot, non-Ready state, generation mismatch.
	virtual bool try_read_page(
		uint64_t handle,
		uint32_t expected_generation,
		TerrainHeightPage& out_page
	) const noexcept = 0;
};

} // namespace Multinet

#endif // MULTINET_TERRAIN_RENDER_SOURCE_H

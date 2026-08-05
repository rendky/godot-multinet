#ifndef MULTINET_CONCRETE_TERRAIN_RENDER_SOURCE_H
#define MULTINET_CONCRETE_TERRAIN_RENDER_SOURCE_H

#include "multinet/world/terrain/outputs/rendering/terrain_render_source.h"
#include "multinet/world/terrain/terrain_queries.h"
#include "multinet/core/spatial/surface_projection.h"
#include "multinet/core/jobs/bounded_background_job_executor.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_profile.h"

#include <atomic>
#include <mutex>
#include <array>
#include <optional>
#include <cmath>

namespace Multinet {

struct ConcreteTerrainSourceKey {
	multinet::rendering::TerrainRenderBlockKey block_key;
	
	uint64_t world_manifest_hash{0};
	uint64_t recipe_hash{0};
	
	uint32_t terrain_version{1};
	uint32_t source_version{1};
	
	bool operator==(const ConcreteTerrainSourceKey& other) const noexcept {
		return block_key.face == other.block_key.face &&
		       block_key.block_u == other.block_key.block_u &&
		       block_key.block_v == other.block_key.block_v &&
		       block_key.lod == other.block_key.lod &&
		       block_key.profile == other.block_key.profile &&
		       world_manifest_hash == other.world_manifest_hash &&
		       recipe_hash == other.recipe_hash &&
		       terrain_version == other.terrain_version &&
		       source_version == other.source_version;
	}
};

struct ConcreteTerrainPageData {
	ConcreteTerrainSourceKey source_key;
	TerrainHeightPage page;
	
	float min_height{0.0f};
	float max_height{0.0f};
	float gradient_bound{0.0f};
	float residual_bound{0.0f};
	float morph_allowance{0.0f};
	
	uint32_t generation{0};
	bool is_valid{false};
};

class ConcreteTerrainRenderSource final : public TerrainRenderSource {
private:
	TerrainRecipe recipe;
	WorldScaleManifest manifest;
	
	TerrainRenderSourceSnapshot current_snapshot;
	
	// Profile consistency: reject requests with unknown profile or out-of-range LOD.
	multinet::rendering::BlockClipmapProfile profile;

	// CPU page pool.
	static constexpr size_t CPU_PAGE_POOL_SIZE = 2048;
	static constexpr size_t PENDING_REQUEST_CAPACITY = 256;
	
	struct RecordSlot {
		TerrainSourceRecord record;
		ConcreteTerrainSourceKey full_key;
		uint32_t generation{ 0 };  // incremented on every reuse
		bool in_use{false};
	};
	
	mutable std::mutex access_mutex;
	
	std::unique_ptr<std::array<RecordSlot, CPU_PAGE_POOL_SIZE>> slots;
	std::unique_ptr<std::array<ConcreteTerrainPageData, CPU_PAGE_POOL_SIZE>> page_data;
	
	// Pending queue stores the identities needed for generation-safe publication.
	struct PendingRequest {
		ConcreteTerrainSourceKey key;
		uint64_t slot_handle;
		uint32_t slot_generation;   // generation at time of enqueue
		uint64_t manifest_hash;
		uint64_t recipe_hash;
		uint32_t terrain_version;
		uint32_t source_version;
		multinet::rendering::TerrainRenderBlockKey canonical_key;
	};
	
	std::array<PendingRequest, PENDING_REQUEST_CAPACITY> pending_queue;
	size_t pending_queue_size{0};
	
	uint32_t next_generation{1};

	// Background executor (not owned; lifetime guaranteed by caller).
	BoundedBackgroundJobExecutor& executor;

	// Lifetime protection for in-flight background jobs.
	std::atomic<bool> shutting_down_{ false };
	std::atomic<uint32_t> in_flight_count_{ 0 };
	std::mutex shutdown_mutex_;
	std::condition_variable shutdown_cv_;

	TerrainPageGenerationMode generation_mode{ TerrainPageGenerationMode::AsynchronousProduction };

	void decrement_in_flight() noexcept;
	std::optional<uint64_t> find_existing_slot(const ConcreteTerrainSourceKey& key) const noexcept;
	std::optional<uint64_t> allocate_slot() noexcept;

	// Submits one generation job to the background executor. Returns false on overflow.
	[[nodiscard]] bool submit_generation_job(const PendingRequest& req) noexcept;

public:
	ConcreteTerrainRenderSource(
		const TerrainRecipe& p_recipe,
		const WorldScaleManifest& p_manifest,
		BoundedBackgroundJobExecutor& p_executor,
		TerrainPageGenerationMode p_mode = TerrainPageGenerationMode::AsynchronousProduction
	);
	~ConcreteTerrainRenderSource() override;

	void set_generation_mode(TerrainPageGenerationMode mode) noexcept { generation_mode = mode; }
	[[nodiscard]] TerrainPageGenerationMode get_generation_mode() const noexcept { return generation_mode; }
	
	// Cancels all queued jobs; waits for in-flight jobs to complete.
	void shutdown() noexcept;
	
	TerrainRenderSourceSnapshot get_snapshot() const noexcept override;
	TerrainSourceRecord get_or_request_record(const multinet::rendering::TerrainRenderBlockKey& key) noexcept override;
	void commit_pending_requests(const multinet::rendering::TerrainRenderBlockKey& camera_key) noexcept override;
	bool try_read_page(
		uint64_t handle,
		uint32_t expected_generation,
		TerrainHeightPage& out_page
	) const noexcept override;

#ifdef DEBUG_ENABLED
	// Synchronous generation for diagnostics and gate fixtures.
	void process_pending_jobs_sync(size_t max_jobs) noexcept;
	void test_set_profile_block_size(float sz) { profile.lod0_block_size = sz; profile.finest_spacing = sz / 16.0f; }
#endif
};

} // namespace Multinet

#endif // MULTINET_CONCRETE_TERRAIN_RENDER_SOURCE_H

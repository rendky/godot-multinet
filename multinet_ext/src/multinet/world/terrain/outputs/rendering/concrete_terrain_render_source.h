#ifndef MULTINET_CONCRETE_TERRAIN_RENDER_SOURCE_H
#define MULTINET_CONCRETE_TERRAIN_RENDER_SOURCE_H

#include "multinet/world/terrain/outputs/rendering/terrain_render_source.h"
#include "multinet/world/terrain/terrain_queries.h"
#include "multinet/world/terrain/terrain_committed_delta.h"
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
	multinet::rendering::TerrainSamplePatchKey sample_patch;
	
	uint64_t world_manifest_hash{0};
	uint64_t recipe_hash{0};
	
	uint32_t page_contract_version{ TERRAIN_PAGE_CONTRACT_VERSION_1 };
	TerrainPagePayloadKind payload_kind{ TerrainPagePayloadKind::AdditiveHeightDeltaV1 };

	uint32_t terrain_version{1};
	uint32_t source_version{1};
	uint32_t committed_delta_version{1};
	uint32_t block_delta_content_version{1};
	
	bool operator==(const ConcreteTerrainSourceKey& other) const noexcept {
		if (payload_kind == TerrainPagePayloadKind::AdditiveHeightDeltaV1) {
			return block_key == other.block_key &&
			       sample_patch == other.sample_patch &&
			       world_manifest_hash == other.world_manifest_hash &&
			       recipe_hash == other.recipe_hash &&
			       page_contract_version == other.page_contract_version &&
			       payload_kind == other.payload_kind &&
			       terrain_version == other.terrain_version &&
			       source_version == other.source_version &&
			       block_delta_content_version == other.block_delta_content_version;
		}
		return block_key == other.block_key &&
		       sample_patch == other.sample_patch &&
		       world_manifest_hash == other.world_manifest_hash &&
		       recipe_hash == other.recipe_hash &&
		       page_contract_version == other.page_contract_version &&
		       payload_kind == other.payload_kind &&
		       terrain_version == other.terrain_version &&
		       source_version == other.source_version &&
		       committed_delta_version == other.committed_delta_version;
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
	WorldDomainManifest domain;
	
	TerrainRenderSourceSnapshot current_snapshot;
	TerrainCommittedDeltaSnapshot current_delta_snapshot;
	
	// Profile consistency: reject requests with unknown profile or out-of-range LOD.
	multinet::rendering::BlockClipmapProfile profile;

	// CPU page pool.
	static constexpr size_t CPU_PAGE_POOL_SIZE = 2048;
	static constexpr size_t PENDING_REQUEST_CAPACITY = FRAME_DEMAND_CAPACITY;
	
	struct RecordSlot {
		TerrainSourceRecord record;
		ConcreteTerrainSourceKey full_key;
		uint32_t generation{ 0 };  // incremented on every reuse
		std::atomic<uint64_t> cancellation_generation{ 0 }; // advanced on wanted-set removal, supersede, reuse, shutdown
		bool in_use{false};
	};

	mutable std::mutex access_mutex;

	std::unique_ptr<std::array<RecordSlot, CPU_PAGE_POOL_SIZE>> slots;
	std::unique_ptr<std::array<ConcreteTerrainPageData, CPU_PAGE_POOL_SIZE>> page_data;

	// Pending queue stores the identities needed for generation-safe publication.
	struct PendingRequest {
		TerrainPageRequestContext context;
		uint64_t slot_handle;
		uint32_t slot_generation;   // generation at time of enqueue
		uint64_t cancellation_generation; // cancellation generation at time of enqueue
		TerrainRequestClass request_class{ TerrainRequestClass::ImmediateVisible };
		int64_t distance_sq_m{ 0 };
		uint64_t wanted_set_epoch{ 0 };
		std::chrono::steady_clock::time_point enqueue_time{ std::chrono::steady_clock::now() };
	};

	std::unique_ptr<std::array<PendingRequest, PENDING_REQUEST_CAPACITY>> pending_queue;
	size_t pending_queue_size{0};

	// Full wanted-set tracking (capacity 512, matching FRAME_DEMAND_CAPACITY)
	static constexpr size_t WANTED_SET_CAPACITY = FRAME_DEMAND_CAPACITY;
	struct WantedKeyEntry {
		TerrainPageRequestIdentity identity;
		TerrainRequestClass request_class{ TerrainRequestClass::ImmediateVisible };
		int64_t distance_sq_m{ 0 };
	};

	std::unique_ptr<std::array<WantedKeyEntry, WANTED_SET_CAPACITY>> current_wanted_set;
	size_t current_wanted_set_size_{ 0 };
	std::unique_ptr<std::array<WantedKeyEntry, WANTED_SET_CAPACITY>> last_wanted_keys_;
	size_t last_wanted_keys_count_{ 0 };
	uint64_t wanted_set_epoch_{ 0 };
	bool wanted_set_overflow_{ false };

	std::atomic<uint32_t> pending_poison_count_{ 0 };
	std::atomic<uint32_t> cancelled_retryable_count_{ 0 };
	std::atomic<float> queue_age_terminal_ms_{ 0.0f };
	std::atomic<float> queue_age_coverage_ms_{ 0.0f };

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
	ConcreteTerrainRenderSource(
		const TerrainRecipe& p_recipe,
		const WorldDomainManifest& p_domain,
		BoundedBackgroundJobExecutor& p_executor,
		TerrainPageGenerationMode p_mode = TerrainPageGenerationMode::AsynchronousProduction
	);
	~ConcreteTerrainRenderSource() override;

	void set_generation_mode(TerrainPageGenerationMode mode) noexcept { generation_mode = mode; }
	[[nodiscard]] TerrainPageGenerationMode get_generation_mode() const noexcept { return generation_mode; }

	void set_payload_kind(TerrainPagePayloadKind kind) noexcept;

	void set_committed_delta_snapshot(const TerrainCommittedDeltaSnapshot& snapshot) noexcept;
	[[nodiscard]] TerrainCommittedDeltaSnapshot get_committed_delta_snapshot() const noexcept;

	// Cancels all queued jobs; waits for in-flight jobs to complete.
	void shutdown() noexcept;

	void cancel_all_page_work_and_advance_epoch() noexcept override;

	TerrainRenderSourceSnapshot get_snapshot() const noexcept override;
	TerrainRenderPublicationView get_publication_view() const noexcept override;
	TerrainSourceRecord get_or_request_record(const multinet::rendering::TerrainRenderBlockKey& key) noexcept override;

	bool try_query_record(
		const TerrainPageRequestIdentity& identity,
		TerrainSourceRecord& out_record
	) const noexcept override;

	bool try_query_record(
		const multinet::rendering::TerrainRenderBlockKey& key,
		TerrainSourceRecord& out_record
	) const noexcept override;

	TerrainSourceRequestResult request_record(
		const TerrainPageRequestContext& context,
		const TerrainRequestMetadata& metadata
	) noexcept override;

	TerrainSourceRequestResult request_record(
		const multinet::rendering::TerrainRenderBlockKey& key,
		const TerrainRequestMetadata& metadata
	) noexcept override;

	void begin_wanted_set(uint64_t epoch) noexcept override;
	[[nodiscard]] bool mark_wanted(
		const TerrainPageRequestIdentity& identity,
		TerrainRequestClass request_class,
		int64_t distance_sq_m,
		uint64_t epoch
	) noexcept override;

	[[nodiscard]] bool mark_wanted(
		const multinet::rendering::TerrainRenderBlockKey& key,
		TerrainRequestClass request_class,
		int64_t distance_sq_m,
		uint64_t epoch
	) noexcept override;

	void end_wanted_set() noexcept override;

	void commit_pending_requests(const multinet::rendering::TerrainRenderBlockKey& camera_key) noexcept override;
	bool try_read_page(
		uint64_t handle,
		uint32_t expected_generation,
		TerrainHeightPage& out_page
	) const noexcept override;

	size_t in_use_record_count() const noexcept;
	size_t missing_in_use_record_count() const noexcept;

	uint32_t get_pending_queue_count() const noexcept;
	uint32_t get_pending_record_count() const noexcept;
	uint32_t get_in_flight_count() const noexcept;
	uint64_t get_executor_submit_count() const noexcept;
	uint32_t get_cancelled_incompatible_count() const noexcept;
	uint64_t get_request_record_call_count() const noexcept;

	uint64_t get_rejected_delta_publication_count() const noexcept override;
	uint32_t get_ready_record_count() const noexcept override;
	uint32_t get_ready_empty_record_count() const noexcept override;
	uint32_t get_invalid_record_count() const noexcept override;
	uint32_t get_missing_record_count() const noexcept override;

	uint32_t get_pending_poison_count() const noexcept { return pending_poison_count_.load(std::memory_order_relaxed); }
	uint32_t get_cancelled_retryable_count() const noexcept { return cancelled_retryable_count_.load(std::memory_order_relaxed); }
	uint64_t get_commit_pending_call_count() const noexcept { return commit_pending_call_count_.load(std::memory_order_relaxed); }
	float get_queue_age_terminal_ms() const noexcept { return queue_age_terminal_ms_.load(std::memory_order_relaxed); }
	float get_queue_age_coverage_ms() const noexcept { return queue_age_coverage_ms_.load(std::memory_order_relaxed); }
	bool get_wanted_set_overflow() const noexcept { return wanted_set_overflow_; }

private:
	std::atomic<uint64_t> commit_pending_call_count_{ 0 };
	std::atomic<uint64_t> request_record_call_count_{ 0 };
	std::atomic<uint64_t> executor_submit_count_{ 0 };
	std::atomic<uint64_t> rejected_delta_publication_count_{ 0 };

	void release_unpublished_slot(uint64_t handle, uint32_t expected_generation) noexcept;

public:
#ifdef DEBUG_ENABLED
	// Synchronous generation for diagnostics and gate fixtures.
	void process_pending_jobs_sync(size_t max_jobs) noexcept;
	void test_set_profile_block_size(float sz) { profile.lod0_block_size = sz; profile.finest_spacing = sz / 16.0f; }
#endif
};

} // namespace Multinet

#endif // MULTINET_CONCRETE_TERRAIN_RENDER_SOURCE_H

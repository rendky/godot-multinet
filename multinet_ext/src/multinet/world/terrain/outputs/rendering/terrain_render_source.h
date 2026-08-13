#ifndef MULTINET_TERRAIN_RENDER_SOURCE_H
#define MULTINET_TERRAIN_RENDER_SOURCE_H

#include "multinet/world/terrain/terrain_recipe_identity.h"
#include "multinet/world/terrain/terrain_queries.h"
#include "multinet/world/terrain/terrain_committed_delta.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_ids.h"

#include <cstdint>
#include <array>

namespace Multinet {

enum class TerrainPagePayloadKind : uint8_t {
	AbsoluteHeightDebugV1 = 0,
	AdditiveHeightDeltaV1 = 1
};

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

	uint32_t page_contract_version{ TERRAIN_PAGE_CONTRACT_VERSION_1 };
	TerrainPagePayloadKind payload_kind{ TerrainPagePayloadKind::AdditiveHeightDeltaV1 };

	uint32_t topology_version{ 1 };
	uint32_t projection_version{ 1 };
	uint32_t terrain_version{ 1 };
	uint32_t source_version{ 1 };
	uint32_t committed_delta_version{ 1 };

	TerrainFallbackBounds fallback_bounds;
};

enum class TerrainSourceState : uint8_t {
	Pending,
	Ready,
	ReadyEmpty,
	Missing,
	Invalid  // Evaluation failed; do not re-enqueue without a version change
};

struct TerrainHeightPage {
	uint32_t page_contract_version{ TERRAIN_PAGE_CONTRACT_VERSION_1 };
	TerrainPagePayloadKind payload_kind{ TerrainPagePayloadKind::AdditiveHeightDeltaV1 };
	uint32_t committed_delta_version{ 1 };
	uint32_t block_delta_content_version{ 1 };

	// 19x19 R32F scalar page. Texels 1..17 are actual BCCM vertices. Texel 0 and 18 are canonical one-sample apron.
	union {
		std::array<float, 19 * 19> heights;
		std::array<float, 19 * 19> samples_m;
	};

	TerrainHeightPage() : samples_m{} {}
	TerrainHeightPage(const TerrainHeightPage& other) : samples_m(other.samples_m) {
		page_contract_version = other.page_contract_version;
		payload_kind = other.payload_kind;
		committed_delta_version = other.committed_delta_version;
		block_delta_content_version = other.block_delta_content_version;
	}
	TerrainHeightPage& operator=(const TerrainHeightPage& other) {
		if (this != &other) {
			page_contract_version = other.page_contract_version;
			payload_kind = other.payload_kind;
			committed_delta_version = other.committed_delta_version;
			block_delta_content_version = other.block_delta_content_version;
			samples_m = other.samples_m;
		}
		return *this;
	}
};

using TerrainScalarPage19 = TerrainHeightPage;

struct TerrainSourceRecord {
	multinet::rendering::TerrainRenderBlockKey canonical_key;

	uint64_t cpu_page_handle{ 0 };
	uint32_t cpu_page_generation{ 0 };

	TerrainSourceState state{ TerrainSourceState::Pending };

	uint32_t page_contract_version{ TERRAIN_PAGE_CONTRACT_VERSION_1 };
	TerrainPagePayloadKind payload_kind{ TerrainPagePayloadKind::AdditiveHeightDeltaV1 };

	uint32_t terrain_version{ 1 };
	uint32_t source_version{ 1 };
	uint32_t committed_delta_version{ 1 };
	uint32_t block_delta_content_version{ 1 };

	float min_height{ 0.0f };
	float max_height{ 0.0f };
	float residual_bound{ 0.0f };
	float morph_allowance{ 0.0f };
	float gradient_bound{ 0.0f };

	uint64_t previous_fallback_handle{ 0 };
};

static constexpr size_t FRAME_DEMAND_CAPACITY = 512;

enum class TerrainRequestClass : uint8_t {
	TerminalBootstrap = 0,
	CoarseCoverage = 1,
	ImmediateVisible = 2,
	AtomicSibling = 3,
	Prefetch = 4
};

constexpr uint8_t admission_rank(TerrainRequestClass req_class) noexcept {
	switch (req_class) {
		case TerrainRequestClass::TerminalBootstrap: return 0;
		case TerrainRequestClass::ImmediateVisible: return 1;
		case TerrainRequestClass::AtomicSibling:    return 1;
		case TerrainRequestClass::CoarseCoverage:   return 2;
		case TerrainRequestClass::Prefetch:         return 3;
		default:                                     return 4;
	}
}

constexpr TerrainRequestClass merge_priority(TerrainRequestClass existing, TerrainRequestClass incoming) noexcept {
	return (admission_rank(incoming) < admission_rank(existing)) ? incoming : existing;
}

constexpr uint64_t INVALID_CPU_PAGE_HANDLE = 0xFFFFFFFFFFFFFFFFULL;

enum class TerrainSourceRequestDisposition : uint8_t {
	ExistingPending,
	ExistingReady,
	ExistingReadyEmpty,
	ExistingInvalid,
	CreatedPending,
	CreatedReadyEmpty,
	RejectedPoolFull,
	RejectedQueueFull,
	RejectedCancelled
};

struct TerrainSourceRequestResult {
	TerrainSourceRecord record{};
	TerrainSourceRequestDisposition disposition{ TerrainSourceRequestDisposition::RejectedPoolFull };
};

struct TerrainRequestMetadata {
	TerrainRequestClass request_class{ TerrainRequestClass::ImmediateVisible };
	int64_t distance_sq_m{ 0 };
	uint64_t wanted_set_epoch{ 0 };
};

struct TerrainRenderPublicationView {
	TerrainRenderSourceSnapshot source;
	TerrainCommittedDeltaSnapshot committed_delta;
};

struct TerrainPageRequestIdentity {
	multinet::rendering::TerrainRenderBlockKey block_key;
	multinet::rendering::TerrainSamplePatchKey sample_patch;
	uint64_t world_manifest_hash{ 0 };
	uint64_t recipe_hash{ 0 };
	uint32_t page_contract_version{ TERRAIN_PAGE_CONTRACT_VERSION_1 };
	TerrainPagePayloadKind payload_kind{ TerrainPagePayloadKind::AdditiveHeightDeltaV1 };
	uint32_t terrain_version{ 1 };
	uint32_t source_version{ 1 };
	uint32_t publication_version{ 1 };
	uint32_t block_content_version{ 1 };

	[[nodiscard]] bool operator==(const TerrainPageRequestIdentity& o) const noexcept {
		return block_key == o.block_key &&
			sample_patch == o.sample_patch &&
			world_manifest_hash == o.world_manifest_hash &&
			recipe_hash == o.recipe_hash &&
			page_contract_version == o.page_contract_version &&
			payload_kind == o.payload_kind &&
			terrain_version == o.terrain_version &&
			source_version == o.source_version &&
			publication_version == o.publication_version &&
			block_content_version == o.block_content_version;
	}
};

struct TerrainPageRequestContext {
	TerrainPageRequestIdentity identity;
	TerrainCommittedDeltaSnapshot committed_delta;
};

inline double required_page_apron_m(
	const multinet::rendering::TerrainRenderBlockKey& key,
	const multinet::rendering::BlockClipmapProfile& profile
) noexcept {
	return static_cast<double>(profile.get_lod_spacing(key.lod));
}

inline TerrainPageRequestContext make_page_request_context(
	const multinet::rendering::TerrainRenderBlockKey& key,
	const multinet::rendering::BlockClipmapProfile& profile,
	const TerrainRenderPublicationView& publication,
	const WorldScaleManifest& manifest
) {
	TerrainPageRequestContext ctx{};
	ctx.identity.block_key = key;
	ctx.identity.world_manifest_hash = publication.source.world_manifest_hash;
	ctx.identity.recipe_hash = publication.source.recipe_identity.recipe_hash;
	ctx.identity.page_contract_version = publication.source.page_contract_version;
	ctx.identity.payload_kind = publication.source.payload_kind;
	ctx.identity.terrain_version = publication.source.terrain_version;
	ctx.identity.source_version = publication.source.source_version;
	ctx.identity.publication_version = publication.source.committed_delta_version;

	double apron = required_page_apron_m(key, profile);
	if (publication.source.payload_kind == TerrainPagePayloadKind::AdditiveHeightDeltaV1 && publication.committed_delta.field) {
		ctx.identity.block_content_version = publication.committed_delta.field->get_block_content_version(key, manifest, profile, apron);
	} else {
		ctx.identity.block_content_version = 1;
	}

	ctx.committed_delta = publication.committed_delta;
	return ctx;
}

inline TerrainPageRequestContext make_page_request_context(
	const multinet::rendering::TerrainRenderBlockKey& key,
	const multinet::rendering::TerrainSamplePatchKey& sample_patch,
	const multinet::rendering::BlockClipmapProfile& profile,
	const TerrainRenderPublicationView& publication,
	const WorldScaleManifest& manifest
) {
	TerrainPageRequestContext ctx = make_page_request_context(key, profile, publication, manifest);
	ctx.identity.sample_patch = sample_patch;
	if (sample_patch.is_valid() &&
		publication.source.payload_kind == TerrainPagePayloadKind::AdditiveHeightDeltaV1 &&
		publication.committed_delta.field) {
		// A presentation patch may cross several canonical content blocks. The
		// publication version is the only honest invalidation token for that set.
		ctx.identity.block_content_version = publication.committed_delta.publication_version;
	}
	return ctx;
}

class TerrainRenderSource {
public:
	virtual ~TerrainRenderSource() = default;

	virtual TerrainRenderSourceSnapshot get_snapshot() const noexcept = 0;

	virtual TerrainRenderPublicationView get_publication_view() const noexcept {
		return TerrainRenderPublicationView{ get_snapshot(), get_committed_delta_snapshot() };
	}

	// Deprecated backward-compatible wrapper
	virtual TerrainSourceRecord get_or_request_record(const multinet::rendering::TerrainRenderBlockKey& key) noexcept = 0;

	// Pure non-allocating query bound to request identity
	virtual bool try_query_record(
		const TerrainPageRequestIdentity& identity,
		TerrainSourceRecord& out_record
	) const noexcept = 0;

	virtual bool try_query_record(
		const multinet::rendering::TerrainRenderBlockKey& key,
		TerrainSourceRecord& out_record
	) const noexcept = 0;

	// Source allocation request bound to request context
	virtual TerrainSourceRequestResult request_record(
		const TerrainPageRequestContext& context,
		const TerrainRequestMetadata& metadata
	) noexcept = 0;

	virtual TerrainSourceRequestResult request_record(
		const multinet::rendering::TerrainRenderBlockKey& key,
		const TerrainRequestMetadata& metadata
	) noexcept = 0;

	virtual void begin_wanted_set(uint64_t epoch) noexcept = 0;
	[[nodiscard]] virtual bool mark_wanted(
		const TerrainPageRequestIdentity& identity,
		TerrainRequestClass request_class,
		int64_t distance_sq_m,
		uint64_t epoch
	) noexcept = 0;

	[[nodiscard]] virtual bool mark_wanted(
		const multinet::rendering::TerrainRenderBlockKey& key,
		TerrainRequestClass request_class,
		int64_t distance_sq_m,
		uint64_t epoch
	) noexcept = 0;

	virtual void end_wanted_set() noexcept = 0;

	virtual void commit_pending_requests(const multinet::rendering::TerrainRenderBlockKey& camera_key) noexcept = 0;

	virtual bool try_read_page(
		uint64_t handle,
		uint32_t expected_generation,
		TerrainHeightPage& out_page
	) const noexcept = 0;

	virtual TerrainCommittedDeltaSnapshot get_committed_delta_snapshot() const noexcept { return {}; }
	virtual void set_committed_delta_snapshot(const TerrainCommittedDeltaSnapshot& snapshot) noexcept {}
	virtual void set_payload_kind(TerrainPagePayloadKind kind) noexcept {}
	virtual void cancel_all_page_work_and_advance_epoch() noexcept {}

	virtual uint32_t get_pending_record_count() const noexcept { return 0; }
	virtual uint32_t get_in_flight_count() const noexcept { return 0; }
	virtual uint32_t get_cancelled_incompatible_count() const noexcept { return 0; }
	virtual uint64_t get_commit_pending_call_count() const noexcept { return 0; }
	virtual uint64_t get_request_record_call_count() const noexcept { return 0; }
	virtual uint64_t get_executor_submit_count() const noexcept { return 0; }
	virtual uint64_t get_rejected_delta_publication_count() const noexcept { return 0; }
	virtual uint32_t get_ready_record_count() const noexcept { return 0; }
	virtual uint32_t get_ready_empty_record_count() const noexcept { return 0; }
	virtual uint32_t get_invalid_record_count() const noexcept { return 0; }
	virtual uint32_t get_missing_record_count() const noexcept { return 0; }
};

} // namespace Multinet

#endif // MULTINET_TERRAIN_RENDER_SOURCE_H

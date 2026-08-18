#ifndef MULTINET_BLOCK_CLIPMAP_RENDERER_H
#define MULTINET_BLOCK_CLIPMAP_RENDERER_H

#include "multinet/rendering/terrain/block_clipmap/block_clipmap_profile.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_state.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_culling.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_shader.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_ids.h"
#include "multinet/rendering/chp/chp_view.h"
#include "multinet/world/terrain/outputs/rendering/terrain_render_source.h"

#ifndef MULTINET_TEST
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#else
#include <vector>
namespace godot {
	class RenderingServer;
	class Camera3D;
	class World3D;
	class Image;
}
#endif

#include <algorithm>
#include <array>
#include <cmath>

#include "multinet/core/spatial/surface_address.h"
#include "multinet/core/spatial/surface_frame.h"
#include "multinet/core/spatial/world_manifests.h"
#include "multinet/world/terrain/outputs/rendering/terrain_render_source.h"
#include "multinet/world/terrain/terrain_recipe_identity.h"

namespace multinet::rendering {

struct BCCMCameraState {
	Multinet::SurfacePosition64 canonical_position;
	Multinet::SurfaceFrame active_frame;
	uint64_t frame_epoch{ 0 };

	// Persistent flat coordinates keep the clipmap lattice phase continuous
	// when canonical U/V wrap onto another face. Explicit teleports start a new
	// unfolding generation instead of guessing continuity.
	double presentation_x_m{ 0.0 };
	double presentation_z_m{ 0.0 };
	uint64_t unfolding_generation{ 1 };
	bool has_presentation_position{ false };
	Multinet::SurfaceFrame unfolding_root_frame{};
	double unfolding_root_presentation_x_m{ 0.0 };
	double unfolding_root_presentation_z_m{ 0.0 };
	bool has_unfolding_root{ false };
	// Analytic V5 owns a continuous geometric chart. It is intentionally
	// separate from active_frame: cube-face aliases may change at a corner,
	// while the displayed tangent axes must stay continuous.
	Multinet::FramePosition64 logical_chart_root_direction{};
	Multinet::FramePosition64 logical_chart_presentation_x_tangent{};
	Multinet::FramePosition64 logical_chart_presentation_z_tangent{};
	bool has_logical_chart{ false };

	// Maps active-frame placement coordinates into the viewport's presentation
	// space. Runtime uses the identity binding; the editor rebases this origin
	// around its actual viewport camera without changing canonical truth.
	godot::Basis presentation_basis{};
	godot::Vector3 presentation_origin{};
	bool has_presentation_binding{ false };

	bool is_visible{ true };
};

// External authority used to validate each source snapshot at update start.
struct BCCMSourceExpectation {
	Multinet::TerrainRecipeIdentity recipe_identity;
	uint64_t world_manifest_hash{ 0 };
	uint32_t topology_version{ 1 };
	uint32_t projection_version{ 1 };
	uint32_t terrain_version{ 1 };
	uint32_t source_version{ 1 };
	uint32_t gpu_analytic_version{ CANONICAL_ANALYTIC_TERRAIN_GPU_VERSION_1 };
};

struct TerrainUpdateResult {
	struct TextureUpload {
		uint8_t lod{ 0 };
		uint32_t gpu_layer{ 0 };
		uint32_t staging_index{ 0 };
		// Full identity for two-phase finalization inside update().
		TerrainRenderBlockKey canonical_key;
		TerrainSamplePatchKey sample_patch;
		uint64_t cpu_page_handle{ 0 };
		uint32_t cpu_page_generation{ 0 };
		uint32_t page_contract_version{ Multinet::TERRAIN_PAGE_CONTRACT_VERSION_1 };
		Multinet::TerrainPagePayloadKind payload_kind{ Multinet::TerrainPagePayloadKind::AdditiveHeightDeltaV1 };
		uint32_t terrain_version{ 1 };
		uint32_t source_version{ 1 };
		uint32_t committed_delta_version{ 1 };
		uint32_t block_delta_content_version{ 1 };
	};

	static constexpr uint32_t MAX_TEXTURE_UPLOADS = 24;
	std::array<TextureUpload, MAX_TEXTURE_UPLOADS> texture_uploads{};
	uint32_t texture_upload_count{ 0 };

	struct LODResult {
		uint32_t visible_count{ 0 };
		bool buffer_changed{ false };
	};
	std::array<LODResult, 16> lods{};
};

struct TerrainGpuPageIdentity {
	TerrainRenderBlockKey key{};
	TerrainSamplePatchKey sample_patch{};
	uint32_t page_contract_version{ Multinet::TERRAIN_PAGE_CONTRACT_VERSION_1 };
	Multinet::TerrainPagePayloadKind payload_kind{ Multinet::TerrainPagePayloadKind::AdditiveHeightDeltaV1 };
	uint32_t terrain_version{ 1 };
	uint32_t source_version{ 1 };
	uint32_t block_delta_content_version{ 1 };

	bool operator==(const TerrainGpuPageIdentity& other) const noexcept {
		return key == other.key &&
		       sample_patch == other.sample_patch &&
		       page_contract_version == other.page_contract_version &&
		       payload_kind == other.payload_kind &&
		       terrain_version == other.terrain_version &&
		       source_version == other.source_version &&
		       block_delta_content_version == other.block_delta_content_version;
	}
	bool operator!=(const TerrainGpuPageIdentity& other) const noexcept {
		return !(*this == other);
	}
};

inline bool exact_page_identity_match(const TerrainGpuPageIdentity& a, const TerrainGpuPageIdentity& b) noexcept {
	return a == b;
}

inline bool same_spatial_block(const TerrainGpuPageIdentity& a, const TerrainGpuPageIdentity& b) noexcept {
	return a.key == b.key && a.sample_patch == b.sample_patch;
}

inline bool stale_same_block_version(const TerrainGpuPageIdentity& a, const TerrainGpuPageIdentity& b) noexcept {
	return same_spatial_block(a, b) && (a.block_delta_content_version != b.block_delta_content_version ||
	                          a.source_version != b.source_version ||
	                          a.terrain_version != b.terrain_version ||
	                          a.payload_kind != b.payload_kind ||
	                          a.page_contract_version != b.page_contract_version);
}

inline bool compatible_stale_additive_page(
	const TerrainGpuPageIdentity& stale,
	const TerrainGpuPageIdentity& requested
) noexcept {
	return same_spatial_block(stale, requested) &&
	       stale.page_contract_version == requested.page_contract_version &&
	       stale.payload_kind == requested.payload_kind &&
	       stale.terrain_version == requested.terrain_version &&
	       stale.block_delta_content_version != requested.block_delta_content_version;
}

struct ResolvedTerrainLayer {
	uint32_t layer{ 0 };
	bool exact_current_resident{ false };
	bool exact_current_ready_empty{ false };
	bool using_stale_previous{ false };
	bool no_delta_content{ false };
	TerrainGpuPageIdentity selected_identity{};
};

enum class TerrainGpuPageState : uint8_t { Free, UploadPending, Resident, Retiring };

struct DetailedRendererDiagnostics {
	uint32_t analytic_base_visible_instances{ 0 };
	uint32_t hybrid_visible_instances{ 0 };
	uint32_t hybrid_zero_delta_instances{ 0 };
	uint32_t hybrid_exact_resident_instances{ 0 };
	uint32_t hybrid_using_stale_previous_instances{ 0 };
	uint32_t hybrid_ready_empty_instances{ 0 };
	uint32_t absolute_debug_visible_instances{ 0 };
	uint32_t absolute_debug_missing_page_instances{ 0 };
	uint32_t visible_constant_fallback_instances{ 0 };
	uint32_t dirty_replacements_pending{ 0 };
	uint32_t stale_delta_pages_retained{ 0 };
	uint32_t resident_delta_layers{ 0 };
	uint32_t upload_pending_delta_layers{ 0 };
	uint32_t resident_additive_delta_layers{ 0 };
	uint32_t upload_pending_additive_delta_layers{ 0 };
	uint32_t resident_absolute_debug_layers{ 0 };
	uint32_t incompatible_jobs_cancelled{ 0 };
	uint32_t source_pending_count{ 0 };
	uint32_t source_in_flight_count{ 0 };
	uint64_t executor_submit_count{ 0 };
	uint64_t commit_pending_call_count{ 0 };
	uint64_t request_record_call_count{ 0 };
	uint64_t rejected_delta_publication_count{ 0 };
	uint32_t expected_source_version{ 0 };
	uint32_t actual_source_version{ 0 };
	uint32_t publication_version{ 1 };
	uint32_t page_contract_version{ 1 };
	Multinet::TerrainPagePayloadKind payload_kind{ Multinet::TerrainPagePayloadKind::AdditiveHeightDeltaV1 };
	uint32_t block_content_version{ 1 };
	uint32_t selected_block_content_version{ 1 };

	bool candidate_overflow{ false };
	bool source_admission_overflow{ false };
	bool ready_upload_overflow{ false };
	bool frame_demand_overflow{ false };
};

struct TerrainSourceDiagnosticsSnapshot {
	uint32_t source_pending_count{ 0 };
	uint32_t source_in_flight_count{ 0 };
	uint64_t executor_submit_count{ 0 };
	uint32_t incompatible_jobs_cancelled{ 0 };
	uint64_t commit_pending_call_count{ 0 };
	uint64_t request_record_call_count{ 0 };
	uint64_t rejected_delta_publication_count{ 0 };
};

struct VisibleInstanceDiagnostic {
	TerrainRenderBlockKey key{};
	uint32_t gpu_layer{ 0 };
};

struct RollingLatencyTracker {
	static constexpr size_t CAPACITY = 64;
	std::array<float, CAPACITY> samples{};
	size_t head{ 0 };
	size_t count{ 0 };

	void add(float val_ms) noexcept {
		samples[head] = val_ms;
		head = (head + 1) % CAPACITY;
		if (count < CAPACITY) ++count;
	}

	[[nodiscard]] float average() const noexcept {
		if (count == 0) return 0.0f;
		float sum = 0.0f;
		for (size_t i = 0; i < count; ++i) sum += samples[i];
		return sum / static_cast<float>(count);
	}

	[[nodiscard]] float p95() const noexcept {
		if (count == 0) return 0.0f;
		std::array<float, CAPACITY> sorted;
		for (size_t i = 0; i < count; ++i) sorted[i] = samples[i];
		std::sort(sorted.begin(), sorted.begin() + count);
		size_t idx = static_cast<size_t>(std::ceil(0.95f * static_cast<float>(count))) - 1;
		if (idx >= count) idx = count - 1;
		return sorted[idx];
	}
};

struct LODCutDiagnostics {
	int64_t prev_center_bx{ 0 };
	int64_t prev_center_bv{ 0 };
	int64_t current_center_bx{ 0 };
	int64_t current_center_bv{ 0 };

	int64_t delta_center_bx{ 0 };
	int64_t delta_center_bv{ 0 };

	double delta_center_u_m{ 0.0 };
	double delta_center_v_m{ 0.0 };

	double snap_period_m{ 0.0 };
	uint32_t snap_steps_crossed_u{ 0 };
	uint32_t snap_steps_crossed_v{ 0 };
	uint32_t max_snap_steps_crossed{ 0 };
	bool skipped_snap_event{ false };

	// Inner-hole ownership diagnostics for LOD1..7
	int32_t prev_hole_dx{ 0 };
	int32_t prev_hole_dz{ 0 };
	int32_t current_hole_dx{ 0 };
	int32_t current_hole_dz{ 0 };
	int32_t hole_delta_dx{ 0 };
	int32_t hole_delta_dz{ 0 };
	bool hole_movement_event{ false };
	uint32_t hole_steps_crossed{ 0 };

	uint32_t candidate_count_before{ 0 };
	uint32_t candidate_count_after{ 0 };
	uint32_t candidates_retained{ 0 };
	uint32_t candidates_added{ 0 };
	uint32_t candidates_removed{ 0 };
	float turnover_fraction{ 0.0f };

	uint32_t submitted_instance_count{ 0 };
	bool instance_buffer_changed{ false };
	size_t instance_bytes_uploaded{ 0 };
};

struct FrameCutDiagnostics {
	uint64_t render_update_serial{ 0 };

	double presentation_observer_x_m{ 0.0 };
	double presentation_observer_z_m{ 0.0 };
	uint8_t canonical_face{ 0 };
	double canonical_u_m{ 0.0 };
	double canonical_v_m{ 0.0 };

	double camera_delta_x_m{ 0.0 };
	double camera_delta_z_m{ 0.0 };
	double ground_plane_distance_moved_m{ 0.0 };
	double delta_seconds{ 0.0 };
	double estimated_speed_m_s{ 0.0 };
	double estimated_speed_km_s{ 0.0 };

	uint8_t active_lod_count{ 0 };
	uint32_t total_instances_submitted{ 0 };
	uint32_t bccm_streams_submitted{ 0 };
	uint32_t multimesh_buffers_rewritten{ 0 };
	size_t total_instance_bytes_uploaded{ 0 };

	bool presentation_rebase_occurred{ false };
	double rebase_shift_x_m{ 0.0 };
	double rebase_shift_z_m{ 0.0 };
	uint64_t rebase_count{ 0 };

	bool freeze_update_active{ false };
	bool chp_effective{ false };
	float chp_signed_altitude_m{ 0.0f };

	uint32_t frame_skipped_snap_events{ 0 };
	uint32_t frame_largest_snap_steps{ 0 };
	uint8_t worst_lod{ 0 };
	uint8_t worst_axis{ 0 }; // 0: none, 1: U, 2: V, 3: both
	float worst_candidate_turnover{ 0.0f };

	std::array<LODCutDiagnostics, BlockClipmapProfile::MAX_LEVELS> lods{};
};


enum class ResolutionClass : uint8_t {
	Analytic,
	ExactResident,
	ExactReadyEmpty,
	StalePrevious,
	NoContent,
	AbsoluteResident,
	AbsoluteAnalyticFallback
};

struct SubmittedInstance {
	TerrainRenderBlockKey key;
	TerrainPresentationBlockKey presentation_key{};
	TerrainSamplePatchKey sample_patch{};
	godot::Basis block_to_active_frame;
	godot::Vector3 local_origin;
	godot::AABB local_aabb;
	uint32_t gpu_layer{ 0 };
	uint8_t edge_mask{ 0 };
	bool is_coverage_parent{ false };
	uint32_t page_contract_version{ Multinet::TERRAIN_PAGE_CONTRACT_VERSION_1 };
	Multinet::TerrainPagePayloadKind payload_kind{ Multinet::TerrainPagePayloadKind::AdditiveHeightDeltaV1 };
	uint32_t requested_block_content_version{ 1 };
	uint32_t block_delta_content_version{ 1 };
	ResolutionClass resolution_class{ ResolutionClass::NoContent };
};

#ifdef DEBUG_ENABLED
struct DebugBlockReplacementState {
	bool submitted{ false };
	TerrainRenderBlockKey key{};
	uint32_t selected_gpu_layer{ 0 };
	ResolutionClass resolution_class{ ResolutionClass::NoContent };
	uint32_t requested_content_version{ 1 };
	uint32_t selected_content_version{ 1 };
	TerrainGpuPageState selected_slot_state{ TerrainGpuPageState::Resident };
	uint32_t resident_same_block_count{ 0 };
	uint32_t upload_pending_same_block_count{ 0 };
	uint32_t retiring_same_block_count{ 0 };
	uint32_t retiring_content_version{ 0 };
	uint64_t retire_after_frame{ 0 };
};
#endif

struct FrameTerrainSubmissionPlan {
	struct LODPlan {
		std::array<SubmittedInstance, BlockClipmapProfile::MAX_CANDIDATES> instances{};
		uint32_t count{ 0 };
		uint32_t lod_0_6_layer_zero_count{ 0 };
		uint32_t lod_7_layer_zero_count{ 0 };
		uint32_t parent_covered_region_count{ 0 };
	};
	std::array<LODPlan, BlockClipmapProfile::MAX_LEVELS> lods{};
	bool valid{ false };
};

struct StreamingDiagnosticsSnapshot {
	uint32_t enumerated_candidates{ 0 };
	uint32_t frustum_visible_candidates{ 0 };
	uint32_t frustum_culled_candidates{ 0 };
	uint32_t resident_visible_candidates{ 0 };
	uint32_t parent_covered_visible_regions{ 0 };
	uint32_t layer_zero_visible_instances{ 0 };
	uint32_t lod_0_6_layer_zero_instances{ 0 };
	uint32_t lod_7_layer_zero_instances{ 0 };

	uint32_t new_source_requests_accepted{ 0 };
	uint32_t existing_pending_lookups{ 0 };
	size_t source_pending_queue_depth{ 0 };

	size_t immediate_visible_queue_depth{ 0 };
	size_t coarse_coverage_queue_depth{ 0 };
	size_t prefetch_queue_depth{ 0 };
	size_t executing_worker_count{ 0 };

	uint32_t ready_pages_awaiting_gpu{ 0 };
	uint32_t pages_uploaded_this_frame{ 0 };
	uint32_t terminal_bootstrap_pending{ 0 };

	uint32_t frame_demand_count{ 0 };
	uint32_t wanted_set_count{ 0 };
	bool wanted_set_overflow{ false };
	uint32_t pending_poison_count{ 0 };
	uint32_t cancelled_retryable_count{ 0 };
	uint32_t terminal_bootstrap_required{ 0 };
	uint32_t terminal_bootstrap_resident{ 0 };
	uint32_t lod_7_layer_zero_visible{ 0 };
	uint32_t previous_plan_retained_due_to_flat_bootstrap{ 0 };
	uint32_t closed_placement_failures{ 0 };
	uint32_t canonical_duplicate_presentations_retained{ 0 };
	uint32_t maximum_patch_transition_count{ 0 };
	float queue_age_terminal_ms{ 0.0f };
	float queue_age_coverage_ms{ 0.0f };

	uint32_t next_ring_terminal_keys_required{ 0 };
	uint32_t next_ring_terminal_keys_resident{ 0 };
	bool ring_transaction_pending{ false };
	float ring_transaction_age_ms{ 0.0f };

	uint64_t cancelled_stale_jobs{ 0 };
	uint64_t completed_stale_jobs{ 0 };

	float avg_generation_latency_ms{ 0.0f };
	float p95_generation_latency_ms{ 0.0f };
	float avg_request_to_resident_latency_ms{ 0.0f };
	float p95_request_to_resident_latency_ms{ 0.0f };
};

struct BlockPlacement {
	godot::Basis block_to_active_frame;
	godot::Vector3 local_origin;
	godot::AABB local_aabb;
	bool valid{ true };
};

struct RendererDiagnosticSnapshot {
	struct SlotSnapshot {
		TerrainGpuPageState state{ TerrainGpuPageState::Free };
		uint64_t cpu_page_handle{ 0 };
		uint32_t cpu_page_generation{ 0 };
		uint32_t gpu_layer{ 0 };
		uint64_t last_referenced_frame{ 0 };
		uint64_t retire_after_frame{ 0 };
		bool is_fallback{ false };
		TerrainRenderBlockKey key;
		TerrainSamplePatchKey sample_patch;
	};

	struct LODSnapshot {
		uint32_t candidate_count{ 0 };
		uint32_t visible_count{ 0 };
		uint32_t visible_keys_count{ 0 };
		uint32_t resident_visible_keys_count{ 0 };
		uint32_t ready_awaiting_gpu_count{ 0 };
		uint32_t uploaded_this_frame_count{ 0 };
		uint32_t layer_zero_visible_count{ 0 };
		uint32_t next_snap_prefetch_keys_count{ 0 };
		uint32_t next_ring_terminal_keys_required{ 0 };
		uint32_t next_ring_terminal_keys_resident{ 0 };
		bool ring_transaction_pending{ false };
		float ring_transaction_age_ms{ 0.0f };
		std::array<SlotSnapshot, 128> slots{};
		std::vector<TerrainRenderBlockKey> candidate_keys;
		std::vector<VisibleInstanceDiagnostic> submitted_visible_diagnostics;
		size_t ring_buffer_float_count{ 0 };
	};

	std::array<LODSnapshot, BlockClipmapProfile::MAX_LEVELS> lods{};
	StreamingDiagnosticsSnapshot streaming_diagnostics{};
};


TerrainRenderBlockKey make_canonical_block_key(
	Multinet::SurfaceFace face,
	int64_t block_u,
	int64_t block_v,
	uint8_t lod,
	const Multinet::WorldScaleManifest& manifest
);

[[nodiscard]] bool make_domain_block_key(
	Multinet::SurfaceFace input_chart,
	int64_t block_u,
	int64_t block_v,
	uint8_t lod,
	const Multinet::WorldDomainManifest& domain,
	TerrainRenderBlockKey& out_key
) noexcept;

TerrainRenderBlockKey derive_canonical_parent_key(
	const TerrainRenderBlockKey& child_key,
	uint8_t target_parent_lod,
	const Multinet::WorldScaleManifest& manifest
);

void enumerate_canonical_child_keys(
	const TerrainRenderBlockKey& parent_key,
	const Multinet::WorldScaleManifest& manifest,
	std::array<TerrainRenderBlockKey, 4>& out_children
);

[[nodiscard]] bool derive_domain_parent_key(
	const TerrainRenderBlockKey& child_key,
	uint8_t target_parent_lod,
	const Multinet::WorldDomainManifest& domain,
	TerrainRenderBlockKey& out_key
) noexcept;

[[nodiscard]] uint32_t enumerate_domain_child_keys(
	const TerrainRenderBlockKey& parent_key,
	const Multinet::WorldDomainManifest& domain,
	std::array<TerrainRenderBlockKey, 4>& out_children
) noexcept;

#ifdef MULTINET_TEST
using RenderID = uint64_t;
static_assert(sizeof(RenderID) == 8, "RenderID must be 8 bytes");
static_assert(std::is_same_v<RenderID, uint64_t>, "RenderID must be uint64_t in test");
#else
using RenderID = godot::RID;
#endif

class BlockClipmapRenderer {
private:
	BlockClipmapProfile profile{};
	BlockClipmapLimits limits_{};
	TerrainSourceDiagnosticsSnapshot last_source_diagnostics_{};
	TerrainSourceMode source_mode{ TerrainSourceMode::AnalyticBase };
	bool analytic_debug_prewarm_pages{ false };
	bool face_colors_enabled{ true };
	bool diamond_triangulation_enabled{ true };
	int chp_debug_reconstruction_mode{ 2 };
	bool chp_debug_negative_height_color{ false };
	bool chp_debug_negative_height_exaggeration{ false };
	int bccm_debug_visual_mode{ 0 };
	Multinet::TerrainFallbackBounds fallback_bounds{};
	Multinet::WorldDomainManifest active_domain{};

	// Copied from the current camera state for block-placement construction.
	// Keeping this at the renderer boundary makes candidate AABBs and submitted
	// transforms pass through the same presentation transform.
	godot::Basis active_presentation_basis{};
	godot::Vector3 active_presentation_origin{};
	godot::Vector3 active_view_world_position{};
	bool has_active_presentation_binding{ false };
	TerrainSamplePatchKey bound_logical_chart_root_{};
	double bound_logical_chart_root_presentation_x_m_{ 0.0 };
	double bound_logical_chart_root_presentation_z_m_{ 0.0 };
	bool has_bound_logical_chart_root_{ false };

	RenderID master_mesh_rid;
	RenderID legacy_mesh_rid;
	BCCMShaderData shader_data;

	struct GpuPageSlot {
		TerrainGpuPageState state{ TerrainGpuPageState::Free };
		uint64_t cpu_page_handle{ 0 };
		uint32_t cpu_page_generation{ 0 };
		uint32_t gpu_layer{ 0 };

		uint32_t page_contract_version{ Multinet::TERRAIN_PAGE_CONTRACT_VERSION_1 };
		Multinet::TerrainPagePayloadKind payload_kind{ Multinet::TerrainPagePayloadKind::AdditiveHeightDeltaV1 };

		uint32_t terrain_version{ 1 };
		uint32_t source_version{ 1 };
		uint32_t committed_delta_version{ 1 };
		uint32_t block_delta_content_version{ 1 };

		float minimum_sample_m{ 0.0f };
		float maximum_sample_m{ 0.0f };

		uint64_t last_referenced_frame{ 0 };
		uint64_t retire_after_frame{ 0 };

		bool is_fallback{ false };  // Slot 0 per LOD: permanent, non-evictable.

		TerrainRenderBlockKey key;
		TerrainSamplePatchKey sample_patch;

		[[nodiscard]] TerrainGpuPageIdentity get_identity() const noexcept {
			return TerrainGpuPageIdentity{
				key,
				sample_patch,
				page_contract_version,
				payload_kind,
				terrain_version,
				source_version,
				block_delta_content_version
			};
		}
	};
	using TerrainGpuSlot = GpuPageSlot;

	struct LODLevelData {
		uint8_t index{ 0 };
		uint32_t active_candidates{ 0 };
		uint32_t visible_count{ 0 };

		RenderID multimesh_rid;
		RenderID instance_rid;
		RenderID material_rid;
		RenderID texture_array_rid;

		std::array<GpuPageSlot, 128> slots{};

		uint32_t last_candidate_count{ 0 };
		uint32_t last_visible_count{ 0 };
		uint32_t submitted_visible_count{ 0 };
		uint8_t submitted_buffer_index{ 0 };

		int32_t last_hole_dx{ 0 };
		int32_t last_hole_dz{ 0 };
		bool has_last_hole{ false };

		std::array<TerrainRenderBlockKey, BlockClipmapProfile::MAX_CANDIDATES> diagnostic_candidate_keys{};
		std::array<VisibleInstanceDiagnostic, BlockClipmapProfile::MAX_CANDIDATES> pending_visible_diagnostics{};
		std::array<VisibleInstanceDiagnostic, BlockClipmapProfile::MAX_CANDIDATES> submitted_visible_diagnostics{};
	};

	std::array<LODLevelData, BlockClipmapProfile::MAX_LEVELS> levels{};

	RenderID scenario_rid;
	bool is_initialized{ false };
	bool has_v5_chart_global_lease_{ false };

	// Retained for change-detection diagnostics only.
	Multinet::TerrainRenderSourceSnapshot last_snapshot;
	BCCMSourceExpectation last_expectation;

	struct CameraMotionTracker {
		double last_u_m{ 0.0 };
		double last_v_m{ 0.0 };
		double dir_u{ 0.0 };
		double dir_v{ 0.0 };
		bool valid{ false };
	};
	CameraMotionTracker motion_tracker{};

	BCCMCameraState last_cam_state;

	// CPU upload buffer — allocated at full size during initialize; never resized on hot path.
	std::vector<float> local_upload_buffer;
	std::array<std::vector<float>, BlockClipmapProfile::MAX_LEVELS> cached_buffers{};

	uint64_t render_frame_id{ 0 };

	struct FrameAdmissionLimits {
		uint32_t max_page_commits{ 24 };
		uint32_t max_upload_bytes{ 256 * 1024 };
		uint32_t max_cross_face_page_commits{ 2 };
		uint32_t max_source_requests{ 64 };
	};

	FrameAdmissionLimits limits;

	struct StagingImage {
#ifdef MULTINET_TEST
		std::vector<uint8_t> data;
#else
		godot::PackedByteArray data;
		godot::Ref<godot::Image> img;
#endif
	};
	std::vector<StagingImage> staging_images;

	static constexpr uint32_t RING_BUFFER_SIZE = 3;
	uint32_t frame_index{ 0 };

	// Per-LOD per-ring-slot float buffers.
	// Full size = MAX_CANDIDATES * 16; never resized after initialization.
	std::array<std::array<std::vector<float>, RING_BUFFER_SIZE>, BlockClipmapProfile::MAX_LEVELS> multimesh_ring_buffers;

#ifndef MULTINET_TEST
	// Pre-allocated GPU upload arrays — avoid per-frame heap churn.
	std::array<std::array<godot::PackedFloat32Array, RING_BUFFER_SIZE>, BlockClipmapProfile::MAX_LEVELS> multimesh_gpu_buffers;
#endif

	// Pre-allocated fixed-capacity candidate array (zero hot-path heap allocation).
	std::array<TerrainClipmapBlockState, BlockClipmapProfile::MAX_CANDIDATES> candidate_blocks{};

	FrameTerrainSubmissionPlan last_submission_plan{};
	StreamingDiagnosticsSnapshot last_streaming_diagnostics{};
	bool has_published_shaped_plan{ false };

	TerrainRenderBlockKey current_lod7_snapped_center_key{};
	bool has_lod7_snapped_center{ false };
	bool ring_transaction_pending{ false };
	uint64_t ring_transaction_start_frame{ 0 };
	uint32_t next_ring_terminal_keys_required{ 0 };
	uint32_t next_ring_terminal_keys_resident{ 0 };

	Multinet::TerrainRecipe cached_recipe_{};
	Multinet::WorldScaleManifest cached_scale_{};
	bool last_bound_chp_gpu_effective_{ false };
	float last_bound_chp_camera_altitude_m_{ 0.0f };

#ifndef MULTINET_TEST

	godot::RID frozen_frustum_mesh_rid_{};
	godot::RID frozen_frustum_instance_rid_{};
	godot::Transform3D frozen_frustum_transform_{};
#endif
	bool has_frozen_frustum_{ false };


	// Per-frame cut and snap tracking
	uint64_t cut_render_update_serial_{ 0 };
	double last_cut_cam_u_{ 0.0 };
	double last_cut_cam_v_{ 0.0 };
	bool has_last_cut_cam_pos_{ false };

	std::array<int64_t, BlockClipmapProfile::MAX_LEVELS> last_cut_center_bx_{};
	std::array<int64_t, BlockClipmapProfile::MAX_LEVELS> last_cut_center_bv_{};
	std::array<bool, BlockClipmapProfile::MAX_LEVELS> has_last_cut_center_{};

	bool high_speed_cut_diagnostics_enabled_{ false };
	FrameCutDiagnostics last_cut_diagnostics_{};
	uint64_t total_skipped_snap_events_{ 0 };
	uint64_t total_multimesh_buffer_rewrites_{ 0 };
	size_t cumulative_instance_bytes_uploaded_{ 0 };

	godot::RID create_master_block_mesh(bool p_diamond_triangulation);

	// Uploads a zero scalar page to GPU layer 0 for a given LOD.
	// Called during initialization only.
#ifndef MULTINET_TEST
	void upload_zero_scalar_layer(godot::RenderingServer* rs, uint8_t lod);
#endif

public:
	BlockClipmapRenderer();
	~BlockClipmapRenderer();

	[[nodiscard]] BlockPlacement build_block_placement(
		const TerrainRenderBlockKey& canonical_key,
		const Multinet::SurfaceFrame& active_frame,
		const Multinet::WorldScaleManifest& manifest,
		const Multinet::TerrainFallbackBounds& bounds,
		const Multinet::TerrainCommittedDeltaSnapshot* delta_snapshot = nullptr,
		const Multinet::TerrainFallbackBounds* stale_bounds = nullptr
	) const;

	[[nodiscard]] BlockPlacement build_presentation_block_placement(
		const TerrainPresentationBlockKey& presentation_key,
		const TerrainRenderBlockKey& canonical_key,
		double observer_presentation_x_m,
		double observer_presentation_z_m,
		const Multinet::TerrainFallbackBounds& bounds,
		const Multinet::TerrainCommittedDeltaSnapshot* delta_snapshot = nullptr,
		const Multinet::TerrainFallbackBounds* stale_bounds = nullptr
	) const;

	// Unified initialization — returns false and releases all resources on any failure.
	[[nodiscard]] bool initialize(
		godot::RenderingServer* rendering_server,
		godot::RID scenario,
		const Multinet::WorldScaleManifest& scale,
		const Multinet::TerrainRecipeIdentity& expected_recipe,
		const Multinet::TerrainFallbackBounds& expected_fallback_bounds
	);
	[[nodiscard]] bool initialize(
		godot::RenderingServer* rendering_server,
		godot::RID scenario,
		const Multinet::WorldDomainManifest& domain,
		const Multinet::TerrainRecipeIdentity& expected_recipe,
		const Multinet::TerrainFallbackBounds& expected_fallback_bounds
	);

	// Select before initialize(). Runtime buffers are sized from this profile,
	// so a live change is a rebuild rather than an in-place mutation.
	[[nodiscard]] bool set_candidate_grid_radius(int32_t radius) noexcept;
	[[nodiscard]] int32_t get_candidate_grid_radius() const noexcept { return profile.candidate_grid_radius; }
	[[nodiscard]] double get_effective_coverage_extent_m() const noexcept;
	[[nodiscard]] double get_effective_coverage_corner_radius_m() const noexcept;

	void cleanup();

	TerrainUpdateResult compute_update(
		const godot::Vector3& cam_pos,
		const FrustumPlanes& frustum,
		const Multinet::WorldScaleManifest& scale,
		const BCCMCameraState& cam_state,
		const BCCMSourceExpectation& expectation,
		Multinet::TerrainRenderSource* terrain_source = nullptr,
		const multinet::rendering::chp::CurvedHorizonView* chp_view = nullptr,
		double delta_seconds = 0.0
	);

	void update_with_view(
		const godot::Vector3& p_camera_world_position,
		const FrustumPlanes& p_frustum,
		const Multinet::WorldScaleManifest& scale,
		const BCCMCameraState& cam_state,
		const BCCMSourceExpectation& expectation,
		Multinet::TerrainRenderSource* terrain_source = nullptr,
		const multinet::rendering::chp::CurvedHorizonView* chp_view = nullptr,
		double delta_seconds = 0.0
	);

	// Freeze keeps the submitted set and page state fixed, but camera
	// movement applies a continuous camera delta and CHP altitude refresh
	// so the live camera moves freely around the frozen cut in world space.
	void update_frozen_view_presentation_delta(
		const godot::Vector3& p_camera_delta,
		const multinet::rendering::chp::CurvedHorizonView* chp_view = nullptr,
		double delta_seconds = 0.0
	) noexcept;

	void update_frozen_view_presentation(
		const godot::Vector3& p_camera_world_position,
		const multinet::rendering::chp::CurvedHorizonView* chp_view = nullptr
	) noexcept;

	// Freeze-only Phase-B2 morph compensation. The submitted instance transforms
	// continue moving camera-relative, while this offset keeps morph-distance
	// evaluation anchored to the observer position at which the cut was frozen.
	void set_parent_morph_view_offset(const godot::Vector2& p_offset_m) noexcept;

	void rebase_frozen_presentation(const godot::Vector3& p_camera_world_position) noexcept {
		update_frozen_view_presentation(p_camera_world_position, nullptr);
	}

	void set_frozen_frustum_visualization(
		const godot::Transform3D& camera_transform,
		float fov_deg,
		float near_m,
		float far_m,
		float aspect
	) noexcept;

	void clear_frozen_frustum_visualization() noexcept;
	bool has_frozen_frustum_visualization() const noexcept { return has_frozen_frustum_; }

	void update(
		godot::Camera3D* p_camera,
		const Multinet::WorldScaleManifest& scale,
		const BCCMCameraState& cam_state,
		const BCCMSourceExpectation& expectation,
		Multinet::TerrainRenderSource* terrain_source = nullptr,
		const multinet::rendering::chp::CurvedHorizonView* chp_view = nullptr,
		double delta_seconds = 0.0
	);

	const BlockClipmapProfile& get_profile() const noexcept { return profile; }

	uint32_t get_candidate_count(uint8_t lod) const {
		return lod < profile.level_count ? levels[lod].last_candidate_count : 0;
	}
	uint32_t get_visible_count(uint8_t lod) const {
		return lod < profile.level_count ? levels[lod].last_visible_count : 0;
	}
	uint8_t get_effective_level_count() const noexcept { return profile.level_count; }
	uint32_t get_submitted_streams() const { return is_initialized ? profile.level_count : 0; }
	bool initialized() const { return is_initialized; }

	TerrainSourceMode get_source_mode() const noexcept { return source_mode; }
	void set_source_mode(TerrainSourceMode mode) noexcept {
		if (mode == source_mode) return;
		source_mode = mode;
		for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
			for (uint32_t j = 1; j < 128; ++j) {
				auto& slot = levels[lod].slots[j];
				if (slot.state == TerrainGpuPageState::Resident || slot.state == TerrainGpuPageState::UploadPending) {
					slot.state = TerrainGpuPageState::Retiring;
					slot.retire_after_frame = render_frame_id + RING_BUFFER_SIZE;
				}
			}
		}
	}

	bool get_analytic_debug_prewarm_pages() const noexcept { return analytic_debug_prewarm_pages; }
	void set_analytic_debug_prewarm_pages(bool prewarm) noexcept { analytic_debug_prewarm_pages = prewarm; }

	bool get_face_colors_enabled() const noexcept { return face_colors_enabled; }
	void set_face_colors_enabled(bool enabled) noexcept;

	bool get_diamond_triangulation_enabled() const noexcept { return diamond_triangulation_enabled; }
	void set_diamond_triangulation_enabled(bool enabled) noexcept;

	int get_chp_debug_reconstruction_mode() const noexcept { return chp_debug_reconstruction_mode; }
	void set_chp_debug_reconstruction_mode(int mode) noexcept { chp_debug_reconstruction_mode = mode; }

	bool get_chp_debug_negative_height_color() const noexcept { return chp_debug_negative_height_color; }
	void set_chp_debug_negative_height_color(bool enabled) noexcept { chp_debug_negative_height_color = enabled; }

	bool get_chp_debug_negative_height_exaggeration() const noexcept { return chp_debug_negative_height_exaggeration; }
	void set_chp_debug_negative_height_exaggeration(bool enabled) noexcept { chp_debug_negative_height_exaggeration = enabled; }

	int get_bccm_debug_visual_mode() const noexcept { return bccm_debug_visual_mode; }
	void set_bccm_debug_visual_mode(int mode) noexcept;

	bool get_high_speed_cut_diagnostics_enabled() const noexcept { return high_speed_cut_diagnostics_enabled_; }
	void set_high_speed_cut_diagnostics_enabled(bool enabled) noexcept {
		if (high_speed_cut_diagnostics_enabled_ != enabled) {
			high_speed_cut_diagnostics_enabled_ = enabled;
			if (enabled) {
				for (auto& lvl : levels) {
					lvl.last_candidate_count = 0;
				}
			}
		}
	}


	const FrameTerrainSubmissionPlan& get_last_submission_plan() const { return last_submission_plan; }
	const StreamingDiagnosticsSnapshot& get_last_streaming_diagnostics() const { return last_streaming_diagnostics; }

#ifdef DEBUG_ENABLED
	DebugBlockReplacementState get_debug_block_state(const TerrainRenderBlockKey& key) const;
#endif

	// Production-logic test initializer.
	// Establishes fallback slot zero, all slot states, buffer sizes, frame counters
	// and expected snapshot identity — without creating Godot RIDs.
	void get_diagnostic_snapshot(RendererDiagnosticSnapshot& out_snap) const;
	DetailedRendererDiagnostics get_detailed_diagnostics() const;
	[[nodiscard]] const FrameCutDiagnostics& get_cut_diagnostics() const noexcept { return last_cut_diagnostics_; }
	[[nodiscard]] uint64_t get_total_skipped_snap_events() const noexcept { return total_skipped_snap_events_; }
	[[nodiscard]] uint64_t get_total_multimesh_buffer_rewrites() const noexcept { return total_multimesh_buffer_rewrites_; }
	[[nodiscard]] size_t get_cumulative_instance_bytes_uploaded() const noexcept { return cumulative_instance_bytes_uploaded_; }


	void bind_material_uniforms(
		const Multinet::TerrainRecipe& recipe,
		const Multinet::WorldScaleManifest& scale
	);
	void bind_material_uniforms(
		const Multinet::TerrainRecipe& recipe,
		const Multinet::WorldDomainManifest& domain
	);

	void initialize_cpu_state_for_test(
		const Multinet::WorldScaleManifest& scale,
		const Multinet::TerrainRecipeIdentity& recipe_identity,
		const Multinet::TerrainFallbackBounds& expected_fallback_bounds
	);

#ifdef DEBUG_ENABLED
	void test_set_profile_levels(uint8_t count) { profile.level_count = count; }
	void test_set_profile_radius(int32_t r) { profile.candidate_grid_radius = r; }
	void test_set_profile_hole_radius(int32_t r) { profile.inner_hole_radius = r; }
	void test_set_profile_block_size(float sz) { profile.lod0_block_size = sz; profile.finest_spacing = sz / 16.0f; }
	void test_set_max_source_requests(uint32_t n) { limits.max_source_requests = n; }
	void test_set_max_page_commits(uint32_t n) {
		limits.max_page_commits = n;
		staging_images.resize(n);
		for (uint32_t i = 0; i < n; ++i) {
#ifdef MULTINET_TEST
			staging_images[i].data.resize(19 * 19 * 4, 0);
#else
			staging_images[i].data.resize(19 * 19 * 4);
#endif
		}
	}
	void test_set_max_cross_face_commits(uint32_t n) { limits.max_cross_face_page_commits = n; }
	void test_clear_slot_residency(uint8_t lod, uint32_t slot_idx) {
		if (lod < profile.level_count && slot_idx < 128) {
			levels[lod].slots[slot_idx].state = TerrainGpuPageState::Free;
			levels[lod].slots[slot_idx].key = TerrainRenderBlockKey{};
		}
	}

	void test_finalize_uploads(const TerrainUpdateResult& res) {
		for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
			levels[lod].last_visible_count = res.lods[lod].visible_count;
			levels[lod].submitted_visible_count = res.lods[lod].visible_count;
			std::copy_n(
				levels[lod].pending_visible_diagnostics.begin(),
				res.lods[lod].visible_count,
				levels[lod].submitted_visible_diagnostics.begin()
			);
		}
		for (uint32_t i = 0; i < res.texture_upload_count; ++i) {
			const auto& u = res.texture_uploads[i];
			if (u.lod < profile.level_count && u.gpu_layer < 128) {
				levels[u.lod].slots[u.gpu_layer].state = TerrainGpuPageState::Resident;
			}
		}
	}
#endif

#ifdef MULTINET_TEST
	const TerrainGpuSlot& inspect_slot(uint8_t lod, uint32_t layer) const { return levels[lod].slots[layer]; }
	const SubmittedInstance& inspect_instance(uint8_t lod, uint32_t index) const { return last_submission_plan.lods[lod].instances[index]; }
	uint32_t count_same_block_slots(uint8_t lod, const TerrainRenderBlockKey& key) const {
		uint32_t cnt = 0;
		if (lod < profile.level_count) {
			for (uint32_t j = 1; j < 128; ++j) {
				if ((levels[lod].slots[j].state == TerrainGpuPageState::Resident || levels[lod].slots[j].state == TerrainGpuPageState::UploadPending) &&
				    levels[lod].slots[j].key == key) {
					++cnt;
				}
			}
		}
		return cnt;
	}
	void finalize_upload(uint8_t lod, uint32_t layer) {
		if (lod < profile.level_count && layer < 128) {
			levels[lod].slots[layer].state = TerrainGpuPageState::Resident;
		}
	}
	void advance_renderer_frame_without_new_demand() {
		++render_frame_id;
		for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
			for (uint32_t j = 1; j < 128; ++j) {
				auto& slot = levels[lod].slots[j];
				if (slot.state == TerrainGpuPageState::Retiring && render_frame_id >= slot.retire_after_frame) {
					slot.state = TerrainGpuPageState::Free;
					slot.key = TerrainRenderBlockKey{};
				}
			}
		}
	}
	uint64_t inspect_retirement_frame(uint8_t lod, uint32_t layer) const { return levels[lod].slots[layer].retire_after_frame; }
	float inspect_bound_chp_camera_altitude() const noexcept { return last_bound_chp_camera_altitude_m_; }
	bool inspect_bound_chp_gpu_effective() const noexcept { return last_bound_chp_gpu_effective_; }
	const godot::Vector3& inspect_active_view_world_position() const noexcept { return active_view_world_position; }
	const std::vector<float>& inspect_ring_buffer(uint8_t lod, uint8_t ring_idx) const noexcept { return multimesh_ring_buffers[lod][ring_idx]; }
#endif
};

} // namespace multinet::rendering

#endif // MULTINET_BLOCK_CLIPMAP_RENDERER_H

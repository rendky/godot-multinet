#ifndef MULTINET_BLOCK_CLIPMAP_RENDERER_H
#define MULTINET_BLOCK_CLIPMAP_RENDERER_H

#include "multinet/rendering/terrain/block_clipmap/block_clipmap_profile.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_state.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_culling.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_shader.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_ids.h"
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

#include <array>

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
};

struct TerrainUpdateResult {
	struct TextureUpload {
		uint8_t lod{ 0 };
		uint32_t gpu_layer{ 0 };
		uint32_t staging_index{ 0 };
		// Full identity for two-phase finalization inside update().
		TerrainRenderBlockKey canonical_key;
		uint64_t cpu_page_handle{ 0 };
		uint32_t cpu_page_generation{ 0 };
		uint32_t terrain_version{ 1 };
		uint32_t source_version{ 1 };
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

enum class TerrainGpuPageState : uint8_t { Free, UploadPending, Resident, Retiring };

struct VisibleInstanceDiagnostic {
	TerrainRenderBlockKey key{};
	uint32_t gpu_layer{ 0 };
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
	};

	struct LODSnapshot {
		uint32_t candidate_count{ 0 };
		uint32_t visible_count{ 0 };
		std::array<SlotSnapshot, 128> slots{};
		std::vector<TerrainRenderBlockKey> candidate_keys;
		std::vector<VisibleInstanceDiagnostic> submitted_visible_diagnostics;
		size_t ring_buffer_float_count{ 0 };
	};

	std::array<LODSnapshot, BlockClipmapProfile::MAX_LEVELS> lods{};
};

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
	Multinet::TerrainFallbackBounds fallback_bounds{};

	RenderID master_mesh_rid;
	BCCMShaderData shader_data;

	struct GpuPageSlot {
		TerrainGpuPageState state{ TerrainGpuPageState::Free };
		uint64_t cpu_page_handle{ 0 };
		uint32_t cpu_page_generation{ 0 };
		uint32_t gpu_layer{ 0 };

		uint32_t terrain_version{ 1 };
		uint32_t source_version{ 1 };

		uint64_t last_referenced_frame{ 0 };
		uint64_t retire_after_frame{ 0 };

		bool is_fallback{ false };  // Slot 0 per LOD: permanent, non-evictable.

		TerrainRenderBlockKey key;
	};

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

		std::array<TerrainRenderBlockKey, BlockClipmapProfile::MAX_CANDIDATES> diagnostic_candidate_keys{};
		std::array<VisibleInstanceDiagnostic, BlockClipmapProfile::MAX_CANDIDATES> pending_visible_diagnostics{};
		std::array<VisibleInstanceDiagnostic, BlockClipmapProfile::MAX_CANDIDATES> submitted_visible_diagnostics{};
	};

	std::array<LODLevelData, BlockClipmapProfile::MAX_LEVELS> levels{};

	RenderID scenario_rid;
	bool is_initialized{ false };

	// Retained for change-detection diagnostics only.
	Multinet::TerrainRenderSourceSnapshot last_snapshot;

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

	godot::RID create_master_block_mesh();

	// Uploads a zero-height fallback page to GPU layer 0 for a given LOD.
	// Called during initialization only.
#ifndef MULTINET_TEST
	void upload_fallback_layer(godot::RenderingServer* rs, uint8_t lod,
	                           float fallback_height, float min_elev, float max_elev);
#endif

public:
	BlockClipmapRenderer();
	~BlockClipmapRenderer();

	// Unified initialization — returns false and releases all resources on any failure.
	[[nodiscard]] bool initialize(
		godot::RenderingServer* rendering_server,
		godot::RID scenario,
		const Multinet::WorldScaleManifest& scale,
		const Multinet::TerrainRecipeIdentity& expected_recipe,
		const Multinet::TerrainFallbackBounds& expected_fallback_bounds
	);

	void cleanup();

	TerrainUpdateResult compute_update(
		const godot::Vector3& cam_pos,
		const FrustumPlanes& frustum,
		const Multinet::WorldScaleManifest& scale,
		const BCCMCameraState& cam_state,
		const BCCMSourceExpectation& expectation,
		Multinet::TerrainRenderSource* terrain_source = nullptr
	);

	void update_with_view(
		const godot::Vector3& p_camera_world_position,
		const FrustumPlanes& p_frustum,
		const Multinet::WorldScaleManifest& scale,
		const BCCMCameraState& cam_state,
		const BCCMSourceExpectation& expectation,
		Multinet::TerrainRenderSource* terrain_source = nullptr
	);

	void update(
		godot::Camera3D* p_camera,
		const Multinet::WorldScaleManifest& scale,
		const BCCMCameraState& cam_state,
		const BCCMSourceExpectation& expectation,
		Multinet::TerrainRenderSource* terrain_source = nullptr
	);

	uint32_t get_candidate_count(uint8_t lod) const {
		return lod < profile.level_count ? levels[lod].last_candidate_count : 0;
	}
	uint32_t get_visible_count(uint8_t lod) const {
		return lod < profile.level_count ? levels[lod].last_visible_count : 0;
	}
	uint32_t get_submitted_streams() const { return is_initialized ? profile.level_count : 0; }
	bool initialized() const { return is_initialized; }

	// Production-logic test initializer.
	// Establishes fallback slot zero, all slot states, buffer sizes, frame counters
	// and expected snapshot identity — without creating Godot RIDs.
	void get_diagnostic_snapshot(RendererDiagnosticSnapshot& out_snap) const;

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
	std::vector<StagingImage>& test_get_staging_images() { return staging_images; }
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
};

} // namespace multinet::rendering

#endif // MULTINET_BLOCK_CLIPMAP_RENDERER_H

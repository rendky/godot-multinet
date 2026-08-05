#include "multinet/rendering/terrain/block_clipmap/block_clipmap_renderer.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include "multinet/core/spatial/world_manifests.h"
#include "multinet/core/spatial/surface_coordinate_conversion.h"
#include "multinet/world/terrain/outputs/rendering/terrain_render_source.h"
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <iostream>
#include <cstring>
#include <algorithm>

namespace multinet::rendering {

// ─── Geometry ────────────────────────────────────────────────────────────────

BlockClipmapRenderer::BlockClipmapRenderer() {
}

BlockClipmapRenderer::~BlockClipmapRenderer() {
	cleanup();
}

godot::RID BlockClipmapRenderer::create_master_block_mesh() {
	godot::PackedVector3Array vertices;
	godot::PackedVector3Array normals;
	godot::PackedVector2Array uvs;
	godot::PackedInt32Array indices;

	const uint32_t verts_across = BlockClipmapProfile::VERTS_PER_EDGE;
	const uint32_t quads_across = BlockClipmapProfile::QUADS_PER_EDGE;

	vertices.resize(BlockClipmapProfile::TOTAL_VERTS);
	normals.resize(BlockClipmapProfile::TOTAL_VERTS);
	uvs.resize(BlockClipmapProfile::TOTAL_VERTS);

	for (uint32_t z = 0; z <= quads_across; ++z) {
		for (uint32_t x = 0; x <= quads_across; ++x) {
			uint32_t idx = z * verts_across + x;
			vertices.set(idx, godot::Vector3(static_cast<float>(x), 0.0f, static_cast<float>(z)));
			normals.set(idx, godot::Vector3(0.0f, 1.0f, 0.0f));
			uvs.set(idx, godot::Vector2(static_cast<float>(x) / 16.0f, static_cast<float>(z) / 16.0f));
		}
	}

	for (uint32_t z = 0; z < quads_across; ++z) {
		for (uint32_t x = 0; x < quads_across; ++x) {
			int v00 = z * verts_across + x;
			int v10 = z * verts_across + (x + 1);
			int v01 = (z + 1) * verts_across + x;
			int v11 = (z + 1) * verts_across + (x + 1);

			indices.push_back(v00);
			indices.push_back(v10);
			indices.push_back(v01);
			indices.push_back(v10);
			indices.push_back(v11);
			indices.push_back(v01);
		}
	}

	godot::Array arrays;
	arrays.resize(godot::RenderingServer::ARRAY_MAX);
	arrays[godot::RenderingServer::ARRAY_VERTEX] = vertices;
	arrays[godot::RenderingServer::ARRAY_NORMAL] = normals;
	arrays[godot::RenderingServer::ARRAY_TEX_UV] = uvs;
	arrays[godot::RenderingServer::ARRAY_INDEX] = indices;

	godot::RenderingServer *rs = godot::RenderingServer::get_singleton();
	godot::RID mesh = rs->mesh_create();
	rs->mesh_add_surface_from_arrays(mesh, godot::RenderingServer::PRIMITIVE_TRIANGLES, arrays);
	return mesh;
}

// ─── Initialization ───────────────────────────────────────────────────────────

#ifndef MULTINET_TEST
void BlockClipmapRenderer::upload_fallback_layer(
	godot::RenderingServer* rs,
	uint8_t lod,
	float fallback_height,
	float min_elev,
	float max_elev
) {
	static constexpr size_t PAGE_FLOATS = 19 * 19;
	static constexpr size_t PAGE_BYTES = PAGE_FLOATS * 4;

	std::array<float, PAGE_FLOATS> fallback_data;
	fallback_data.fill(fallback_height);

	godot::PackedByteArray raw;
	raw.resize(PAGE_BYTES);
	std::memcpy(raw.ptrw(), fallback_data.data(), PAGE_BYTES);

	auto img = godot::Image::create_from_data(19, 19, false, godot::Image::FORMAT_RF, raw);
	rs->texture_2d_update(levels[lod].texture_array_rid, img, 0);
}
#endif

bool BlockClipmapRenderer::initialize(
	godot::RenderingServer* rendering_server,
	godot::RID scenario,
	const Multinet::WorldScaleManifest& scale,
	const Multinet::TerrainRecipeIdentity& expected_recipe,
	const Multinet::TerrainFallbackBounds& expected_fallback_bounds
) {
	if (is_initialized) return true;

#ifndef MULTINET_TEST
	if (!rendering_server) return false;
	if (!scenario.is_valid()) return false;
	if (!scale.is_valid()) return false;

	scenario_rid = scenario;
	godot::RenderingServer* rs = rendering_server;

	// --- CPU buffers ----------------------------------------------------------
	constexpr size_t FULL_FLOAT_COUNT = BlockClipmapProfile::MAX_CANDIDATES * 16;
	local_upload_buffer.resize(FULL_FLOAT_COUNT, 0.0f);
	for (int i = 0; i < BlockClipmapProfile::MAX_LEVELS; ++i) {
		cached_buffers[i].resize(FULL_FLOAT_COUNT, 0.0f);
		for (int j = 0; j < static_cast<int>(RING_BUFFER_SIZE); ++j) {
			multimesh_ring_buffers[i][j].resize(FULL_FLOAT_COUNT, 0.0f);
			multimesh_gpu_buffers[i][j].resize(FULL_FLOAT_COUNT);
		}
	}

	// --- Staging images -------------------------------------------------------
	staging_images.resize(limits.max_page_commits);
	for (uint32_t i = 0; i < limits.max_page_commits; ++i) {
		staging_images[i].data.resize(19 * 19 * 4);
		// Zero the staging buffer.
		uint8_t* raw = staging_images[i].data.ptrw();
		std::memset(raw, 0, 19 * 19 * 4);
		staging_images[i].img = godot::Image::create_from_data(
			19, 19, false, godot::Image::FORMAT_RF, staging_images[i].data
		);
	}

	// --- Master mesh + shader -------------------------------------------------
	master_mesh_rid = create_master_block_mesh();
	if (!master_mesh_rid.is_valid()) {
		cleanup();
		return false;
	}

	shader_data = create_bccm_shader_material();
	if (!shader_data.shader_rid.is_valid() || !shader_data.material_rid.is_valid()) {
		std::cerr << "[Multinet BCCM] ERROR: Shader or material RID invalid after creation." << std::endl;
		cleanup();
		return false;
	}

	rs->mesh_surface_set_material(master_mesh_rid, 0, shader_data.material_rid);

	std::cout << "[Multinet BCCM] INITIALIZED: "
			  << "Shader RID Valid: " << shader_data.shader_rid.is_valid() << ", "
			  << "Material RID Valid: " << shader_data.material_rid.is_valid() << ", "
			  << "Master Mesh Material Valid: true" << std::endl;

	// Fallback page content: clamp(0, minimum_height, maximum_height).
	const float min_elev = expected_fallback_bounds.minimum_height;
	const float max_elev = expected_fallback_bounds.maximum_height;
	const float fallback_height = std::clamp(0.0f, min_elev, max_elev);

	// --- Per-LOD resources ----------------------------------------------------
	godot::Ref<godot::Image> empty_img = godot::Image::create(19, 19, false, godot::Image::FORMAT_RF);
	godot::TypedArray<godot::Image> initial_layers;
	initial_layers.resize(128);
	for (int i = 0; i < 128; ++i) {
		initial_layers[i] = empty_img;
	}

	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		LODLevelData& level = levels[lod];
		level.index = lod;

		// MultiMesh: 3D transform + custom data = 16 floats/instance.
		level.multimesh_rid = rs->multimesh_create();
		rs->multimesh_allocate_data(
			level.multimesh_rid,
			BlockClipmapProfile::MAX_CANDIDATES,
			godot::RenderingServer::MULTIMESH_TRANSFORM_3D,
			false,
			true  // use_custom_data
		);
		rs->multimesh_set_mesh(level.multimesh_rid, master_mesh_rid);
		rs->multimesh_set_custom_aabb(
			level.multimesh_rid,
			godot::AABB(godot::Vector3(0, 0, 0), godot::Vector3(0.01f, 0.01f, 0.01f))
		);

		level.instance_rid = rs->instance_create();
		if (!level.instance_rid.is_valid()) { cleanup(); return false; }
		rs->instance_set_base(level.instance_rid, level.multimesh_rid);
		rs->instance_set_scenario(level.instance_rid, scenario_rid);
		rs->instance_set_ignore_culling(level.instance_rid, true);
		rs->instance_geometry_set_cast_shadows_setting(
			level.instance_rid,
			godot::RenderingServer::SHADOW_CASTING_SETTING_OFF
		);
		rs->instance_set_custom_aabb(
			level.instance_rid,
			godot::AABB(godot::Vector3(0, 0, 0), godot::Vector3(0.01f, 0.01f, 0.01f))
		);

		// 128-layer height texture array.
		level.texture_array_rid = rs->texture_2d_layered_create(
			initial_layers,
			godot::RenderingServer::TEXTURE_LAYERED_2D_ARRAY
		);
		if (!level.texture_array_rid.is_valid()) { cleanup(); return false; }

		level.material_rid = rs->material_create();
		if (!level.material_rid.is_valid()) { cleanup(); return false; }
		rs->material_set_shader(level.material_rid, shader_data.shader_rid);
		rs->material_set_param(level.material_rid, "height_pages", level.texture_array_rid);
		rs->instance_geometry_set_material_override(level.instance_rid, level.material_rid);

		// Layer 0: permanent fallback.
		upload_fallback_layer(rs, lod, fallback_height, min_elev, max_elev);
		level.slots[0].state = TerrainGpuPageState::Resident;
		level.slots[0].gpu_layer = 0;
		level.slots[0].is_fallback = true;
		level.slots[0].last_referenced_frame = 0;
		level.slots[0].retire_after_frame = UINT64_MAX;

		// Slots 1..127: Free.
		for (uint32_t i = 1; i < 128; ++i) {
			level.slots[i].state = TerrainGpuPageState::Free;
			level.slots[i].gpu_layer = i;
			level.slots[i].is_fallback = false;
		}
	}

	// Store expected identity for snapshot validation.
	last_snapshot.recipe_identity = expected_recipe;
	last_snapshot.world_manifest_hash = scale.manifest_hash;
	last_snapshot.topology_version = scale.topology_version;
	last_snapshot.projection_version = scale.projection_version;
	last_snapshot.fallback_bounds = expected_fallback_bounds;

	is_initialized = true;
	return true;
#else
	// Test path: delegate to production CPU initializer.
	initialize_cpu_state_for_test(scale, expected_recipe, expected_fallback_bounds);
	return true;
#endif
}

void BlockClipmapRenderer::initialize_cpu_state_for_test(
	const Multinet::WorldScaleManifest& scale,
	const Multinet::TerrainRecipeIdentity& recipe_identity,
	const Multinet::TerrainFallbackBounds& expected_fallback_bounds
) {
	constexpr size_t FULL_FLOAT_COUNT = BlockClipmapProfile::MAX_CANDIDATES * 16;

	local_upload_buffer.resize(FULL_FLOAT_COUNT, 0.0f);
	for (int i = 0; i < BlockClipmapProfile::MAX_LEVELS; ++i) {
		cached_buffers[i].resize(FULL_FLOAT_COUNT, 0.0f);
		for (int j = 0; j < static_cast<int>(RING_BUFFER_SIZE); ++j) {
			multimesh_ring_buffers[i][j].resize(FULL_FLOAT_COUNT, 0.0f);
		}
	}

	staging_images.resize(limits.max_page_commits);
	for (uint32_t i = 0; i < limits.max_page_commits; ++i) {
		// Resize the staging data buffer — std::vector (test) vs PackedByteArray (prod).
#ifdef MULTINET_TEST
		staging_images[i].data.resize(19 * 19 * 4, 0);
		// img left null in test — no Godot Image.
#else
		staging_images[i].data.resize(19 * 19 * 4);
		staging_images[i].img = godot::Image::create(19, 19, false, godot::Image::FORMAT_RF);
#endif
	}

	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		LODLevelData& level = levels[lod];
		level.index = lod;

		// Layer 0: permanent fallback.
		level.slots[0].state = TerrainGpuPageState::Resident;
		level.slots[0].gpu_layer = 0;
		level.slots[0].is_fallback = true;
		level.slots[0].last_referenced_frame = 0;
		level.slots[0].retire_after_frame = UINT64_MAX;

		// Slots 1..127: Free.
		for (uint32_t i = 1; i < 128; ++i) {
			level.slots[i].state = TerrainGpuPageState::Free;
			level.slots[i].gpu_layer = i;
			level.slots[i].is_fallback = false;
		}
	}

	// Expected snapshot identity for validation.
	last_snapshot.recipe_identity = recipe_identity;
	last_snapshot.world_manifest_hash = scale.manifest_hash;
	last_snapshot.topology_version = scale.topology_version;
	last_snapshot.projection_version = scale.projection_version;
	last_snapshot.fallback_bounds = expected_fallback_bounds;
	last_snapshot.terrain_version = 1;
	last_snapshot.source_version = 1;

	render_frame_id = 0;
	frame_index = 0;
	is_initialized = true;
}

void BlockClipmapRenderer::cleanup() {
#ifndef MULTINET_TEST
	godot::RenderingServer *rs = godot::RenderingServer::get_singleton();

	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		LODLevelData& level = levels[lod];
		if (level.instance_rid.is_valid()) {
			rs->free_rid(level.instance_rid);
			level.instance_rid = godot::RID();
		}
		if (level.multimesh_rid.is_valid()) {
			rs->free_rid(level.multimesh_rid);
			level.multimesh_rid = godot::RID();
		}
		if (level.material_rid.is_valid()) {
			rs->free_rid(level.material_rid);
			level.material_rid = godot::RID();
		}
		if (level.texture_array_rid.is_valid()) {
			rs->free_rid(level.texture_array_rid);
			level.texture_array_rid = godot::RID();
		}
	}

	if (master_mesh_rid.is_valid()) {
		rs->free_rid(master_mesh_rid);
		master_mesh_rid = godot::RID();
	}

	if (shader_data.material_rid.is_valid()) {
		rs->free_rid(shader_data.material_rid);
		shader_data.material_rid = godot::RID();
	}
	if (shader_data.shader_rid.is_valid()) {
		rs->free_rid(shader_data.shader_rid);
		shader_data.shader_rid = godot::RID();
	}
#endif
	is_initialized = false;
}

// ─── compute_update (CPU — unconditional) ─────────────────────────────────────

TerrainUpdateResult BlockClipmapRenderer::compute_update(
	const godot::Vector3& cam_pos,
	const FrustumPlanes& frustum,
	const Multinet::WorldScaleManifest& scale,
	const BCCMCameraState& cam_state,
	const BCCMSourceExpectation& expectation,
	Multinet::TerrainRenderSource* terrain_source
) {
	TerrainUpdateResult result;
	if (!is_initialized) return result;
	if (!cam_state.is_visible || cam_state.frame_epoch == 0 ||
	    cam_state.active_frame.topology_version != scale.topology_version ||
	    cam_state.active_frame.projection_version != scale.projection_version ||
	    cam_state.active_frame.origin.topology_version != scale.topology_version ||
	    cam_state.active_frame.origin.projection_version != scale.projection_version ||
	    cam_state.canonical_position.topology_version != scale.topology_version ||
	    cam_state.canonical_position.projection_version != scale.projection_version) {
		return result;
	}

	render_frame_id++;

	// Phase 0 — validate source snapshot against external authority.
	bool snapshot_valid = false;
	if (terrain_source) {
		Multinet::TerrainRenderSourceSnapshot snap = terrain_source->get_snapshot();
		// Retain for diagnostics.
		last_snapshot = snap;

		snapshot_valid =
			snap.world_manifest_hash == expectation.world_manifest_hash &&
			snap.topology_version == expectation.topology_version &&
			snap.projection_version == expectation.projection_version &&
			snap.recipe_identity == expectation.recipe_identity &&
			snap.terrain_version == expectation.terrain_version &&
			snap.source_version == expectation.source_version;

		static int snap_print_count = 0;
		if (!snapshot_valid && ++snap_print_count <= 3) {
			std::cerr << "[RENDERER] snapshot_valid=FALSE"
			          << " mh:" << snap.world_manifest_hash << "==" << expectation.world_manifest_hash << "?" << (snap.world_manifest_hash == expectation.world_manifest_hash)
			          << " tv:" << snap.topology_version << "==" << expectation.topology_version << "?" << (snap.topology_version == expectation.topology_version)
			          << " pv:" << snap.projection_version << "==" << expectation.projection_version << "?" << (snap.projection_version == expectation.projection_version)
			          << " ri:" << (snap.recipe_identity == expectation.recipe_identity)
			          << " terv:" << snap.terrain_version << "==" << expectation.terrain_version << "?" << (snap.terrain_version == expectation.terrain_version)
			          << " srcv:" << snap.source_version << "==" << expectation.source_version << "?" << (snap.source_version == expectation.source_version)
			          << std::endl;
			std::cerr << "[RENDERER] snap.recipe_identity: rv=" << snap.recipe_identity.recipe_version
			          << " sv=" << snap.recipe_identity.schema_version
			          << " tv=" << snap.recipe_identity.topology_version
			          << " pv=" << snap.recipe_identity.projection_version
			          << " algo=" << snap.recipe_identity.deterministic_algorithm_id
			          << " seed=" << snap.recipe_identity.world_seed
			          << " rh=" << snap.recipe_identity.recipe_hash
			          << " mh=" << snap.recipe_identity.manifest_hash
			          << std::endl;
			std::cerr << "[RENDERER] exp.recipe_identity: rv=" << expectation.recipe_identity.recipe_version
			          << " sv=" << expectation.recipe_identity.schema_version
			          << " tv=" << expectation.recipe_identity.topology_version
			          << " pv=" << expectation.recipe_identity.projection_version
			          << " algo=" << expectation.recipe_identity.deterministic_algorithm_id
			          << " seed=" << expectation.recipe_identity.world_seed
			          << " rh=" << expectation.recipe_identity.recipe_hash
			          << " mh=" << expectation.recipe_identity.manifest_hash
			          << std::endl;
		}
	}

	// Phase 0b — on snapshot mismatch, push all dynamic Resident slots toward Retirement.
	if (!snapshot_valid) {
		for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
			for (uint32_t i = 1; i < 128; ++i) {
				auto& slot = levels[lod].slots[i];
				if (slot.state == TerrainGpuPageState::Resident) {
					slot.state = TerrainGpuPageState::Retiring;
					slot.retire_after_frame = render_frame_id + RING_BUFFER_SIZE;
				}
			}
		}
	}

	// Phase 1 — retire eligible Retiring slots.
	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		for (uint32_t i = 1; i < 128; ++i) {
			auto& slot = levels[lod].slots[i];
			if (slot.state == TerrainGpuPageState::Retiring &&
			    render_frame_id >= slot.retire_after_frame) {
				slot.state = TerrainGpuPageState::Free;
			}
		}
	}

	// Phase 2 — canonical camera projection.
	Multinet::FramePosition64 cam_frame_pos;
	if (!Multinet::try_surface_to_frame(cam_state.canonical_position, cam_state.active_frame, scale, cam_frame_pos)) {
		return result;
	}

	auto dot = [](const Multinet::FramePosition64& p, const Multinet::Vec3d& axis) {
		return p.x * axis.x + p.y * axis.y + p.z * axis.z;
	};
	double active_cam_u = cam_state.active_frame.origin.u_m + dot(cam_frame_pos, cam_state.active_frame.tangent_basis.u_axis);
	double active_cam_v = cam_state.active_frame.origin.v_m + dot(cam_frame_pos, cam_state.active_frame.tangent_basis.v_axis);

	// Per-frame counters.
	uint32_t total_commits = 0;
	size_t total_upload_bytes = 0;
	uint32_t total_source_requests = 0;
	uint32_t total_cross_face_commits = 0;
	uint32_t total_evictions = 0;

	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		const double block_size = profile.get_lod_block_size(lod);
		const double spacing = profile.get_lod_spacing(lod);

		const double snap_size = (lod + 1 < profile.level_count)
			? profile.get_lod_block_size(lod + 1)
			: block_size;

		int64_t center_bx = static_cast<int64_t>(std::floor(
			(std::floor(active_cam_u / snap_size) * snap_size) / block_size
		));
		int64_t center_bv = static_cast<int64_t>(std::floor(
			(std::floor(active_cam_v / snap_size) * snap_size) / block_size
		));

		int32_t hole_dx = 0;
		int32_t hole_dz = 0;
		if (lod > 0) {
			int64_t prev_bx = static_cast<int64_t>(std::floor(
				(std::floor(active_cam_u / block_size) * block_size) / block_size
			));
			int64_t prev_bv = static_cast<int64_t>(std::floor(
				(std::floor(active_cam_v / block_size) * block_size) / block_size
			));
			hole_dx = static_cast<int32_t>(prev_bx - center_bx);
			hole_dz = static_cast<int32_t>(prev_bv - center_bv);
		}

		int32_t r = profile.candidate_grid_radius;
		int32_t hole_r = profile.inner_hole_radius;
		uint32_t cand_idx = 0;

		// Reset per-LOD diagnostic collections for current frame.
		// Arrays don't need clear(); we rely on last_candidate_count and last_visible_count.

		// Phase 3 — enumerate candidates (center-outward by distance squared).
		struct CandOffset {
			int32_t du, dv;
			int32_t dist_sq;
		};
		std::array<CandOffset, BlockClipmapProfile::MAX_GRID_OFFSETS> offsets{};
		uint32_t offset_count = 0;

		for (int32_t dv = -r; dv < r; ++dv) {
			for (int32_t du = -r; du < r; ++du) {
				if (lod > 0) {
					int32_t hu = du - hole_dx;
					int32_t hv = dv - hole_dz;
					if (hu >= -hole_r && hu < hole_r && hv >= -hole_r && hv < hole_r) continue;
				}
				if (offset_count < BlockClipmapProfile::MAX_GRID_OFFSETS) {
					offsets[offset_count] = { du, dv, du * du + dv * dv };
					++offset_count;
				}
			}
		}

		std::sort(offsets.begin(), offsets.begin() + offset_count, [](const CandOffset& a, const CandOffset& b) {
			return a.dist_sq < b.dist_sq;
		});

		for (uint32_t off_i = 0; off_i < offset_count; ++off_i) {
			int32_t du = offsets[off_i].du;
			int32_t dv = offsets[off_i].dv;

				if (cand_idx >= BlockClipmapProfile::MAX_CANDIDATES) break;

				int64_t bx = center_bx + du;
				int64_t bv_coord = center_bv + dv;

				Multinet::SurfaceAddress addr;
				addr.face = cam_state.active_frame.origin.face;
				addr.u_mm = static_cast<int64_t>(std::round((bx * block_size + block_size * 0.5) * 1000.0));
				addr.v_mm = static_cast<int64_t>(std::round((bv_coord * block_size + block_size * 0.5) * 1000.0));
				addr.topology_version = cam_state.active_frame.topology_version;
				addr.projection_version = cam_state.active_frame.projection_version;

				Multinet::SurfaceAddress canon = Multinet::canonicalize_surface_address(addr, scale);
				if (static_cast<uint8_t>(canon.face) == 255) continue;

				int32_t canon_bx = static_cast<int32_t>(std::floor((canon.u_mm * 0.001) / block_size));
				int32_t canon_bv = static_cast<int32_t>(std::floor((canon.v_mm * 0.001) / block_size));

				TerrainRenderBlockKey key{ canon.face, canon_bx, canon_bv, lod, ORDINARY_BCCM_V1_PROFILE, 0 };

				bool duplicate = false;
				for (uint32_t i = 0; i < cand_idx; ++i) {
					if (candidate_blocks[i].placement.key == key) {
						duplicate = true;
						break;
					}
				}
				if (duplicate) continue;

				Multinet::SurfacePosition64 min_corner;
				min_corner.face = canon.face;
				min_corner.u_m = canon_bx * block_size;
				min_corner.v_m = canon_bv * block_size;
				min_corner.altitude_m = 0.0;
				min_corner.topology_version = canon.topology_version;
				min_corner.projection_version = canon.projection_version;

				Multinet::FramePosition64 f_min;
				if (!Multinet::try_surface_to_frame(min_corner, cam_state.active_frame, scale, f_min)) continue;

				Multinet::SurfacePosition64 pu = min_corner; pu.u_m += 1.0;
				Multinet::SurfacePosition64 pv = min_corner; pv.v_m += 1.0;
				Multinet::SurfacePosition64 ph = min_corner; ph.altitude_m += 1.0;

				Multinet::FramePosition64 fu, fv_fp, fh;
				if (!Multinet::try_surface_to_frame(pu, cam_state.active_frame, scale, fu)) continue;
				if (!Multinet::try_surface_to_frame(pv, cam_state.active_frame, scale, fv_fp)) continue;
				if (!Multinet::try_surface_to_frame(ph, cam_state.active_frame, scale, fh)) continue;

				godot::Vector3 vx(fu.x - f_min.x, fu.y - f_min.y, fu.z - f_min.z);
				godot::Vector3 vz(fv_fp.x - f_min.x, fv_fp.y - f_min.y, fv_fp.z - f_min.z);
				godot::Vector3 vy(fh.x - f_min.x, fh.y - f_min.y, fh.z - f_min.z);
				godot::Basis block_basis;
				block_basis.set_column(0, vx);
				block_basis.set_column(1, vy);
				block_basis.set_column(2, vz);

				TerrainClipmapBlockState& state = candidate_blocks[cand_idx];
				state.placement.key = key;
				state.placement.block_to_active_frame = block_basis;
				state.placement.local_origin = godot::Vector3(f_min.x, f_min.y, f_min.z);
				state.placement.frame_epoch = cam_state.frame_epoch;

				uint8_t edge_mask = 0;
				if (du == r - 1) edge_mask |= 1;
				if (du == -r)    edge_mask |= 2;
				if (dv == r - 1) edge_mask |= 4;
				if (dv == -r)    edge_mask |= 8;
				state.edge_mask = edge_mask;
				state.gpu_layer = 0; // Default: fallback layer.

				levels[lod].diagnostic_candidate_keys[cand_idx] = key;
				cand_idx++;
			}

		levels[lod].last_candidate_count = cand_idx;

		// Phase 4 — residency scan, admission, retirement.
		for (uint32_t i = 0; i < cand_idx; ++i) {
			TerrainClipmapBlockState& state = candidate_blocks[i];

			bool cross_face = (state.placement.key.face != cam_state.active_frame.origin.face);

			// Conservative fallback bounds derived from Terrain authority
			Multinet::TerrainFallbackBounds fb = snapshot_valid ? last_snapshot.fallback_bounds : fallback_bounds;

			float lower = fb.minimum_height - fb.residual_bound - fb.morph_allowance;
			float upper = fb.maximum_height + fb.residual_bound + fb.morph_allowance;

			uint32_t found_slot = 0; // 0 = fallback
			bool already_resident = false;

			if (snapshot_valid && terrain_source) {
				// Resident scan BEFORE requesting new record! Look up by key.
				for (uint32_t j = 1; j < 128; ++j) {
					auto& slot = levels[lod].slots[j];
					if ((slot.state == TerrainGpuPageState::Resident || slot.state == TerrainGpuPageState::UploadPending) &&
					    slot.key == state.placement.key) {
						found_slot = j;
						already_resident = true;
						break;
					}
				}

				if (already_resident) {
					levels[lod].slots[found_slot].last_referenced_frame = render_frame_id;
					static uint64_t last_res_frame = 0;
					static uint32_t res_hit = 0;
					if (render_frame_id != last_res_frame) {
						if (last_res_frame > 0 && last_res_frame <= 10) {
							std::cerr << "[RENDERER-RESIDENT] Frame " << last_res_frame << " already_resident=" << res_hit << std::endl;
						}
						last_res_frame = render_frame_id;
						res_hit = 0;
					}
					++res_hit;
				} else {
					// Request a page without artificially limiting iterations
					++total_source_requests;

					Multinet::TerrainSourceRecord record =
						terrain_source->get_or_request_record(state.placement.key);

						// Per-frame state tracking (static counters reset each frame via render_frame_id)
						static uint64_t last_diag_frame = 0;
						static uint32_t pending_count = 0, ready_count = 0, missing_count = 0, invalid_count = 0;
						if (render_frame_id != last_diag_frame) {
							if (last_diag_frame > 0 && last_diag_frame <= 5) {
								std::cerr << "[RENDERER-P4] Frame " << last_diag_frame
										  << " records: pending=" << pending_count
										  << " ready=" << ready_count
										  << " missing=" << missing_count
										  << " invalid=" << invalid_count
										  << std::endl;
							}
							last_diag_frame = render_frame_id;
							pending_count = ready_count = missing_count = invalid_count = 0;
						}
						if (record.state == Multinet::TerrainSourceState::Pending) ++pending_count;
						else if (record.state == Multinet::TerrainSourceState::Ready) ++ready_count;
						else if (record.state == Multinet::TerrainSourceState::Missing) ++missing_count;
						else if (record.state == Multinet::TerrainSourceState::Invalid) ++invalid_count;

						if (record.state == Multinet::TerrainSourceState::Ready &&
						    total_commits < limits.max_page_commits &&
						    total_upload_bytes + 1444 <= limits.max_upload_bytes)
						{
						static uint64_t last_ready_diag_frame = 0;
						static uint32_t ready_no_slot = 0, ready_no_read = 0, ready_uploaded = 0;
						if (render_frame_id != last_ready_diag_frame) {
							if (last_ready_diag_frame > 0 && last_ready_diag_frame <= 5) {
								std::cerr << "[RENDERER-P4-READY] Frame " << last_ready_diag_frame
								          << " no_slot=" << ready_no_slot
								          << " no_read=" << ready_no_read
								          << " uploaded=" << ready_uploaded
								          << std::endl;
							}
							last_ready_diag_frame = render_frame_id;
							ready_no_slot = ready_no_read = ready_uploaded = 0;
						}
							if (!cross_face || total_cross_face_commits < limits.max_cross_face_page_commits) {
								uint32_t candidate_slot = 0;

								// Search full dynamic range 1..127 for Free slots
								for (uint32_t j = 1; j < 128; ++j) {
									if (levels[lod].slots[j].state == TerrainGpuPageState::Free) {
										candidate_slot = j;
										break;
									}
								}

								// Search full dynamic range 1..127 for expired Retiring slots
								if (candidate_slot == 0) {
									for (uint32_t j = 1; j < 128; ++j) {
										auto& slot = levels[lod].slots[j];
										if (slot.state == TerrainGpuPageState::Retiring &&
										    render_frame_id >= slot.retire_after_frame) {
											candidate_slot = j;
											break;
										}
									}
								}

								// Search full dynamic range 1..127 for oldest Resident slot from a PREVIOUS frame to evict
								if (candidate_slot == 0) {
									uint64_t oldest = UINT64_MAX;
									for (uint32_t j = 1; j < 128; ++j) {
										auto& slot = levels[lod].slots[j];
										if (!slot.is_fallback &&
										    slot.state == TerrainGpuPageState::Resident &&
										    slot.last_referenced_frame < render_frame_id &&
										    slot.last_referenced_frame < oldest) {
											oldest = slot.last_referenced_frame;
											candidate_slot = j;
										}
									}

									if (candidate_slot != 0) {
										auto& evict = levels[lod].slots[candidate_slot];
										evict.state = TerrainGpuPageState::Retiring;
										evict.retire_after_frame = render_frame_id + RING_BUFFER_SIZE;
										candidate_slot = 0;
									} else {
										static int no_slot_diag = 0;
										if (++no_slot_diag <= 5) std::cerr << "[RENDERER-P4-ERR] candidate_slot=0, no evictable found" << std::endl;
									}
								}

								if (candidate_slot != 0 &&
								    result.texture_upload_count < TerrainUpdateResult::MAX_TEXTURE_UPLOADS &&
								    total_commits < limits.max_page_commits) {
									Multinet::TerrainHeightPage page;
									bool read_ok = terrain_source->try_read_page(
										record.cpu_page_handle,
										record.cpu_page_generation,
										page);
									static int read_print = 0;
									if (++read_print <= 5) {
										std::cerr << "[RENDERER-P4-READ] candidate_slot=" << candidate_slot
										          << " read_ok=" << read_ok
										          << " handle=" << record.cpu_page_handle
										          << " gen=" << record.cpu_page_generation
										          << std::endl;
									}
									if (read_ok)
									{
										uint32_t staging_idx = total_commits;

#ifdef MULTINET_TEST
										uint8_t* dest = staging_images[staging_idx].data.data();
#else
										uint8_t* dest = staging_images[staging_idx].data.ptrw();
#endif
										std::memcpy(dest, page.heights.data(), 19 * 19 * 4);
#ifndef MULTINET_TEST
										staging_images[staging_idx].img->set_data(
											19, 19, false, godot::Image::FORMAT_RF, staging_images[staging_idx].data
										);
#endif

										auto& slot = levels[lod].slots[candidate_slot];
										slot.state = TerrainGpuPageState::UploadPending;
										slot.cpu_page_handle = record.cpu_page_handle;
										slot.cpu_page_generation = record.cpu_page_generation;
										slot.terrain_version = record.terrain_version;
										slot.source_version = record.source_version;
										slot.key = state.placement.key;
										slot.last_referenced_frame = render_frame_id;

										TerrainUpdateResult::TextureUpload& upload =
											result.texture_uploads[result.texture_upload_count++];
										upload.lod = lod;
										upload.gpu_layer = candidate_slot;
										upload.staging_index = staging_idx;
										upload.canonical_key = state.placement.key;
										upload.cpu_page_handle = record.cpu_page_handle;
										upload.cpu_page_generation = record.cpu_page_generation;
										upload.terrain_version = record.terrain_version;
										upload.source_version = record.source_version;

										++total_commits;
										total_upload_bytes += 1444;
										if (cross_face) ++total_cross_face_commits;

										found_slot = candidate_slot;

										lower = record.min_height - record.residual_bound - record.morph_allowance;
										upper = record.max_height + record.residual_bound + record.morph_allowance;
									}
								}
							}
						}
					}
				}

			// Assign layer and local AABB unconditionally
			state.gpu_layer = found_slot;
			state.placement.local_aabb = godot::AABB(
				godot::Vector3(0.0f, lower, 0.0f),
				godot::Vector3(static_cast<float>(block_size), std::max(0.001f, upper - lower), static_cast<float>(block_size))
			);

			// Frustum cull unconditionally (NO continue!)
			godot::AABB global_aabb = godot::Transform3D(
				state.placement.block_to_active_frame, godot::Vector3(0, 0, 0)
			).xform(state.placement.local_aabb);
			global_aabb.position += state.placement.local_origin;
			state.is_visible = frustum.intersects_aabb(global_aabb);
		}

		// Phase 5 — build instance buffer.
		uint32_t vis_count = 0;
		float* ptr = local_upload_buffer.data();

		for (uint32_t i = 0; i < cand_idx; ++i) {
			const TerrainClipmapBlockState& state = candidate_blocks[i];
			if (!state.is_visible) continue;

			levels[lod].pending_visible_diagnostics[vis_count] = {
				state.placement.key,
				state.gpu_layer
			};

			godot::Basis basis = state.placement.block_to_active_frame.scaled(
				godot::Vector3(static_cast<float>(spacing), 1.0f, static_cast<float>(spacing))
			);

			size_t base = vis_count * 16;
			ptr[base +  0] = basis.rows[0].x;
			ptr[base +  1] = basis.rows[1].x;
			ptr[base +  2] = basis.rows[2].x;
			ptr[base +  3] = state.placement.local_origin.x;

			ptr[base +  4] = basis.rows[0].y;
			ptr[base +  5] = basis.rows[1].y;
			ptr[base +  6] = basis.rows[2].y;
			ptr[base +  7] = state.placement.local_origin.y;

			ptr[base +  8] = basis.rows[0].z;
			ptr[base +  9] = basis.rows[1].z;
			ptr[base + 10] = basis.rows[2].z;
			ptr[base + 11] = state.placement.local_origin.z;

			ptr[base + 12] = static_cast<float>(state.edge_mask);
			ptr[base + 13] = static_cast<float>(state.gpu_layer);
			ptr[base + 14] = 0.0f;
			ptr[base + 15] = 0.0f;

			++vis_count;
		}

		constexpr size_t FLOATS_PER_INSTANCE = 16;
		std::memcpy(
			multimesh_ring_buffers[lod][frame_index].data(),
			ptr,
			vis_count * FLOATS_PER_INSTANCE * sizeof(float)
		);

		bool buffer_changed = false;
		if (vis_count > 0) {
			const uint8_t* new_ptr = reinterpret_cast<const uint8_t*>(local_upload_buffer.data());
			uint8_t* cached_ptr = reinterpret_cast<uint8_t*>(cached_buffers[lod].data());

			size_t byte_count = vis_count * FLOATS_PER_INSTANCE * sizeof(float);
			if (std::memcmp(new_ptr, cached_ptr, byte_count) != 0) {
				std::memcpy(cached_ptr, new_ptr, byte_count);
				buffer_changed = true;
			}
		}

		if (vis_count != levels[lod].last_visible_count) {
			buffer_changed = true;
		}

		result.lods[lod].visible_count = vis_count;
		result.lods[lod].buffer_changed = buffer_changed;
	}

	if (terrain_source) {
		multinet::rendering::TerrainRenderBlockKey cam_key;
		cam_key.face = cam_state.active_frame.origin.face;
		cam_key.block_u = static_cast<int32_t>(std::floor(active_cam_u / profile.get_lod_block_size(0)));
		cam_key.block_v = static_cast<int32_t>(std::floor(active_cam_v / profile.get_lod_block_size(0)));
		cam_key.lod = 0;
		terrain_source->commit_pending_requests(cam_key);
	}

	return result;
}

// ─── update (Production GPU submission — two-phase) ──────────────────────────

#ifndef MULTINET_TEST
void BlockClipmapRenderer::update_with_view(
	const godot::Vector3& p_camera_world_position,
	const FrustumPlanes& p_frustum,
	const Multinet::WorldScaleManifest& scale,
	const BCCMCameraState& cam_state,
	const BCCMSourceExpectation& expectation,
	Multinet::TerrainRenderSource* terrain_source
) {
	if (!is_initialized) return;
	if (!cam_state.is_visible) return;

	TerrainUpdateResult result = compute_update(p_camera_world_position, p_frustum, scale, cam_state, expectation, terrain_source);

	godot::RenderingServer* rs = godot::RenderingServer::get_singleton();

	// Phase 6 — two-phase GPU finalization: re-validate each upload before committing.
	auto patch_rejected = [&](uint8_t failed_lod, uint32_t failed_layer, const char* reason) {
		static int r = 0; if (++r <= 10) std::cerr << "[P6-REJECT] lod=" << (int)failed_lod << " layer=" << failed_layer << " reason=" << reason << std::endl;
		auto& lvl = levels[failed_lod];
		lvl.slots[failed_layer].state = TerrainGpuPageState::Free;
		for (uint32_t i = 0; i < result.lods[failed_lod].visible_count; ++i) {
			if (lvl.pending_visible_diagnostics[i].gpu_layer == failed_layer) {
				lvl.pending_visible_diagnostics[i].gpu_layer = 0;
				size_t base = i * 16;
				multimesh_ring_buffers[failed_lod][frame_index][base + 13] = 0.0f;
			}
		}
	};

	for (uint32_t u = 0; u < result.texture_upload_count; ++u) {
		const TerrainUpdateResult::TextureUpload& upload = result.texture_uploads[u];
		uint8_t lod = upload.lod;
		uint32_t gpu_layer = upload.gpu_layer;

		// Validate RID and layer range.
		if (!levels[lod].texture_array_rid.is_valid()) {
			patch_rejected(lod, gpu_layer, "RID_INVALID");
			continue;
		}
		if (gpu_layer == 0 || gpu_layer >= 128) {
			patch_rejected(lod, gpu_layer, "LAYER_OUT_OF_RANGE");
			continue;
		}
		if (upload.staging_index >= staging_images.size()) {
			patch_rejected(lod, gpu_layer, "STAGING_INDEX_OUT_OF_RANGE");
			continue;
		}
		if (!staging_images[upload.staging_index].img.is_valid()) {
			patch_rejected(lod, gpu_layer, "IMG_INVALID");
			continue;
		}

		// Verify the slot hasn't been clobbered between compute and update.
		auto& slot = levels[lod].slots[gpu_layer];
		if (slot.state != TerrainGpuPageState::UploadPending ||
		    slot.cpu_page_handle != upload.cpu_page_handle ||
		    slot.cpu_page_generation != upload.cpu_page_generation) {
			patch_rejected(lod, gpu_layer, "SLOT_CLOBBERED");
			continue;
		}

		// Issue the GPU upload. (No success return from Godot; validated before call.)
		rs->texture_2d_update(
			levels[lod].texture_array_rid,
			staging_images[upload.staging_index].img,
			static_cast<int64_t>(gpu_layer)
		);

		// Finalize: UploadPending → Resident.
		slot.state = TerrainGpuPageState::Resident;
		slot.key = upload.canonical_key;
		static int ok = 0; if (++ok <= 10) std::cerr << "[P6-OK] lod=" << (int)lod << " layer=" << gpu_layer << std::endl;
	}

	// Phase 7 — MultiMesh submission (full-size buffer).
	constexpr size_t FLOATS_PER_INSTANCE = 16;
	constexpr size_t FULL_FLOAT_COUNT = BlockClipmapProfile::MAX_CANDIDATES * FLOATS_PER_INSTANCE;

	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		LODLevelData& level = levels[lod];

		uint32_t vis_count = result.lods[lod].visible_count;
		bool changed = result.lods[lod].buffer_changed;

		if (!changed && level.last_visible_count == vis_count) continue;

		// Copy populated CPU data into the pre-allocated GPU array.
		float* ring_ptr = multimesh_ring_buffers[lod][frame_index].data();
		auto& gpu_buf = multimesh_gpu_buffers[lod][frame_index];

		// gpu_buf was pre-allocated at FULL_FLOAT_COUNT during initialize.
		std::memcpy(gpu_buf.ptrw(), ring_ptr, FULL_FLOAT_COUNT * sizeof(float));

		rs->multimesh_set_buffer(level.multimesh_rid, gpu_buf);
		rs->multimesh_set_visible_instances(level.multimesh_rid, vis_count);
		
		level.last_visible_count = vis_count;
		level.submitted_visible_count = vis_count;

		std::copy_n(
			level.pending_visible_diagnostics.begin(),
			vis_count,
			level.submitted_visible_diagnostics.begin()
		);
	}

	frame_index = (frame_index + 1) % RING_BUFFER_SIZE;
}

void BlockClipmapRenderer::update(
	godot::Camera3D* p_camera,
	const Multinet::WorldScaleManifest& scale,
	const BCCMCameraState& cam_state,
	const BCCMSourceExpectation& expectation,
	Multinet::TerrainRenderSource* terrain_source
) {
	if (!p_camera) return;
	godot::Vector3 cam_pos = p_camera->get_global_position();
	FrustumPlanes frustum = FrustumPlanes::extract_from_camera(p_camera);
	update_with_view(cam_pos, frustum, scale, cam_state, expectation, terrain_source);
}
#else
void BlockClipmapRenderer::update_with_view(
	const godot::Vector3& p_camera_world_position,
	const FrustumPlanes& p_frustum,
	const Multinet::WorldScaleManifest& scale,
	const BCCMCameraState& cam_state,
	const BCCMSourceExpectation& expectation,
	Multinet::TerrainRenderSource* terrain_source
) {}

void BlockClipmapRenderer::update(
	godot::Camera3D* p_camera,
	const Multinet::WorldScaleManifest& scale,
	const BCCMCameraState& cam_state,
	const BCCMSourceExpectation& expectation,
	Multinet::TerrainRenderSource* terrain_source
) {}
#endif

void BlockClipmapRenderer::get_diagnostic_snapshot(RendererDiagnosticSnapshot& out_snap) const {
	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		auto& lod_snap = out_snap.lods[lod];
		const auto& level = levels[lod];

		lod_snap.candidate_count = level.last_candidate_count;
		lod_snap.visible_count = level.last_visible_count;
		lod_snap.ring_buffer_float_count = BlockClipmapProfile::MAX_CANDIDATES * 16;

		for (uint32_t i = 0; i < 128; ++i) {
			const auto& slot = level.slots[i];
			auto& s_snap = lod_snap.slots[i];
			s_snap.state = slot.state;
			s_snap.cpu_page_handle = slot.cpu_page_handle;
			s_snap.cpu_page_generation = slot.cpu_page_generation;
			s_snap.gpu_layer = slot.gpu_layer;
			s_snap.last_referenced_frame = slot.last_referenced_frame;
			s_snap.retire_after_frame = slot.retire_after_frame;
			s_snap.is_fallback = slot.is_fallback;
			s_snap.key = slot.key;
		}

		lod_snap.candidate_keys.assign(level.diagnostic_candidate_keys.begin(), level.diagnostic_candidate_keys.begin() + level.last_candidate_count);
		lod_snap.submitted_visible_diagnostics.assign(level.submitted_visible_diagnostics.begin(), level.submitted_visible_diagnostics.begin() + level.submitted_visible_count);
	}
}

} // namespace multinet::rendering

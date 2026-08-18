#include "multinet/rendering/terrain/block_clipmap/block_clipmap_renderer.h"
#include "multinet/rendering/terrain/block_clipmap/terrain_sample_patch.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include "multinet/core/spatial/world_manifests.h"
#include "multinet/core/spatial/surface_coordinate_conversion.h"
#include "multinet/core/spatial/world_domain.h"
#include "multinet/world/terrain/outputs/rendering/terrain_render_source.h"
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <cassert>
#include <limits>

namespace multinet::rendering {

namespace {

#ifndef MULTINET_TEST
constexpr const char* V5_ROOT_PRESENTATION_GLOBAL = "multinet_bccm_v5_root_presentation_m";
constexpr const char* V5_ROOT_DIRECTION_GLOBAL = "multinet_bccm_v5_root_direction";
constexpr const char* V5_X_TANGENT_GLOBAL = "multinet_bccm_v5_presentation_x_tangent";
constexpr const char* V5_Z_TANGENT_GLOBAL = "multinet_bccm_v5_presentation_z_tangent";
uint32_t v5_chart_global_owner_count = 0;

void acquire_v5_chart_globals(godot::RenderingServer* rs) {
	if (!rs) return;
	if (v5_chart_global_owner_count++ != 0) return;
	rs->global_shader_parameter_add(V5_ROOT_PRESENTATION_GLOBAL,
		godot::RenderingServer::GLOBAL_VAR_TYPE_VEC2, godot::Vector2());
	rs->global_shader_parameter_add(V5_ROOT_DIRECTION_GLOBAL,
		godot::RenderingServer::GLOBAL_VAR_TYPE_VEC3, godot::Vector3(1.0f, 0.0f, 0.0f));
	rs->global_shader_parameter_add(V5_X_TANGENT_GLOBAL,
		godot::RenderingServer::GLOBAL_VAR_TYPE_VEC3, godot::Vector3());
	rs->global_shader_parameter_add(V5_Z_TANGENT_GLOBAL,
		godot::RenderingServer::GLOBAL_VAR_TYPE_VEC3, godot::Vector3());
}

void release_v5_chart_globals(godot::RenderingServer* rs) {
	if (v5_chart_global_owner_count == 0) return;
	if (--v5_chart_global_owner_count != 0 || !rs) return;
	rs->global_shader_parameter_remove(V5_ROOT_PRESENTATION_GLOBAL);
	rs->global_shader_parameter_remove(V5_ROOT_DIRECTION_GLOBAL);
	rs->global_shader_parameter_remove(V5_X_TANGENT_GLOBAL);
	rs->global_shader_parameter_remove(V5_Z_TANGENT_GLOBAL);
}

void set_v5_chart_globals(
	godot::RenderingServer* rs,
	double root_presentation_x,
	double root_presentation_z,
	const godot::Vector3& root_direction,
	const godot::Vector3& presentation_x_tangent,
	const godot::Vector3& presentation_z_tangent
) {
	if (!rs) return;
	rs->global_shader_parameter_set(V5_ROOT_PRESENTATION_GLOBAL,
		godot::Vector2(static_cast<float>(root_presentation_x), static_cast<float>(root_presentation_z)));
	rs->global_shader_parameter_set(V5_ROOT_DIRECTION_GLOBAL, root_direction);
	rs->global_shader_parameter_set(V5_X_TANGENT_GLOBAL, presentation_x_tangent);
	rs->global_shader_parameter_set(V5_Z_TANGENT_GLOBAL, presentation_z_tangent);
}
#endif

struct TerrainRootLatticeAnchors {
	int32_t cell[8][3]{};
	float fraction[8][3]{};
};

inline TerrainRootLatticeAnchors compute_root_anchors(
	const Multinet::FramePosition64& root_direction,
	double radius_m,
	const Multinet::TerrainRecipe& recipe
) {
	TerrainRootLatticeAnchors anchors{};
	const double px = root_direction.x * radius_m;
	const double py = root_direction.y * radius_m;
	const double pz = root_direction.z * radius_m;

	double freq = recipe.legacy_signals.continental_frequency;
	const uint8_t octaves = std::min<uint8_t>(recipe.legacy_signals.octave_count, 8);

	for (uint8_t oct = 0; oct < octaves; ++oct) {
		double sx = px * freq;
		double sy = py * freq;
		double sz = pz * freq;

		double fx = std::floor(sx);
		double fy = std::floor(sy);
		double fz = std::floor(sz);

		anchors.cell[oct][0] = static_cast<int32_t>(fx);
		anchors.cell[oct][1] = static_cast<int32_t>(fy);
		anchors.cell[oct][2] = static_cast<int32_t>(fz);

		anchors.fraction[oct][0] = static_cast<float>(sx - fx);
		anchors.fraction[oct][1] = static_cast<float>(sy - fy);
		anchors.fraction[oct][2] = static_cast<float>(sz - fz);

		freq *= static_cast<double>(recipe.legacy_signals.lacunarity);
	}
	return anchors;
}

bool try_make_canonical_block_key(
	Multinet::SurfaceFace face,
	int64_t block_u,
	int64_t block_v,
	uint8_t lod,
	const Multinet::WorldScaleManifest& manifest,
	TerrainRenderBlockKey& out_key
) noexcept {
	if (!manifest.is_valid() || !Multinet::is_valid_surface_face(face) || lod >= BlockClipmapProfile::MAX_LEVELS) return false;
	BlockClipmapProfile prof_inst;
	double block_size = prof_inst.get_lod_block_size(lod);
	if (!std::isfinite(block_size) || !(block_size > 0.0)) return false;
	Multinet::SurfaceAddress addr;
	addr.face = face;
	addr.u_mm = static_cast<int64_t>(std::round((block_u * block_size + block_size * 0.5) * 1000.0));
	addr.v_mm = static_cast<int64_t>(std::round((block_v * block_size + block_size * 0.5) * 1000.0));
	addr.topology_version = manifest.topology_version;
	addr.projection_version = manifest.projection_version;

	Multinet::SurfaceAddress canon = Multinet::canonicalize_surface_address(addr, manifest);
	if (!canon.is_valid()) return false;
	int32_t canon_u = static_cast<int32_t>(floor_div(static_cast<int64_t>(std::floor(canon.u_mm * 0.001)), static_cast<int64_t>(block_size)));
	int32_t canon_v = static_cast<int32_t>(floor_div(static_cast<int64_t>(std::floor(canon.v_mm * 0.001)), static_cast<int64_t>(block_size)));

	out_key = TerrainRenderBlockKey{ canon.face, canon_u, canon_v, lod, ORDINARY_BCCM_V1_PROFILE, 0 };
	return out_key.face != static_cast<Multinet::SurfaceFace>(255);
}

} // namespace

TerrainRenderBlockKey make_canonical_block_key(
	Multinet::SurfaceFace face,
	int64_t block_u,
	int64_t block_v,
	uint8_t lod,
	const Multinet::WorldScaleManifest& manifest
) {
	TerrainRenderBlockKey out{};
	if (!try_make_canonical_block_key(face, block_u, block_v, lod, manifest, out)) {
		out.face = static_cast<Multinet::SurfaceFace>(255);
	}
	return out;
}

bool make_domain_block_key(
	Multinet::SurfaceFace input_chart,
	int64_t block_u,
	int64_t block_v,
	uint8_t lod,
	const Multinet::WorldDomainManifest& domain,
	TerrainRenderBlockKey& out_key
) noexcept {
	if (!domain.is_valid() || lod >= BlockClipmapProfile::MAX_LEVELS) return false;
	BlockClipmapProfile profile;
	const double block_size = profile.get_lod_block_size(lod);
	if (domain.is_finite()) {
		if (!Multinet::finite_block_intersects_domain(block_u, block_v, block_size, domain)) return false;
		out_key = TerrainRenderBlockKey{ Multinet::SurfaceFace::PositiveX, static_cast<int32_t>(block_u), static_cast<int32_t>(block_v), lod, ORDINARY_BCCM_V1_PROFILE, 0 };
		return true;
	}
	if (lod >= Multinet::derive_domain_compatible_bccm_level_count(domain, profile.lod0_block_size, profile.level_count)) return false;
	return try_make_canonical_block_key(input_chart, block_u, block_v, lod, domain.closed_surface, out_key);
}

TerrainRenderBlockKey derive_canonical_parent_key(
	const TerrainRenderBlockKey& child_key,
	uint8_t target_parent_lod,
	const Multinet::WorldScaleManifest& manifest
) {
	if (target_parent_lod <= child_key.lod) return child_key;
	uint8_t lod_diff = target_parent_lod - child_key.lod;
	int64_t parent_u = child_key.block_u;
	int64_t parent_v = child_key.block_v;
	for (uint8_t d = 0; d < lod_diff; ++d) {
		parent_u = floor_div(parent_u, 2);
		parent_v = floor_div(parent_v, 2);
	}
	return make_canonical_block_key(child_key.face, parent_u, parent_v, target_parent_lod, manifest);
}

void enumerate_canonical_child_keys(
	const TerrainRenderBlockKey& parent_key,
	const Multinet::WorldScaleManifest& manifest,
	std::array<TerrainRenderBlockKey, 4>& out_children
) {
	uint8_t child_lod = parent_key.lod > 0 ? parent_key.lod - 1 : 0;
	int64_t base_u = parent_key.block_u * 2;
	int64_t base_v = parent_key.block_v * 2;

	out_children[0] = make_canonical_block_key(parent_key.face, base_u + 0, base_v + 0, child_lod, manifest);
	out_children[1] = make_canonical_block_key(parent_key.face, base_u + 1, base_v + 0, child_lod, manifest);
	out_children[2] = make_canonical_block_key(parent_key.face, base_u + 0, base_v + 1, child_lod, manifest);
	out_children[3] = make_canonical_block_key(parent_key.face, base_u + 1, base_v + 1, child_lod, manifest);
}

bool derive_domain_parent_key(
	const TerrainRenderBlockKey& child_key,
	uint8_t target_parent_lod,
	const Multinet::WorldDomainManifest& domain,
	TerrainRenderBlockKey& out_key
) noexcept {
	if (!domain.is_valid()) return false;
	if (!domain.is_finite()) {
		out_key = derive_canonical_parent_key(child_key, target_parent_lod, domain.closed_surface);
		return out_key.is_valid();
	}
	if (target_parent_lod <= child_key.lod) {
		out_key = child_key;
		return true;
	}
	int64_t u = child_key.block_u;
	int64_t v = child_key.block_v;
	for (uint8_t d = 0; d < target_parent_lod - child_key.lod; ++d) {
		u = floor_div(u, 2);
		v = floor_div(v, 2);
	}
	return make_domain_block_key(Multinet::SurfaceFace::PositiveX, u, v, target_parent_lod, domain, out_key);
}

uint32_t enumerate_domain_child_keys(
	const TerrainRenderBlockKey& parent_key,
	const Multinet::WorldDomainManifest& domain,
	std::array<TerrainRenderBlockKey, 4>& out_children
) noexcept {
	if (!domain.is_valid()) return 0;
	if (!domain.is_finite()) {
		enumerate_canonical_child_keys(parent_key, domain.closed_surface, out_children);
		uint32_t valid = 0;
		for (const auto& key : out_children) if (key.is_valid()) out_children[valid++] = key;
		return valid;
	}
	uint32_t count = 0;
	const uint8_t child_lod = parent_key.lod > 0 ? parent_key.lod - 1 : 0;
	const int64_t base_u = parent_key.block_u * 2;
	const int64_t base_v = parent_key.block_v * 2;
	for (int i = 0; i < 4; ++i) {
		const int64_t u = base_u + (i & 1);
		const int64_t v = base_v + ((i >> 1) & 1);
		TerrainRenderBlockKey key;
		if (make_domain_block_key(Multinet::SurfaceFace::PositiveX, u, v, child_lod, domain, key)) out_children[count++] = key;
	}
	return count;
}

// ─── Geometry ────────────────────────────────────────────────────────────────

BlockClipmapRenderer::BlockClipmapRenderer() {
}

BlockClipmapRenderer::~BlockClipmapRenderer() {
	cleanup();
}

bool BlockClipmapRenderer::set_candidate_grid_radius(int32_t radius) noexcept {
	if (is_initialized || radius < 1 || radius > BlockClipmapProfile::MAX_SUPPORTED_CANDIDATE_GRID_RADIUS) {
		return false;
	}
	profile.candidate_grid_radius = radius;
	profile.inner_hole_radius = std::min(profile.inner_hole_radius, radius - 1);
	return true;
}

double BlockClipmapRenderer::get_effective_coverage_extent_m() const noexcept {
	if (profile.level_count == 0) return 0.0;
	return static_cast<double>(profile.candidate_grid_radius) * 2.0 *
		static_cast<double>(profile.get_lod_block_size(static_cast<uint8_t>(profile.level_count - 1)));
}

double BlockClipmapRenderer::get_effective_coverage_corner_radius_m() const noexcept {
	if (profile.level_count == 0) return 0.0;
	const double terminal_block_m = static_cast<double>(
		profile.get_lod_block_size(static_cast<uint8_t>(profile.level_count - 1)));
	// The outer ring snaps independently, so the far candidate can sit one
	// terminal block beyond the nominal grid extent on either axis.
	return (static_cast<double>(profile.candidate_grid_radius) + 1.0) * terminal_block_m *
		1.41421356237309504880;
}

bool BlockClipmapRenderer::initialize(
	godot::RenderingServer* rendering_server,
	godot::RID scenario,
	const Multinet::WorldDomainManifest& domain,
	const Multinet::TerrainRecipeIdentity& expected_recipe,
	const Multinet::TerrainFallbackBounds& expected_fallback_bounds
) {
	if (!domain.is_valid()) return false;
	active_domain = domain;
	profile.level_count = Multinet::derive_flat_presentation_bccm_level_count(
		domain,
		profile.lod0_block_size,
		profile.candidate_grid_radius,
		8
	);
	return initialize(rendering_server, scenario, Multinet::make_compatibility_scale_manifest(domain), expected_recipe, expected_fallback_bounds);
}

godot::RID BlockClipmapRenderer::create_master_block_mesh(bool p_diamond_triangulation) {
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

			if (p_diamond_triangulation && ((x + z) & 1u) != 0u) {
				indices.push_back(v00);
				indices.push_back(v10);
				indices.push_back(v11);
				indices.push_back(v00);
				indices.push_back(v11);
				indices.push_back(v01);
			} else {
				indices.push_back(v00);
				indices.push_back(v10);
				indices.push_back(v01);
				indices.push_back(v10);
				indices.push_back(v11);
				indices.push_back(v01);
			}
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
void BlockClipmapRenderer::upload_zero_scalar_layer(
	godot::RenderingServer* rs,
	uint8_t lod
) {
	static constexpr size_t PAGE_FLOATS = 19 * 19;
	static constexpr size_t PAGE_BYTES = PAGE_FLOATS * 4;

	std::array<float, PAGE_FLOATS> zero_data;
	zero_data.fill(0.0f);

	for (size_t i = 0; i < PAGE_FLOATS; ++i) {
		assert(zero_data[i] == 0.0f);
	}

	godot::PackedByteArray raw;
	raw.resize(PAGE_BYTES);
	std::memcpy(raw.ptrw(), zero_data.data(), PAGE_BYTES);

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
	if (!active_domain.is_valid()) profile.level_count = 8;

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
	staging_images.resize(limits_.max_page_commits);
	for (uint32_t i = 0; i < limits_.max_page_commits; ++i) {
		staging_images[i].data.resize(19 * 19 * 4);
		// Zero the staging buffer.
		uint8_t* raw = staging_images[i].data.ptrw();
		std::memset(raw, 0, 19 * 19 * 4);
		staging_images[i].img = godot::Image::create_from_data(
			19, 19, false, godot::Image::FORMAT_RF, staging_images[i].data
		);
	}

	// --- Master mesh + shader -------------------------------------------------
	master_mesh_rid = create_master_block_mesh(true);
	if (!master_mesh_rid.is_valid()) {
		cleanup();
		return false;
	}
	legacy_mesh_rid = create_master_block_mesh(false);
	if (!legacy_mesh_rid.is_valid()) {
		cleanup();
		return false;
	}
	acquire_v5_chart_globals(rs);
	has_v5_chart_global_lease_ = true;

	shader_data = create_bccm_shader_material();
	if (!shader_data.shader_rid.is_valid() || !shader_data.material_rid.is_valid()) {
		std::cerr << "[Multinet BCCM] ERROR: Shader or material RID invalid after creation." << std::endl;
		cleanup();
		return false;
	}

	rs->mesh_surface_set_material(master_mesh_rid, 0, shader_data.material_rid);
	rs->mesh_surface_set_material(legacy_mesh_rid, 0, shader_data.material_rid);

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

	const bool is_certified_profile = (profile.candidate_grid_radius == 4 && profile.inner_hole_radius == 2);
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
		const godot::RID selected_mesh = (diamond_triangulation_enabled && lod == 0) ? master_mesh_rid : legacy_mesh_rid;
		rs->multimesh_set_mesh(level.multimesh_rid, selected_mesh);
		rs->multimesh_set_custom_aabb(
			level.multimesh_rid,
			godot::AABB(godot::Vector3(0, 0, 0), godot::Vector3(0.01f, 0.01f, 0.01f))
		);

		level.instance_rid = rs->instance_create();
		if (!level.instance_rid.is_valid()) { cleanup(); return false; }
		rs->instance_set_base(level.instance_rid, level.multimesh_rid);
		rs->instance_set_scenario(level.instance_rid, scenario_rid);
		rs->instance_set_visible(level.instance_rid, is_visible_);
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
		rs->material_set_param(level.material_rid, "parent_morph_enabled", is_certified_profile);
		rs->material_set_param(level.material_rid, "current_lod_index", static_cast<uint32_t>(lod));
		rs->material_set_param(level.material_rid, "active_ordinary_level_count", static_cast<uint32_t>(profile.level_count));
		rs->instance_geometry_set_material_override(level.instance_rid, level.material_rid);

		// Layer 0: permanent zero scalar page.
		upload_zero_scalar_layer(rs, lod);
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

void BlockClipmapRenderer::bind_material_uniforms(
	const Multinet::TerrainRecipe& recipe,
	const Multinet::WorldScaleManifest& scale
) {
	cached_recipe_ = recipe;
	cached_scale_ = scale;
#ifndef MULTINET_TEST
	godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
	if (!rs) return;
	const bool is_certified_profile = (profile.candidate_grid_radius == 4 && profile.inner_hole_radius == 2);

	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		godot::RID mat = levels[lod].material_rid;
		if (!mat.is_valid()) continue;

		rs->material_set_param(mat, "terrain_seed", recipe.identity.world_seed);
		rs->material_set_param(mat, "continental_frequency", recipe.legacy_signals.continental_frequency);
		rs->material_set_param(mat, "persistence", recipe.legacy_signals.persistence);
		rs->material_set_param(mat, "lacunarity", recipe.legacy_signals.lacunarity);
		rs->material_set_param(mat, "octave_count", recipe.legacy_signals.octave_count);
		rs->material_set_param(mat, "min_elevation", recipe.legacy_signals.min_elevation_m);
		rs->material_set_param(mat, "max_elevation", recipe.legacy_signals.max_elevation_m);
		rs->material_set_param(mat, "chart_half_extent_m", static_cast<float>(scale.chart_half_extent_mm) * 0.001f);
		rs->material_set_param(mat, "logical_area_radius_m", static_cast<float>(scale.logical_area_radius_m));
		rs->material_set_param(mat, "lod_spacing", static_cast<float>(profile.get_lod_spacing(lod)));
		rs->material_set_param(mat, "lod_block_size", static_cast<float>(profile.get_lod_block_size(lod)));
		rs->material_set_param(mat, "parent_morph_enabled", is_certified_profile);
		rs->material_set_param(mat, "current_lod_index", static_cast<uint32_t>(lod));
		rs->material_set_param(mat, "active_ordinary_level_count", static_cast<uint32_t>(profile.level_count));
		rs->material_set_param(mat, "world_domain_topology", 1u);
		rs->material_set_param(mat, "finite_half_extent_x_m", 0.0f);
		rs->material_set_param(mat, "finite_half_extent_z_m", 0.0f);
		rs->material_set_param(mat, "analytic_normal_sample_step_m", Multinet::CANONICAL_ANALYTIC_NORMAL_SAMPLE_STEP_M);
		rs->material_set_param(mat, "face_color_0", godot::Vector3(0.86f, 0.30f, 0.28f));
		rs->material_set_param(mat, "face_color_1", godot::Vector3(0.24f, 0.62f, 0.94f));
		rs->material_set_param(mat, "face_color_2", godot::Vector3(0.34f, 0.82f, 0.42f));
		rs->material_set_param(mat, "face_color_3", godot::Vector3(0.68f, 0.38f, 0.88f));
		rs->material_set_param(mat, "face_color_4", godot::Vector3(0.94f, 0.70f, 0.24f));
		rs->material_set_param(mat, "face_color_5", godot::Vector3(0.90f, 0.34f, 0.70f));
		rs->material_set_param(mat, "face_colors_enabled", face_colors_enabled);
		rs->material_set_param(mat, "chp_gpu_effective", false);
		rs->material_set_param(mat, "chp_function_class", 2u);
		rs->material_set_param(mat, "chp_radius_m", 0.0f);
		rs->material_set_param(mat, "chp_inverse_radius", 0.0f);
		rs->material_set_param(mat, "chp_inverse_radius_squared", 0.0f);
		rs->material_set_param(mat, "chp_camera_altitude_m", 0.0f);
		rs->material_set_param(mat, "chp_certified_max_distance_m", 0.0f);
		rs->material_set_param(mat, "chp_certified_max_u", 0.0f);
		rs->material_set_param(mat, "chp_debug_reconstruction_mode", chp_debug_reconstruction_mode);
		rs->material_set_param(mat, "chp_debug_negative_height_color", chp_debug_negative_height_color);
		rs->material_set_param(mat, "chp_debug_negative_height_exaggeration", chp_debug_negative_height_exaggeration);
		rs->material_set_param(mat, "bccm_debug_visual_mode", bccm_debug_visual_mode);

		const TerrainRootLatticeAnchors initial_anchors = compute_root_anchors(Multinet::FramePosition64{ 1.0, 0.0, 0.0 }, scale.logical_area_radius_m, recipe);
		rs->material_set_param(mat, "terrain_root_cell_0", godot::Vector3i(initial_anchors.cell[0][0], initial_anchors.cell[0][1], initial_anchors.cell[0][2]));
		rs->material_set_param(mat, "terrain_root_cell_1", godot::Vector3i(initial_anchors.cell[1][0], initial_anchors.cell[1][1], initial_anchors.cell[1][2]));
		rs->material_set_param(mat, "terrain_root_cell_2", godot::Vector3i(initial_anchors.cell[2][0], initial_anchors.cell[2][1], initial_anchors.cell[2][2]));
		rs->material_set_param(mat, "terrain_root_cell_3", godot::Vector3i(initial_anchors.cell[3][0], initial_anchors.cell[3][1], initial_anchors.cell[3][2]));
		rs->material_set_param(mat, "terrain_root_cell_4", godot::Vector3i(initial_anchors.cell[4][0], initial_anchors.cell[4][1], initial_anchors.cell[4][2]));
		rs->material_set_param(mat, "terrain_root_cell_5", godot::Vector3i(initial_anchors.cell[5][0], initial_anchors.cell[5][1], initial_anchors.cell[5][2]));
		rs->material_set_param(mat, "terrain_root_cell_6", godot::Vector3i(initial_anchors.cell[6][0], initial_anchors.cell[6][1], initial_anchors.cell[6][2]));
		rs->material_set_param(mat, "terrain_root_cell_7", godot::Vector3i(initial_anchors.cell[7][0], initial_anchors.cell[7][1], initial_anchors.cell[7][2]));

		rs->material_set_param(mat, "terrain_root_fraction_0", godot::Vector3(initial_anchors.fraction[0][0], initial_anchors.fraction[0][1], initial_anchors.fraction[0][2]));
		rs->material_set_param(mat, "terrain_root_fraction_1", godot::Vector3(initial_anchors.fraction[1][0], initial_anchors.fraction[1][1], initial_anchors.fraction[1][2]));
		rs->material_set_param(mat, "terrain_root_fraction_2", godot::Vector3(initial_anchors.fraction[2][0], initial_anchors.fraction[2][1], initial_anchors.fraction[2][2]));
		rs->material_set_param(mat, "terrain_root_fraction_3", godot::Vector3(initial_anchors.fraction[3][0], initial_anchors.fraction[3][1], initial_anchors.fraction[3][2]));
		rs->material_set_param(mat, "terrain_root_fraction_4", godot::Vector3(initial_anchors.fraction[4][0], initial_anchors.fraction[4][1], initial_anchors.fraction[4][2]));
		rs->material_set_param(mat, "terrain_root_fraction_5", godot::Vector3(initial_anchors.fraction[5][0], initial_anchors.fraction[5][1], initial_anchors.fraction[5][2]));
		rs->material_set_param(mat, "terrain_root_fraction_6", godot::Vector3(initial_anchors.fraction[6][0], initial_anchors.fraction[6][1], initial_anchors.fraction[6][2]));
		rs->material_set_param(mat, "terrain_root_fraction_7", godot::Vector3(initial_anchors.fraction[7][0], initial_anchors.fraction[7][1], initial_anchors.fraction[7][2]));
	}
#endif
}

void BlockClipmapRenderer::bind_material_uniforms(
	const Multinet::TerrainRecipe& recipe,
	const Multinet::WorldDomainManifest& domain
) {
	if (!domain.is_valid()) return;
	if (!domain.is_finite()) {
		bind_material_uniforms(recipe, domain.closed_surface);
		return;
	}
#ifndef MULTINET_TEST
	godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
	if (!rs) return;
	const bool is_certified_profile = (profile.candidate_grid_radius == 4 && profile.inner_hole_radius == 2);
	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		godot::RID mat = levels[lod].material_rid;
		if (!mat.is_valid()) continue;
		rs->material_set_param(mat, "terrain_seed", recipe.identity.world_seed);
		rs->material_set_param(mat, "continental_frequency", recipe.legacy_signals.continental_frequency);
		rs->material_set_param(mat, "persistence", recipe.legacy_signals.persistence);
		rs->material_set_param(mat, "lacunarity", recipe.legacy_signals.lacunarity);
		rs->material_set_param(mat, "octave_count", recipe.legacy_signals.octave_count);
		rs->material_set_param(mat, "min_elevation", recipe.legacy_signals.min_elevation_m);
		rs->material_set_param(mat, "max_elevation", recipe.legacy_signals.max_elevation_m);
		rs->material_set_param(mat, "chart_half_extent_m", 0.0f);
		rs->material_set_param(mat, "logical_area_radius_m", 0.0f);
		rs->material_set_param(mat, "lod_spacing", static_cast<float>(profile.get_lod_spacing(lod)));
		rs->material_set_param(mat, "lod_block_size", static_cast<float>(profile.get_lod_block_size(lod)));
		rs->material_set_param(mat, "parent_morph_enabled", is_certified_profile);
		rs->material_set_param(mat, "current_lod_index", static_cast<uint32_t>(lod));
		rs->material_set_param(mat, "active_ordinary_level_count", static_cast<uint32_t>(profile.level_count));
		rs->material_set_param(mat, "world_domain_topology", 0u);
		rs->material_set_param(mat, "finite_half_extent_x_m", static_cast<float>(domain.finite.half_extent_x_mm) * 0.001f);
		rs->material_set_param(mat, "finite_half_extent_z_m", static_cast<float>(domain.finite.half_extent_z_mm) * 0.001f);
		rs->material_set_param(mat, "analytic_normal_sample_step_m", Multinet::CANONICAL_ANALYTIC_NORMAL_SAMPLE_STEP_M);
		rs->material_set_param(mat, "face_colors_enabled", face_colors_enabled);
		rs->material_set_param(mat, "chp_gpu_effective", false);
		rs->material_set_param(mat, "chp_function_class", 2u);
		rs->material_set_param(mat, "chp_radius_m", 0.0f);
		rs->material_set_param(mat, "chp_inverse_radius", 0.0f);
		rs->material_set_param(mat, "chp_inverse_radius_squared", 0.0f);
		rs->material_set_param(mat, "chp_camera_altitude_m", 0.0f);
		rs->material_set_param(mat, "chp_certified_max_distance_m", 0.0f);
		rs->material_set_param(mat, "chp_certified_max_u", 0.0f);
		rs->material_set_param(mat, "chp_debug_reconstruction_mode", chp_debug_reconstruction_mode);
		rs->material_set_param(mat, "chp_debug_negative_height_color", chp_debug_negative_height_color);
		rs->material_set_param(mat, "chp_debug_negative_height_exaggeration", chp_debug_negative_height_exaggeration);
		rs->material_set_param(mat, "bccm_debug_visual_mode", bccm_debug_visual_mode);
	}
#endif
}

void BlockClipmapRenderer::set_face_colors_enabled(bool enabled) noexcept {
	face_colors_enabled = enabled;
#ifndef MULTINET_TEST
	godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
	if (!rs) return;
	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		godot::RID mat = levels[lod].material_rid;
		if (mat.is_valid()) rs->material_set_param(mat, "face_colors_enabled", face_colors_enabled);
	}
#endif
}

void BlockClipmapRenderer::set_diamond_triangulation_enabled(bool enabled) noexcept {
	diamond_triangulation_enabled = enabled;
#ifndef MULTINET_TEST
	godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
	if (!rs) return;
	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		if (levels[lod].multimesh_rid.is_valid()) {
			const godot::RID selected_mesh = (diamond_triangulation_enabled && lod == 0) ? master_mesh_rid : legacy_mesh_rid;
			rs->multimesh_set_mesh(levels[lod].multimesh_rid, selected_mesh);
		}
	}
#endif
}

void BlockClipmapRenderer::set_bccm_debug_visual_mode(int mode) noexcept {
	bccm_debug_visual_mode = mode;
#ifndef MULTINET_TEST
	godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
	if (!rs) return;
	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		godot::RID mat = levels[lod].material_rid;
		if (mat.is_valid()) {
			rs->material_set_param(mat, "bccm_debug_visual_mode", mode);
		}
	}
#endif
}

void BlockClipmapRenderer::set_visible(bool visible) noexcept {
	is_visible_ = visible;
#ifndef MULTINET_TEST
	godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
	if (!rs) return;
	for (auto& lvl : levels) {
		if (lvl.instance_rid.is_valid()) {
			rs->instance_set_visible(lvl.instance_rid, visible);
		}
	}
	if (frozen_frustum_instance_rid_.is_valid()) {
		rs->instance_set_visible(frozen_frustum_instance_rid_, visible);
	}
#endif
}

void BlockClipmapRenderer::update_frozen_view_presentation_delta(
	const godot::Vector3& p_camera_delta,
	const multinet::rendering::chp::CurvedHorizonView* chp_view,
	double delta_seconds
) noexcept {
	const bool chp_effective = chp_view && chp_view->chp_effective && has_active_presentation_binding;
	last_bound_chp_gpu_effective_ = chp_effective;
	if (chp_effective) {
		last_bound_chp_camera_altitude_m_ = static_cast<float>(chp_view->camera_surface_height_m);
	}

	last_cut_diagnostics_.freeze_update_active = true;
	last_cut_diagnostics_.camera_delta_x_m = p_camera_delta.x;
	last_cut_diagnostics_.camera_delta_z_m = p_camera_delta.z;
	last_cut_diagnostics_.ground_plane_distance_moved_m = std::sqrt(
		p_camera_delta.x * p_camera_delta.x + p_camera_delta.z * p_camera_delta.z
	);
	last_cut_diagnostics_.delta_seconds = delta_seconds;
	if (delta_seconds > 0.0 && std::isfinite(delta_seconds)) {
		last_cut_diagnostics_.estimated_speed_m_s = last_cut_diagnostics_.ground_plane_distance_moved_m / delta_seconds;
		last_cut_diagnostics_.estimated_speed_km_s = last_cut_diagnostics_.estimated_speed_m_s * 0.001;
	} else {
		last_cut_diagnostics_.estimated_speed_m_s = 0.0;
		last_cut_diagnostics_.estimated_speed_km_s = 0.0;
	}
	last_cut_diagnostics_.chp_effective = chp_effective;
	if (chp_view) {
		last_cut_diagnostics_.chp_signed_altitude_m = static_cast<float>(chp_view->camera_surface_height_m);
	}

#ifndef MULTINET_TEST
	if (!is_initialized || !has_active_presentation_binding) return;
	godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
	if (!rs) return;

	// A. Apply continuous camera delta to submitted MultiMesh instances
	if (p_camera_delta.length_squared() > 1e-12f) {
		for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
			LODLevelData& level = levels[lod];
			if (!level.multimesh_rid.is_valid() || level.last_visible_count == 0) continue;
			godot::PackedFloat32Array& buffer = multimesh_gpu_buffers[lod][level.submitted_buffer_index];
			float* values = buffer.ptrw();
			for (uint32_t i = 0; i < level.last_visible_count; ++i) {
				const size_t base = static_cast<size_t>(i) * 16u;
				values[base + 3] -= p_camera_delta.x;
				values[base + 7] -= p_camera_delta.y;
				values[base + 11] -= p_camera_delta.z;
			}
			rs->multimesh_set_buffer(level.multimesh_rid, buffer);
		}
		active_view_world_position += p_camera_delta;

		// Shift frozen frustum debug visualization instance by -delta so it stays fixed in world space
		if (has_frozen_frustum_ && frozen_frustum_instance_rid_.is_valid()) {
			frozen_frustum_transform_.origin -= p_camera_delta;
			rs->instance_set_transform(frozen_frustum_instance_rid_, frozen_frustum_transform_);
		}
	}

	// B. Refresh view-dependent CHP GPU state
	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		godot::RID material = levels[lod].material_rid;
		if (!material.is_valid()) continue;
		rs->material_set_param(material, "chp_gpu_effective", chp_effective);
		if (chp_effective) {
			rs->material_set_param(material, "chp_camera_altitude_m", static_cast<float>(chp_view->camera_surface_height_m));
		}
	}
#else
	// In test mode: apply translation delta to local test ring buffers
	if (p_camera_delta.length_squared() > 1e-12f) {
		for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
			LODLevelData& level = levels[lod];
			if (level.last_visible_count == 0) continue;
			auto& ring_buf = multimesh_ring_buffers[lod][level.submitted_buffer_index];
			for (uint32_t i = 0; i < level.last_visible_count; ++i) {
				const size_t base = static_cast<size_t>(i) * 16u;
				if (base + 11 < ring_buf.size()) {
					ring_buf[base + 3] -= p_camera_delta.x;
					ring_buf[base + 7] -= p_camera_delta.y;
					ring_buf[base + 11] -= p_camera_delta.z;
				}
			}
		}
		active_view_world_position += p_camera_delta;
	}
#endif
}

void BlockClipmapRenderer::update_frozen_view_presentation(
	const godot::Vector3& p_camera_world_position,
	const multinet::rendering::chp::CurvedHorizonView* chp_view
) noexcept {
	const godot::Vector3 delta = p_camera_world_position - active_view_world_position;
	update_frozen_view_presentation_delta(delta, chp_view);
}

void BlockClipmapRenderer::set_parent_morph_view_offset(const godot::Vector2& p_offset_m) noexcept {
#ifndef MULTINET_TEST
	if (!is_initialized) return;
	godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
	if (!rs) return;
	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		godot::RID material = levels[lod].material_rid;
		if (material.is_valid()) {
			rs->material_set_param(material, "parent_morph_view_offset_m", p_offset_m);
		}
	}
#else
	(void)p_offset_m;
#endif
}

void BlockClipmapRenderer::set_frozen_frustum_visualization(
	const godot::Transform3D& camera_transform,
	float fov_deg,
	float near_m,
	float far_m,
	float aspect
) noexcept {
#ifndef MULTINET_TEST
	godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
	if (!rs || !scenario_rid.is_valid()) return;

	clear_frozen_frustum_visualization();

	const float vis_near = std::max(0.1f, near_m);
	const float vis_far = std::min(far_m, 5000.0f);
	const float half_fov_rad = godot::Math::deg_to_rad(fov_deg * 0.5f);
	const float tan_half = std::tan(half_fov_rad);

	const float near_h = 2.0f * vis_near * tan_half;
	const float near_w = near_h * aspect;
	const float far_h = 2.0f * vis_far * tan_half;
	const float far_w = far_h * aspect;

	const godot::Vector3 nlb(-near_w * 0.5f, -near_h * 0.5f, -vis_near);
	const godot::Vector3 nrb( near_w * 0.5f, -near_h * 0.5f, -vis_near);
	const godot::Vector3 nrt( near_w * 0.5f,  near_h * 0.5f, -vis_near);
	const godot::Vector3 nlt(-near_w * 0.5f,  near_h * 0.5f, -vis_near);

	const godot::Vector3 flb(-far_w * 0.5f, -far_h * 0.5f, -vis_far);
	const godot::Vector3 frb( far_w * 0.5f, -far_h * 0.5f, -vis_far);
	const godot::Vector3 frt( far_w * 0.5f,  far_h * 0.5f, -vis_far);
	const godot::Vector3 flt(-far_w * 0.5f,  far_h * 0.5f, -vis_far);

	const godot::Basis& b = camera_transform.basis;
	const auto xform = [&](const godot::Vector3& p) {
		return b.xform(p);
	};

	godot::PackedVector3Array vertices;
	vertices.resize(24);
	// Near plane rect
	vertices[0] = xform(nlb); vertices[1] = xform(nrb);
	vertices[2] = xform(nrb); vertices[3] = xform(nrt);
	vertices[4] = xform(nrt); vertices[5] = xform(nlt);
	vertices[6] = xform(nlt); vertices[7] = xform(nlb);
	// Far plane rect
	vertices[8]  = xform(flb); vertices[9]  = xform(frb);
	vertices[10] = xform(frb); vertices[11] = xform(frt);
	vertices[12] = xform(frt); vertices[13] = xform(flt);
	vertices[14] = xform(flt); vertices[15] = xform(flb);
	// Connecting rays
	vertices[16] = xform(nlb); vertices[17] = xform(flb);
	vertices[18] = xform(nrb); vertices[19] = xform(frb);
	vertices[20] = xform(nrt); vertices[21] = xform(frt);
	vertices[22] = xform(nlt); vertices[23] = xform(flt);

	godot::Array arrays;
	arrays.resize(godot::RenderingServer::ARRAY_MAX);
	arrays[godot::RenderingServer::ARRAY_VERTEX] = vertices;

	frozen_frustum_mesh_rid_ = rs->mesh_create();
	rs->mesh_add_surface_from_arrays(frozen_frustum_mesh_rid_, godot::RenderingServer::PRIMITIVE_LINES, arrays);

	frozen_frustum_instance_rid_ = rs->instance_create();
	rs->instance_set_base(frozen_frustum_instance_rid_, frozen_frustum_mesh_rid_);
	rs->instance_set_scenario(frozen_frustum_instance_rid_, scenario_rid);
	frozen_frustum_transform_ = godot::Transform3D();
	rs->instance_set_transform(frozen_frustum_instance_rid_, frozen_frustum_transform_);
	rs->instance_set_visible(frozen_frustum_instance_rid_, is_visible_);
	has_frozen_frustum_ = true;
#else
	has_frozen_frustum_ = true;
#endif
}

void BlockClipmapRenderer::clear_frozen_frustum_visualization() noexcept {
#ifndef MULTINET_TEST
	godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
	if (rs) {
		if (frozen_frustum_instance_rid_.is_valid()) {
			rs->free_rid(frozen_frustum_instance_rid_);
			frozen_frustum_instance_rid_ = godot::RID();
		}
		if (frozen_frustum_mesh_rid_.is_valid()) {
			rs->free_rid(frozen_frustum_mesh_rid_);
			frozen_frustum_mesh_rid_ = godot::RID();
		}
		frozen_frustum_transform_ = godot::Transform3D();
	}
#endif
	has_frozen_frustum_ = false;
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

	staging_images.resize(limits_.max_page_commits);
	for (uint32_t i = 0; i < limits_.max_page_commits; ++i) {
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
	clear_frozen_frustum_visualization();
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
	if (legacy_mesh_rid.is_valid()) {
		rs->free_rid(legacy_mesh_rid);
		legacy_mesh_rid = godot::RID();
	}

	if (shader_data.material_rid.is_valid()) {
		rs->free_rid(shader_data.material_rid);
		shader_data.material_rid = godot::RID();
	}
	if (shader_data.shader_rid.is_valid()) {
		rs->free_rid(shader_data.shader_rid);
		shader_data.shader_rid = godot::RID();
	}
	if (has_v5_chart_global_lease_) {
		release_v5_chart_globals(rs);
		has_v5_chart_global_lease_ = false;
	}
	#endif
	is_initialized = false;
	active_domain = Multinet::WorldDomainManifest{};
	profile.level_count = 8;
	has_active_presentation_binding = false;

#ifndef MULTINET_TEST
	active_presentation_basis = godot::Basis();
	active_presentation_origin = godot::Vector3();
	active_view_world_position = godot::Vector3();
#endif
	bound_logical_chart_root_ = TerrainSamplePatchKey{};

	bound_logical_chart_root_presentation_x_m_ = 0.0;
	bound_logical_chart_root_presentation_z_m_ = 0.0;
	has_bound_logical_chart_root_ = false;
}

struct FrameDemandEntry {
	TerrainRenderBlockKey key;
	Multinet::TerrainPageRequestContext request_context{};
	Multinet::TerrainRequestClass request_class{ Multinet::TerrainRequestClass::ImmediateVisible };
	int64_t dist_sq_m{ 0 };
	BlockPlacement placement;
	uint8_t raw_edge_mask{ 0 };
	Multinet::TerrainSourceState source_state{ Multinet::TerrainSourceState::Missing };
	Multinet::TerrainSourceRecord source_record{};
	uint32_t resident_gpu_layer{ 0 };
	bool is_wanted{ false };
	bool is_admitted{ false };
	bool is_visible{ false };
};

struct FrameDemandTable {
	static constexpr size_t CAPACITY = Multinet::FRAME_DEMAND_CAPACITY;
	std::array<FrameDemandEntry, CAPACITY> entries{};
	size_t count{ 0 };

	FrameDemandEntry* find(
		const TerrainRenderBlockKey& key,
		const TerrainSamplePatchKey& sample_patch = TerrainSamplePatchKey{}
	) noexcept {
		for (size_t i = 0; i < count; ++i) {
			if (entries[i].key == key &&
				entries[i].request_context.identity.sample_patch == sample_patch) return &entries[i];
		}
		return nullptr;
	}

	FrameDemandEntry* insert_or_update(
		const TerrainRenderBlockKey& key,
		const Multinet::TerrainPageRequestContext& req_ctx,
		Multinet::TerrainRequestClass request_class,
		int64_t dist_sq_m,
		const BlockPlacement& place,
		uint8_t raw_edge_mask = 0,
		bool is_visible = false
	) noexcept {
		FrameDemandEntry* existing = find(key, req_ctx.identity.sample_patch);
		if (existing) {
			existing->request_class = Multinet::merge_priority(existing->request_class, request_class);
			existing->dist_sq_m = std::min(existing->dist_sq_m, dist_sq_m);
			if (is_visible) existing->is_visible = true;
			if (raw_edge_mask != 0) existing->raw_edge_mask = raw_edge_mask;
			return existing;
		}
		if (count >= CAPACITY) return nullptr;

		FrameDemandEntry& e = entries[count++];
		e.key = key;
		e.request_context = req_ctx;
		e.request_class = request_class;
		e.dist_sq_m = dist_sq_m;
		e.placement = place;
		e.raw_edge_mask = raw_edge_mask;
		e.source_state = Multinet::TerrainSourceState::Missing;
		e.source_record = Multinet::TerrainSourceRecord{};
		e.resident_gpu_layer = 0;
		e.is_wanted = true;
		e.is_admitted = false;
		e.is_visible = is_visible;
		return &e;
	}
};

// ─── compute_update (CPU — unconditional) ─────────────────────────────────────

TerrainUpdateResult BlockClipmapRenderer::compute_update(
	const godot::Vector3& cam_pos,
	const FrustumPlanes& frustum,
	const Multinet::WorldScaleManifest& scale,
	const BCCMCameraState& cam_state,
	const BCCMSourceExpectation& expectation,
	Multinet::TerrainRenderSource* terrain_source,
	const multinet::rendering::chp::CurvedHorizonView* chp_view,
	double delta_seconds
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

	has_active_presentation_binding = cam_state.has_presentation_binding;
	active_presentation_basis = cam_state.presentation_basis;
	active_presentation_origin = cam_state.presentation_origin;
	active_view_world_position = cam_pos;

	render_frame_id++;

	last_expectation = expectation;
	if (terrain_source) {
		last_source_diagnostics_.source_pending_count = terrain_source->get_pending_record_count();
		last_source_diagnostics_.source_in_flight_count = terrain_source->get_in_flight_count();
		last_source_diagnostics_.executor_submit_count = terrain_source->get_executor_submit_count();
		last_source_diagnostics_.incompatible_jobs_cancelled = terrain_source->get_cancelled_incompatible_count();
		last_source_diagnostics_.commit_pending_call_count = terrain_source->get_commit_pending_call_count();
		last_source_diagnostics_.request_record_call_count = terrain_source->get_request_record_call_count();
		last_source_diagnostics_.rejected_delta_publication_count = terrain_source->get_rejected_delta_publication_count();
	}
	Multinet::TerrainRenderPublicationView publication{};

	// Phase 0 — validate source snapshot against external authority.
	bool snapshot_valid = false;
	if (terrain_source) {
		publication = terrain_source->get_publication_view();
		// Retain for diagnostics.
		last_snapshot = publication.source;

		snapshot_valid =
			publication.source.world_manifest_hash == expectation.world_manifest_hash &&
			publication.source.topology_version == expectation.topology_version &&
			publication.source.projection_version == expectation.projection_version &&
			publication.source.recipe_identity == expectation.recipe_identity &&
			publication.source.terrain_version == expectation.terrain_version &&
			publication.source.source_version == expectation.source_version &&
			expectation.gpu_analytic_version == CANONICAL_ANALYTIC_TERRAIN_GPU_VERSION_1;

		static int snap_print_count = 0;
		if (!snapshot_valid && ++snap_print_count <= 3) {
			std::cerr << "[RENDERER] snapshot_valid=FALSE"
			          << " mh:" << publication.source.world_manifest_hash << "==" << expectation.world_manifest_hash << "?" << (publication.source.world_manifest_hash == expectation.world_manifest_hash)
			          << " tv:" << publication.source.topology_version << "==" << expectation.topology_version << "?" << (publication.source.topology_version == expectation.topology_version)
			          << " pv:" << publication.source.projection_version << "==" << expectation.projection_version << "?" << (publication.source.projection_version == expectation.projection_version)
			          << " ri:" << (publication.source.recipe_identity == expectation.recipe_identity)
			          << " terv:" << publication.source.terrain_version << "==" << expectation.terrain_version << "?" << (publication.source.terrain_version == expectation.terrain_version)
			          << " srcv:" << publication.source.source_version << "==" << expectation.source_version << "?" << (publication.source.source_version == expectation.source_version)
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
	const bool frame_ok = active_domain.is_valid()
		? Multinet::try_domain_surface_to_frame(cam_state.canonical_position, cam_state.active_frame, active_domain, cam_frame_pos)
		: Multinet::try_surface_to_frame(cam_state.canonical_position, cam_state.active_frame, scale, cam_frame_pos);
	if (!frame_ok) {
		return result;
	}

	auto dot = [](const Multinet::FramePosition64& p, const Multinet::Vec3d& axis) {
		return p.x * axis.x + p.y * axis.y + p.z * axis.z;
	};
	double active_cam_u = cam_state.active_frame.origin.u_m + dot(cam_frame_pos, cam_state.active_frame.tangent_basis.u_axis);
	double active_cam_v = cam_state.active_frame.origin.v_m + dot(cam_frame_pos, cam_state.active_frame.tangent_basis.v_axis);
	const bool closed_presentation = active_domain.is_valid() && !active_domain.is_finite();
	if (closed_presentation && cam_state.has_presentation_position) {
		active_cam_u = cam_state.presentation_x_m;
		active_cam_v = cam_state.presentation_z_m;
	}

	// Reset submission plan and diagnostic metrics
	last_submission_plan.valid = true;
	for (auto& lod : last_submission_plan.lods) {
		lod.count = 0;
		lod.lod_0_6_layer_zero_count = 0;
		lod.lod_7_layer_zero_count = 0;
	}
	last_streaming_diagnostics = StreamingDiagnosticsSnapshot{};

	cut_render_update_serial_++;
	FrameCutDiagnostics frame_cut_diag{};
	frame_cut_diag.render_update_serial = cut_render_update_serial_;
	frame_cut_diag.presentation_observer_x_m = active_cam_u;
	frame_cut_diag.presentation_observer_z_m = active_cam_v;
	frame_cut_diag.canonical_face = static_cast<uint8_t>(cam_state.canonical_position.face);
	frame_cut_diag.canonical_u_m = cam_state.canonical_position.u_m;
	frame_cut_diag.canonical_v_m = cam_state.canonical_position.v_m;
	frame_cut_diag.delta_seconds = delta_seconds;

	if (has_last_cut_cam_pos_) {
		frame_cut_diag.camera_delta_x_m = active_cam_u - last_cut_cam_u_;
		frame_cut_diag.camera_delta_z_m = active_cam_v - last_cut_cam_v_;
		frame_cut_diag.ground_plane_distance_moved_m = std::sqrt(
			frame_cut_diag.camera_delta_x_m * frame_cut_diag.camera_delta_x_m +
			frame_cut_diag.camera_delta_z_m * frame_cut_diag.camera_delta_z_m
		);
		if (delta_seconds > 0.0 && std::isfinite(delta_seconds)) {
			frame_cut_diag.estimated_speed_m_s = frame_cut_diag.ground_plane_distance_moved_m / delta_seconds;
			frame_cut_diag.estimated_speed_km_s = frame_cut_diag.estimated_speed_m_s * 0.001;
		} else {
			frame_cut_diag.estimated_speed_m_s = 0.0;
			frame_cut_diag.estimated_speed_km_s = 0.0;
		}
	}
	last_cut_cam_u_ = active_cam_u;
	last_cut_cam_v_ = active_cam_v;
	has_last_cut_cam_pos_ = true;


	frame_cut_diag.active_lod_count = profile.level_count;
	frame_cut_diag.bccm_streams_submitted = profile.level_count;
	frame_cut_diag.chp_effective = chp_view && chp_view->chp_effective;
	if (chp_view) {
		frame_cut_diag.chp_signed_altitude_m = static_cast<float>(chp_view->camera_surface_height_m);
	}

	// Phase 3 — Unified FrameDemandTable Construction & Candidate Enumeration
	FrameDemandTable demand_table;

	struct GlobalCandInfo {
		TerrainRenderBlockKey key;
		TerrainPresentationBlockKey presentation_key{};
		TerrainSamplePatchKey sample_patch{};
		godot::Basis block_to_active_frame;
		godot::Vector3 local_origin;
		godot::AABB local_aabb;
		bool is_visible{ false };
		int64_t dist_sq_m{ 0 };
		uint8_t lod{ 0 };
		uint8_t raw_edge_mask{ 0 };
	};

	thread_local static std::array<std::array<GlobalCandInfo, BlockClipmapProfile::MAX_CANDIDATES>, BlockClipmapProfile::MAX_LEVELS> lod_candidates;
	std::array<uint32_t, BlockClipmapProfile::MAX_LEVELS> lod_candidate_counts{};

	Multinet::TerrainFallbackBounds fb = snapshot_valid ? last_snapshot.fallback_bounds : fallback_bounds;
	auto make_closed_identity = [&](int64_t presentation_u, int64_t presentation_v, uint8_t lod,
		TerrainPresentationBlockKey& out_presentation,
		TerrainRenderBlockKey& out_owner,
		TerrainSamplePatchKey& out_patch,
		uint32_t& out_transitions) -> bool {
		const double block_size = profile.get_lod_block_size(lod);
		const double centre_x = (static_cast<double>(presentation_u) + 0.5) * block_size;
		const double centre_z = (static_cast<double>(presentation_v) + 0.5) * block_size;
		const Multinet::SurfaceFrame& unfolding_root = cam_state.has_unfolding_root
			? cam_state.unfolding_root_frame : cam_state.active_frame;
		const double root_presentation_x = cam_state.has_unfolding_root
			? cam_state.unfolding_root_presentation_x_m : active_cam_u;
		const double root_presentation_z = cam_state.has_unfolding_root
			? cam_state.unfolding_root_presentation_z_m : active_cam_v;
		Multinet::SurfacePosition64 owner_position{};
		if (!try_make_logical_sample_patch(
			unfolding_root,
			root_presentation_x,
			root_presentation_z,
			centre_x,
			centre_z,
			std::max<uint64_t>(1, cam_state.unfolding_generation),
			lod,
			ORDINARY_BCCM_V1_PROFILE,
			active_domain,
			out_patch,
			&owner_position)) return false;
		out_transitions = 0;

		const double owner_u = std::floor(owner_position.u_m / block_size);
		const double owner_v = std::floor(owner_position.v_m / block_size);
		if (owner_u < static_cast<double>((std::numeric_limits<int32_t>::min)()) ||
			owner_u > static_cast<double>((std::numeric_limits<int32_t>::max)()) ||
			owner_v < static_cast<double>((std::numeric_limits<int32_t>::min)()) ||
			owner_v > static_cast<double>((std::numeric_limits<int32_t>::max)())) return false;

		out_presentation = TerrainPresentationBlockKey{
			presentation_u, presentation_v,
			std::max<uint64_t>(1, cam_state.unfolding_generation),
			lod, ORDINARY_BCCM_V1_PROFILE
		};
		out_owner = TerrainRenderBlockKey{
			owner_position.face,
			static_cast<int32_t>(owner_u),
			static_cast<int32_t>(owner_v),
			lod, ORDINARY_BCCM_V1_PROFILE, 0
		};
		last_streaming_diagnostics.maximum_patch_transition_count = std::max(
			last_streaming_diagnostics.maximum_patch_transition_count, out_transitions);
		return out_presentation.is_valid() && out_owner.is_valid() && out_patch.is_valid();
	};

	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		const QuantizedLODCenter q_center = compute_lod_center(active_cam_u, active_cam_v, lod, profile);
		const int64_t center_bx = q_center.center_bx;
		const int64_t center_bv = q_center.center_bv;
		const double block_size = q_center.block_size_m;
		const double snap_size = q_center.snap_size_m;

		LODCutDiagnostics& lod_cut_diag = frame_cut_diag.lods[lod];
		lod_cut_diag.current_center_bx = center_bx;
		lod_cut_diag.current_center_bv = center_bv;
		lod_cut_diag.snap_period_m = snap_size;
		lod_cut_diag.candidate_count_before = levels[lod].last_candidate_count;

		if (has_last_cut_center_[lod]) {
			lod_cut_diag.prev_center_bx = last_cut_center_bx_[lod];
			lod_cut_diag.prev_center_bv = last_cut_center_bv_[lod];
			lod_cut_diag.delta_center_bx = center_bx - last_cut_center_bx_[lod];
			lod_cut_diag.delta_center_bv = center_bv - last_cut_center_bv_[lod];
			lod_cut_diag.delta_center_u_m = static_cast<double>(lod_cut_diag.delta_center_bx) * block_size;
			lod_cut_diag.delta_center_v_m = static_cast<double>(lod_cut_diag.delta_center_bv) * block_size;
			lod_cut_diag.snap_steps_crossed_u = static_cast<uint32_t>(std::llround(std::abs(lod_cut_diag.delta_center_u_m) / snap_size));
			lod_cut_diag.snap_steps_crossed_v = static_cast<uint32_t>(std::llround(std::abs(lod_cut_diag.delta_center_v_m) / snap_size));
			lod_cut_diag.max_snap_steps_crossed = std::max(lod_cut_diag.snap_steps_crossed_u, lod_cut_diag.snap_steps_crossed_v);
			lod_cut_diag.skipped_snap_event = (lod_cut_diag.max_snap_steps_crossed > 1);

			if (lod_cut_diag.skipped_snap_event) {
				frame_cut_diag.frame_skipped_snap_events++;
				total_skipped_snap_events_++;
			}
			if (lod_cut_diag.max_snap_steps_crossed > frame_cut_diag.frame_largest_snap_steps) {
				frame_cut_diag.frame_largest_snap_steps = lod_cut_diag.max_snap_steps_crossed;
				frame_cut_diag.worst_lod = lod;
				if (lod_cut_diag.snap_steps_crossed_u > lod_cut_diag.snap_steps_crossed_v) {
					frame_cut_diag.worst_axis = 1;
				} else if (lod_cut_diag.snap_steps_crossed_v > lod_cut_diag.snap_steps_crossed_u) {
					frame_cut_diag.worst_axis = 2;
				} else {
					frame_cut_diag.worst_axis = 3;
				}
			}
		}
		last_cut_center_bx_[lod] = center_bx;
		last_cut_center_bv_[lod] = center_bv;
		has_last_cut_center_[lod] = true;

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

			lod_cut_diag.current_hole_dx = hole_dx;
			lod_cut_diag.current_hole_dz = hole_dz;
			if (levels[lod].has_last_hole) {
				lod_cut_diag.prev_hole_dx = levels[lod].last_hole_dx;
				lod_cut_diag.prev_hole_dz = levels[lod].last_hole_dz;
				lod_cut_diag.hole_delta_dx = hole_dx - levels[lod].last_hole_dx;
				lod_cut_diag.hole_delta_dz = hole_dz - levels[lod].last_hole_dz;
				lod_cut_diag.hole_movement_event = (lod_cut_diag.hole_delta_dx != 0 || lod_cut_diag.hole_delta_dz != 0);
				lod_cut_diag.hole_steps_crossed = static_cast<uint32_t>(std::max(std::abs(lod_cut_diag.hole_delta_dx), std::abs(lod_cut_diag.hole_delta_dz)));
			}
			levels[lod].last_hole_dx = hole_dx;
			levels[lod].last_hole_dz = hole_dz;
			levels[lod].has_last_hole = true;
		}

		int32_t r = profile.candidate_grid_radius;
		int32_t hole_r = profile.inner_hole_radius;

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
			if (lod_candidate_counts[lod] >= BlockClipmapProfile::MAX_CANDIDATES) break;

			int32_t du = offsets[off_i].du;
			int32_t dv = offsets[off_i].dv;

			int64_t bx = center_bx + du;
			int64_t bv_coord = center_bv + dv;

			TerrainRenderBlockKey key;
			TerrainPresentationBlockKey presentation_key{};
			TerrainSamplePatchKey sample_patch{};
			uint32_t patch_transitions = 0;
			if (closed_presentation) {
				if (!make_closed_identity(
					bx, bv_coord, lod, presentation_key, key, sample_patch, patch_transitions)) {
					++last_streaming_diagnostics.closed_placement_failures;
					continue;
				}
			} else if (active_domain.is_valid()) {
				if (!make_domain_block_key(cam_state.active_frame.origin.face, bx, bv_coord, lod, active_domain, key)) continue;
			} else {
				key = make_canonical_block_key(cam_state.active_frame.origin.face, bx, bv_coord, lod, scale);
				if (!key.is_valid()) continue;
			}

			bool dup = false;
			for (uint32_t c_idx = 0; c_idx < lod_candidate_counts[lod]; ++c_idx) {
				const auto& prior = lod_candidates[lod][c_idx];
				if (closed_presentation) {
					if (prior.presentation_key == presentation_key) { dup = true; break; }
					if (prior.key == key) ++last_streaming_diagnostics.canonical_duplicate_presentations_retained;
				} else if (prior.key == key) {
					dup = true;
					break;
				}
			}
			if (dup) continue;

			Multinet::TerrainPageRequestContext req_ctx = closed_presentation
				? Multinet::make_page_request_context(key, sample_patch, profile, publication, scale)
				: Multinet::make_page_request_context(key, profile, publication, scale);

			Multinet::TerrainFallbackBounds stale_bounds{};
			stale_bounds.minimum_height = 1e9f;
			stale_bounds.maximum_height = -1e9f;
			bool found_stale = false;

			for (uint32_t j = 1; j < 128; ++j) {
				const auto& slot = levels[lod].slots[j];
				if ((slot.state == TerrainGpuPageState::Resident || slot.state == TerrainGpuPageState::UploadPending) &&
				    slot.key == req_ctx.identity.block_key &&
				    slot.sample_patch == req_ctx.identity.sample_patch &&
				    slot.page_contract_version == req_ctx.identity.page_contract_version &&
				    slot.payload_kind == req_ctx.identity.payload_kind &&
				    slot.terrain_version == req_ctx.identity.terrain_version &&
				    slot.source_version == req_ctx.identity.source_version)
				{
					stale_bounds.minimum_height = std::min(stale_bounds.minimum_height, slot.minimum_sample_m);
					stale_bounds.maximum_height = std::max(stale_bounds.maximum_height, slot.maximum_sample_m);
					found_stale = true;
				}
			}

			BlockPlacement place = closed_presentation
				? build_presentation_block_placement(
					presentation_key, key, active_cam_u, active_cam_v, fb,
					&publication.committed_delta, found_stale ? &stale_bounds : nullptr)
				: build_block_placement(key, cam_state.active_frame, scale, fb, &publication.committed_delta, found_stale ? &stale_bounds : nullptr);
			if (!place.valid) {
				if (closed_presentation) ++last_streaming_diagnostics.closed_placement_failures;
				continue;
			}

			godot::AABB global_aabb = godot::Transform3D(
				place.block_to_active_frame, godot::Vector3(0, 0, 0)
			).xform(place.local_aabb);
			global_aabb.position += place.local_origin;
			if (has_active_presentation_binding) {
				global_aabb.position += active_view_world_position;
			}
			bool is_visible = frustum.intersects_aabb(global_aabb);
			if (chp_view && chp_view->chp_effective && has_active_presentation_binding) {
				// R1: Conservative debug culling under CHP - frustum culling bypassed
				is_visible = true;
			}

			uint8_t edge_mask = 0;
			if (du == r - 1) edge_mask |= 1;
			if (du == -r)    edge_mask |= 2;
			if (dv == r - 1) edge_mask |= 4;
			if (dv == -r)    edge_mask |= 8;

			GlobalCandInfo cand;
			cand.key = key;
			cand.presentation_key = presentation_key;
			cand.sample_patch = sample_patch;
			cand.block_to_active_frame = place.block_to_active_frame;
			cand.local_origin = place.local_origin;
			cand.local_aabb = place.local_aabb;
			cand.is_visible = is_visible;
			cand.dist_sq_m = static_cast<int64_t>(offsets[off_i].dist_sq * block_size * block_size);
			cand.lod = lod;
			cand.raw_edge_mask = edge_mask;

			if (lod_candidate_counts[lod] < BlockClipmapProfile::MAX_CANDIDATES) {
				lod_candidates[lod][lod_candidate_counts[lod]++] = cand;
			} else {
				last_streaming_diagnostics.wanted_set_overflow = true;
			}

			// Populate unified FrameDemandTable!
			if (is_visible) {
				if (source_mode == TerrainSourceMode::HybridAdditiveDelta) {
					bool may_have_delta = true;
					if (publication.committed_delta.field) {
						double req_apron = profile.get_lod_spacing(lod);
						may_have_delta = sample_patch.is_valid() || (active_domain.is_valid()
							? publication.committed_delta.field->block_may_have_nonzero_delta(key, active_domain, profile, req_apron)
							: publication.committed_delta.field->block_may_have_nonzero_delta(key, scale, profile, req_apron));
					} else {
						may_have_delta = false;
					}

					bool has_stale_gpu_slot = false;
					for (uint32_t j = 1; j < 128; ++j) {
						const auto& slot = levels[lod].slots[j];
						if ((slot.state == TerrainGpuPageState::Resident || slot.state == TerrainGpuPageState::UploadPending) &&
							slot.key == key && slot.sample_patch == sample_patch) {
							has_stale_gpu_slot = true;
							break;
						}
					}

					if (may_have_delta || has_stale_gpu_slot) {
						demand_table.insert_or_update(key, req_ctx, Multinet::TerrainRequestClass::ImmediateVisible, cand.dist_sq_m, place, edge_mask, true);
					}
				} else if (source_mode == TerrainSourceMode::AbsoluteHeightPageDebug || analytic_debug_prewarm_pages) {
					if (lod < profile.level_count - 1) {
						// 1. ImmediateVisible
						demand_table.insert_or_update(key, req_ctx, Multinet::TerrainRequestClass::ImmediateVisible, cand.dist_sq_m, place, edge_mask, true);

						const uint8_t parent_lod = lod + 1 < profile.level_count ? lod + 1 : lod;
						if (closed_presentation) {
							// Coverage identities live in the same unfolding as the visible
							// block. Canonical parent arithmetic loses that phase at seams.
							int64_t parent_u = floor_div(bx, int64_t{ 2 });
							int64_t parent_v = floor_div(bv_coord, int64_t{ 2 });
							for (int sibling_index = 0; sibling_index < 4; ++sibling_index) {
								const int64_t sibling_u = parent_u * 2 + (sibling_index & 1);
								const int64_t sibling_v = parent_v * 2 + ((sibling_index >> 1) & 1);
								TerrainPresentationBlockKey sibling_presentation{};
								TerrainRenderBlockKey sibling_owner{};
								TerrainSamplePatchKey sibling_patch{};
								uint32_t sibling_transitions = 0;
								if (!make_closed_identity(sibling_u, sibling_v, lod, sibling_presentation,
									sibling_owner, sibling_patch, sibling_transitions)) {
									++last_streaming_diagnostics.closed_placement_failures;
									continue;
								}
								const auto sibling_ctx = Multinet::make_page_request_context(
									sibling_owner, sibling_patch, profile, publication, scale);
								const BlockPlacement sibling_place = build_presentation_block_placement(
									sibling_presentation, sibling_owner, active_cam_u, active_cam_v, fb,
									&publication.committed_delta);
								if (sibling_place.valid) {
									demand_table.insert_or_update(sibling_owner, sibling_ctx,
										Multinet::TerrainRequestClass::AtomicSibling, cand.dist_sq_m,
										sibling_place, 0, false);
								}
							}

							for (uint8_t target_lod = parent_lod; target_lod < profile.level_count; ++target_lod) {
								if (target_lod > parent_lod) {
									parent_u = floor_div(parent_u, int64_t{ 2 });
									parent_v = floor_div(parent_v, int64_t{ 2 });
								}
								TerrainPresentationBlockKey parent_presentation{};
								TerrainRenderBlockKey parent_owner{};
								TerrainSamplePatchKey parent_patch{};
								uint32_t parent_transitions = 0;
								if (!make_closed_identity(parent_u, parent_v, target_lod, parent_presentation,
									parent_owner, parent_patch, parent_transitions)) {
									++last_streaming_diagnostics.closed_placement_failures;
									break;
								}
								const auto parent_ctx = Multinet::make_page_request_context(
									parent_owner, parent_patch, profile, publication, scale);
								const BlockPlacement parent_place = build_presentation_block_placement(
									parent_presentation, parent_owner, active_cam_u, active_cam_v, fb,
									&publication.committed_delta);
								if (!parent_place.valid) continue;
								const Multinet::TerrainRequestClass parent_class =
									(target_lod == profile.level_count - 1)
										? Multinet::TerrainRequestClass::TerminalBootstrap
										: Multinet::TerrainRequestClass::CoarseCoverage;
								demand_table.insert_or_update(parent_owner, parent_ctx, parent_class,
									cand.dist_sq_m, parent_place, 0, false);
							}
						} else {
							// 2. AtomicSibling
							TerrainRenderBlockKey parent_key;
							if (active_domain.is_valid()) {
								if (!derive_domain_parent_key(key, parent_lod, active_domain, parent_key)) continue;
							} else {
								parent_key = derive_canonical_parent_key(key, parent_lod, scale);
							}
							std::array<TerrainRenderBlockKey, 4> sibs{};
							const uint32_t sibling_count = active_domain.is_valid()
								? enumerate_domain_child_keys(parent_key, active_domain, sibs)
								: (enumerate_canonical_child_keys(parent_key, scale, sibs), 4u);
							for (uint32_t sibling_index = 0; sibling_index < sibling_count; ++sibling_index) {
								const auto& sib = sibs[sibling_index];
								Multinet::TerrainPageRequestContext sib_ctx = Multinet::make_page_request_context(sib, profile, publication, scale);
								BlockPlacement sib_place = build_block_placement(sib, cam_state.active_frame, scale, fb, &publication.committed_delta);
								demand_table.insert_or_update(sib, sib_ctx, Multinet::TerrainRequestClass::AtomicSibling, cand.dist_sq_m, sib_place, 0, false);
							}

							// 3. CoarseCoverage & TerminalBootstrap recursively up to top level
							TerrainRenderBlockKey p_k = parent_key;
							for (uint8_t target_lod = lod + 1; target_lod < profile.level_count; ++target_lod) {
								if (target_lod > lod + 1) {
									if (active_domain.is_valid()) {
										if (!derive_domain_parent_key(p_k, target_lod, active_domain, p_k)) break;
									} else {
										p_k = derive_canonical_parent_key(p_k, target_lod, scale);
									}
								}
								Multinet::TerrainRequestClass p_class = (target_lod == profile.level_count - 1) ? Multinet::TerrainRequestClass::TerminalBootstrap : Multinet::TerrainRequestClass::CoarseCoverage;
								Multinet::TerrainPageRequestContext p_ctx = Multinet::make_page_request_context(p_k, profile, publication, scale);
								BlockPlacement p_place = build_block_placement(p_k, cam_state.active_frame, scale, fb, &publication.committed_delta);
								demand_table.insert_or_update(p_k, p_ctx, p_class, cand.dist_sq_m, p_place, 0, false);
							}
						}
					} else {
						// Requirement 1: Visible top-level (terminal) candidates are classified directly as TerminalBootstrap!
						// Do not create atomic siblings or recursive parents for top-level candidates.
						demand_table.insert_or_update(key, req_ctx, Multinet::TerrainRequestClass::TerminalBootstrap, cand.dist_sq_m, place, edge_mask, true);
					}
				}
			}
		}

		lod_cut_diag.candidate_count_after = lod_candidate_counts[lod];
		if (high_speed_cut_diagnostics_enabled_) {
			uint32_t retained = 0;
			const uint32_t prev_count = levels[lod].last_candidate_count;
			const uint32_t curr_count = lod_candidate_counts[lod];

			for (uint32_t curr_i = 0; curr_i < curr_count; ++curr_i) {
				const auto& curr_key = lod_candidates[lod][curr_i].key;
				for (uint32_t prev_i = 0; prev_i < prev_count; ++prev_i) {
					if (levels[lod].diagnostic_candidate_keys[prev_i] == curr_key) {
						retained++;
						break;
					}
				}
			}
			if (prev_count > 0) {
				lod_cut_diag.candidates_retained = retained;
				lod_cut_diag.candidates_added = (curr_count > retained) ? (curr_count - retained) : 0;
				lod_cut_diag.candidates_removed = (prev_count > retained) ? (prev_count - retained) : 0;
				lod_cut_diag.turnover_fraction = static_cast<float>(lod_cut_diag.candidates_removed) / static_cast<float>(prev_count);
			} else {
				lod_cut_diag.candidates_retained = curr_count;
				lod_cut_diag.candidates_added = 0;
				lod_cut_diag.candidates_removed = 0;
				lod_cut_diag.turnover_fraction = 0.0f;
			}


			if (lod_cut_diag.turnover_fraction > frame_cut_diag.worst_candidate_turnover) {
				frame_cut_diag.worst_candidate_turnover = lod_cut_diag.turnover_fraction;
			}

			levels[lod].last_candidate_count = lod_candidate_counts[lod];
			for (size_t i = 0; i < lod_candidate_counts[lod]; ++i) {
				levels[lod].diagnostic_candidate_keys[i] = lod_candidates[lod][i].key;
			}
		} else {
			levels[lod].last_candidate_count = lod_candidate_counts[lod];
			lod_cut_diag.candidates_retained = 0;
			lod_cut_diag.candidates_added = 0;
			lod_cut_diag.candidates_removed = 0;
			lod_cut_diag.turnover_fraction = 0.0f;
		}
	}
	last_streaming_diagnostics.frame_demand_count = static_cast<uint32_t>(demand_table.count);

	// Phase 3b — Full Wanted-Set Registration & Admission Queue (Skipped in AnalyticBase unless prewarming)
	uint32_t total_source_requests = 0;
	const bool prewarm_active = (source_mode != TerrainSourceMode::AnalyticBase || analytic_debug_prewarm_pages);
	if (snapshot_valid && terrain_source && prewarm_active) {
		terrain_source->begin_wanted_set(render_frame_id);

		for (size_t i = 0; i < demand_table.count; ++i) {
			auto& entry = demand_table.entries[i];
			if (!terrain_source->mark_wanted(entry.request_context.identity, entry.request_class, entry.dist_sq_m, render_frame_id)) {
				last_streaming_diagnostics.wanted_set_overflow = true;
				last_submission_plan.valid = false;
			}
		}
		terrain_source->end_wanted_set();

		// Query source state for all demand entries
		for (size_t i = 0; i < demand_table.count; ++i) {
			auto& entry = demand_table.entries[i];
			Multinet::TerrainSourceRecord rec;
			if (terrain_source->try_query_record(entry.request_context.identity, rec)) {
				entry.source_state = rec.state;
				entry.source_record = rec;
				if (rec.state == Multinet::TerrainSourceState::Pending || rec.state == Multinet::TerrainSourceState::Ready) {
					entry.is_admitted = true;
				}
			} else {
				entry.source_state = Multinet::TerrainSourceState::Missing;
			}
		}

		// Admission Queue from missing entries with per-LOD reservation
		struct MissingAdmission {
			size_t table_idx;
			Multinet::TerrainRequestClass request_class;
			int64_t dist_sq_m;
			uint8_t lod;
			bool selected{ false };
		};
		thread_local static std::array<MissingAdmission, BlockClipmapLimits::MAX_FRAME_DEMAND> missing_queue;
		uint32_t missing_queue_count = 0;

		for (size_t i = 0; i < demand_table.count; ++i) {
			if (demand_table.entries[i].source_state == Multinet::TerrainSourceState::Missing && missing_queue_count < BlockClipmapLimits::MAX_FRAME_DEMAND) {
				missing_queue[missing_queue_count++] = MissingAdmission{ i, demand_table.entries[i].request_class, demand_table.entries[i].dist_sq_m, demand_table.entries[i].key.lod };
			}
		}

		std::sort(missing_queue.begin(), missing_queue.begin() + missing_queue_count, [&](const MissingAdmission& a, const MissingAdmission& b) {
			uint8_t rank_a = Multinet::admission_rank(a.request_class);
			uint8_t rank_b = Multinet::admission_rank(b.request_class);
			if (rank_a != rank_b) return rank_a < rank_b;
			if (a.lod != b.lod) return a.lod < b.lod; // Fine LOD first!
			return a.dist_sq_m < b.dist_sq_m;
		});

		// Per-LOD reservation: reserve 1 admission per active LOD if missing candidates exist
		std::array<size_t, BlockClipmapLimits::MAX_SOURCE_REQUESTS> selected_admissions;
		uint32_t selected_admissions_count = 0;
		std::array<bool, BlockClipmapProfile::MAX_LEVELS> lod_admitted{};

		for (uint32_t m_idx = 0; m_idx < missing_queue_count; ++m_idx) {
			auto& adm = missing_queue[m_idx];
			if (selected_admissions_count >= limits_.max_source_requests) break;
			if (!lod_admitted[adm.lod]) {
				lod_admitted[adm.lod] = true;
				adm.selected = true;
				selected_admissions[selected_admissions_count++] = adm.table_idx;
			}
		}

		// Fill remaining budget up to limits_.max_source_requests (64) by priority rank
		for (uint32_t m_idx = 0; m_idx < missing_queue_count; ++m_idx) {
			auto& adm = missing_queue[m_idx];
			if (selected_admissions_count >= limits_.max_source_requests) break;
			if (!adm.selected) {
				adm.selected = true;
				selected_admissions[selected_admissions_count++] = adm.table_idx;
			}
		}

		for (uint32_t s_idx = 0; s_idx < selected_admissions_count; ++s_idx) {
			auto& entry = demand_table.entries[selected_admissions[s_idx]];

			Multinet::TerrainRequestMetadata meta;
			meta.request_class = entry.request_class;
			meta.distance_sq_m = entry.dist_sq_m;
			meta.wanted_set_epoch = render_frame_id;

			auto req_res = terrain_source->request_record(entry.request_context, meta);
			if (req_res.disposition == Multinet::TerrainSourceRequestDisposition::CreatedPending ||
			    req_res.disposition == Multinet::TerrainSourceRequestDisposition::ExistingPending) {
				if (req_res.disposition == Multinet::TerrainSourceRequestDisposition::CreatedPending) {
					++total_source_requests;
				}
				entry.is_admitted = true;
				entry.source_state = Multinet::TerrainSourceState::Pending;
			} else if (req_res.disposition == Multinet::TerrainSourceRequestDisposition::ExistingReady) {
				entry.is_admitted = true;
				entry.source_state = Multinet::TerrainSourceState::Ready;
				entry.source_record = req_res.record;
			}
		}
	}

	// Phase 3c — Priority-Sorted Page Upload Allocation & Texture Upload Staging
	uint32_t total_commits = 0;
	if (snapshot_valid && terrain_source) {
		struct ReadyUploadCandidate {
			size_t table_idx;
			TerrainRenderBlockKey key;
			Multinet::TerrainRequestClass request_class;
			uint8_t lod;
			int64_t dist_sq_m;
			uint8_t rank;
			bool selected{ false };
		};

		thread_local static std::array<ReadyUploadCandidate, BlockClipmapLimits::MAX_FRAME_DEMAND> ready_queue;
		uint32_t ready_queue_count = 0;

		for (size_t i = 0; i < demand_table.count; ++i) {
			if (demand_table.entries[i].source_state == Multinet::TerrainSourceState::Ready) {
				const auto& key = demand_table.entries[i].key;
				uint8_t lod = key.lod;
				TerrainGpuPageIdentity entry_id;
				entry_id.key = key;
				entry_id.sample_patch = demand_table.entries[i].request_context.identity.sample_patch;
				entry_id.page_contract_version = demand_table.entries[i].source_record.page_contract_version;
				entry_id.payload_kind = demand_table.entries[i].source_record.payload_kind;
				entry_id.terrain_version = demand_table.entries[i].source_record.terrain_version;
				entry_id.source_version = demand_table.entries[i].source_record.source_version;
				entry_id.block_delta_content_version = demand_table.entries[i].source_record.block_delta_content_version;

				bool already_allocated = false;
				for (uint32_t j = 1; j < 128; ++j) {
					const auto& slot = levels[lod].slots[j];
					if ((slot.state == TerrainGpuPageState::Resident || slot.state == TerrainGpuPageState::UploadPending) &&
					    exact_page_identity_match(slot.get_identity(), entry_id)) {
						already_allocated = true;
						demand_table.entries[i].resident_gpu_layer = j;
						break;
					}
				}
				if (!already_allocated && ready_queue_count < BlockClipmapLimits::MAX_FRAME_DEMAND) {
					ReadyUploadCandidate ruc;
					ruc.table_idx = i;
					ruc.key = key;
					ruc.request_class = demand_table.entries[i].request_class;
					ruc.lod = lod;
					ruc.dist_sq_m = demand_table.entries[i].dist_sq_m;
					ruc.rank = Multinet::admission_rank(ruc.request_class);
					ready_queue[ready_queue_count++] = ruc;
				}
			}
		}

		std::sort(ready_queue.begin(), ready_queue.begin() + ready_queue_count, [](const ReadyUploadCandidate& a, const ReadyUploadCandidate& b) {
			if (a.rank != b.rank) return a.rank < b.rank;
			if (a.dist_sq_m != b.dist_sq_m) return a.dist_sq_m < b.dist_sq_m; // Physical distance first!
			if (a.lod != b.lod) return a.lod > b.lod; // Coarser LOD tie-breaker
			if (a.key.face != b.key.face) return static_cast<uint8_t>(a.key.face) < static_cast<uint8_t>(b.key.face);
			if (a.key.block_u != b.key.block_u) return a.key.block_u < b.key.block_u;
			return a.key.block_v < b.key.block_v;
		});

		// 1. Per-LOD reservation: reserve 2 upload slots per active LOD if ready candidates exist
		std::array<size_t, BlockClipmapLimits::MAX_PAGE_COMMITS> selected_uploads;
		uint32_t selected_uploads_count = 0;
		std::array<uint32_t, BlockClipmapProfile::MAX_LEVELS> lod_reserved{};

		for (uint32_t r_idx = 0; r_idx < ready_queue_count; ++r_idx) {
			auto& ruc = ready_queue[r_idx];
			if (selected_uploads_count >= limits_.max_page_commits) break;
			if (lod_reserved[ruc.lod] < 2) {
				lod_reserved[ruc.lod]++;
				ruc.selected = true;
				selected_uploads[selected_uploads_count++] = ruc.table_idx;
			}
		}

		// 2. Fill remaining budget up to limits_.max_page_commits (24) by priority rank
		for (uint32_t r_idx = 0; r_idx < ready_queue_count; ++r_idx) {
			auto& ruc = ready_queue[r_idx];
			if (selected_uploads_count >= limits_.max_page_commits) break;
			if (!ruc.selected) {
				ruc.selected = true;
				selected_uploads[selected_uploads_count++] = ruc.table_idx;
			}
		}

		for (uint32_t u_idx = 0; u_idx < selected_uploads_count; ++u_idx) {
			auto& entry = demand_table.entries[selected_uploads[u_idx]];
			uint8_t lod = entry.key.lod;

			TerrainGpuPageIdentity entry_id;
			entry_id.key = entry.key;
			entry_id.sample_patch = entry.request_context.identity.sample_patch;
			entry_id.page_contract_version = entry.source_record.page_contract_version;
			entry_id.payload_kind = entry.source_record.payload_kind;
			entry_id.terrain_version = entry.source_record.terrain_version;
			entry_id.source_version = entry.source_record.source_version;
			entry_id.block_delta_content_version = entry.source_record.block_delta_content_version;

			bool already_allocated = false;
			for (uint32_t j = 1; j < 128; ++j) {
				const auto& slot = levels[lod].slots[j];
				if ((slot.state == TerrainGpuPageState::Resident || slot.state == TerrainGpuPageState::UploadPending) &&
				    exact_page_identity_match(slot.get_identity(), entry_id)) {
					already_allocated = true;
					entry.resident_gpu_layer = j;
					break;
				}
			}

			if (!already_allocated) {
				if (total_commits >= limits_.max_page_commits) {
					continue;
				}
				uint32_t free_slot = 0;
				for (uint32_t j = 1; j < 128; ++j) {
					if (levels[lod].slots[j].state == TerrainGpuPageState::Free) {
						free_slot = j;
						break;
					}
				}

				if (free_slot == 0) {
					auto is_slot_wanted_this_frame = [&](const TerrainRenderBlockKey& slot_key, const TerrainSamplePatchKey& slot_patch) -> bool {
						return (demand_table.find(slot_key, slot_patch) != nullptr);
					};

					uint32_t lru_slot = 0;
					uint64_t oldest_frame = UINT64_MAX;
					for (uint32_t j = 1; j < 128; ++j) {
						const auto& slot = levels[lod].slots[j];
						if (slot.state == TerrainGpuPageState::Resident &&
						    !is_slot_wanted_this_frame(slot.key, slot.sample_patch) &&
						    slot.last_referenced_frame < oldest_frame) {
							oldest_frame = slot.last_referenced_frame;
							lru_slot = j;
						}
					}
					if (lru_slot != 0) {
						auto& evict_slot = levels[lod].slots[lru_slot];
						evict_slot.state = TerrainGpuPageState::Retiring;
						evict_slot.retire_after_frame = render_frame_id + RING_BUFFER_SIZE;
					}
				}

				if (free_slot != 0) {
					Multinet::TerrainHeightPage page;
					if (terrain_source->try_read_page(entry.source_record.cpu_page_handle, entry.source_record.cpu_page_generation, page)) {
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

						float p_min = 1e9f, p_max = -1e9f;
						for (float s : page.heights) {
							if (s < p_min) p_min = s;
							if (s > p_max) p_max = s;
						}
						if (p_min > p_max) { p_min = 0.0f; p_max = 0.0f; }

						auto& slot = levels[lod].slots[free_slot];
						slot.state = TerrainGpuPageState::UploadPending;
						slot.cpu_page_handle = entry.source_record.cpu_page_handle;
						slot.cpu_page_generation = entry.source_record.cpu_page_generation;
						slot.terrain_version = entry.source_record.terrain_version;
						slot.source_version = entry.source_record.source_version;
						slot.page_contract_version = entry.source_record.page_contract_version;
						slot.payload_kind = entry.source_record.payload_kind;
						slot.committed_delta_version = entry.source_record.committed_delta_version;
						slot.block_delta_content_version = entry.source_record.block_delta_content_version;
						slot.minimum_sample_m = p_min;
						slot.maximum_sample_m = p_max;
						slot.is_fallback = false;
						slot.key = entry.key;
						slot.sample_patch = entry.request_context.identity.sample_patch;
						slot.last_referenced_frame = render_frame_id;

						TerrainUpdateResult::TextureUpload& upload =
							result.texture_uploads[result.texture_upload_count++];
						upload.lod = lod;
						upload.gpu_layer = free_slot;
						upload.staging_index = staging_idx;
						upload.canonical_key = entry.key;
						upload.sample_patch = entry.request_context.identity.sample_patch;
						upload.cpu_page_handle = entry.source_record.cpu_page_handle;
						upload.cpu_page_generation = entry.source_record.cpu_page_generation;
						upload.terrain_version = entry.source_record.terrain_version;
						upload.source_version = entry.source_record.source_version;
						upload.page_contract_version = entry.source_record.page_contract_version;
						upload.payload_kind = entry.source_record.payload_kind;
						upload.committed_delta_version = entry.source_record.committed_delta_version;
						upload.block_delta_content_version = entry.source_record.block_delta_content_version;

						entry.resident_gpu_layer = free_slot;
						++total_commits;
					}
				}
			}
		}
	}

	// Phase 4 — Single-Pass Coverage Ownership Resolution & FrameTerrainSubmissionPlan Construction
	struct ResolutionResult {
		ResolutionClass res_class{ ResolutionClass::NoContent };
		uint32_t layer{ 0 };
		bool exact_resident{ false };
		bool exact_ready_empty{ false };
		bool stale_previous{ false };
		uint32_t selected_block_content_version{ 1 };
	};

	auto resolve_layer = [this, &scale, &publication](uint8_t lod, const Multinet::TerrainPageRequestContext& req_ctx, const Multinet::TerrainSourceRecord& frame_rec) -> ResolutionResult {
		ResolutionResult res{};
		res.layer = 0;

		if (source_mode == TerrainSourceMode::AnalyticBase) {
			res.res_class = ResolutionClass::Analytic;
			return res;
		}

		if (source_mode == TerrainSourceMode::HybridAdditiveDelta) {
			bool is_ready_empty = (frame_rec.state == Multinet::TerrainSourceState::ReadyEmpty);
			if (!is_ready_empty) {
				if (!publication.committed_delta.field) {
					is_ready_empty = true;
				} else {
					double req_apron = profile.get_lod_spacing(req_ctx.identity.block_key.lod);
					const bool may_have_delta = req_ctx.identity.sample_patch.is_valid() || (active_domain.is_valid()
						? publication.committed_delta.field->block_may_have_nonzero_delta(req_ctx.identity.block_key, active_domain, profile, req_apron)
						: publication.committed_delta.field->block_may_have_nonzero_delta(req_ctx.identity.block_key, scale, profile, req_apron));
					if (!may_have_delta) {
						is_ready_empty = true;
					}
				}
			}

			if (is_ready_empty) {
				res.layer = 0;
				res.exact_ready_empty = true;
				res.res_class = ResolutionClass::ExactReadyEmpty;
				res.selected_block_content_version = req_ctx.identity.block_content_version;
				return res;
			}

			// Exact Resident match
			uint32_t exact_slot = 0;
			for (uint32_t j = 1; j < 128; ++j) {
				const auto& slot = levels[lod].slots[j];
				if (slot.state == TerrainGpuPageState::Resident &&
				    slot.key == req_ctx.identity.block_key &&
				    slot.sample_patch == req_ctx.identity.sample_patch &&
				    slot.page_contract_version == req_ctx.identity.page_contract_version &&
				    slot.payload_kind == req_ctx.identity.payload_kind &&
				    slot.terrain_version == req_ctx.identity.terrain_version &&
				    slot.source_version == req_ctx.identity.source_version &&
				    slot.block_delta_content_version == req_ctx.identity.block_content_version)
				{
					exact_slot = j;
					break;
				}
			}

			if (exact_slot != 0) {
				res.layer = exact_slot;
				res.exact_resident = true;
				res.res_class = ResolutionClass::ExactResident;
				res.selected_block_content_version = levels[lod].slots[exact_slot].block_delta_content_version;
				return res;
			}

			// Stale Previous match: highest content version
			uint32_t stale_slot = 0;
			uint32_t highest_stale_ver = 0;
			for (uint32_t j = 1; j < 128; ++j) {
				const auto& slot = levels[lod].slots[j];
				if (slot.state == TerrainGpuPageState::Resident &&
				    slot.key == req_ctx.identity.block_key &&
				    slot.sample_patch == req_ctx.identity.sample_patch &&
				    slot.page_contract_version == req_ctx.identity.page_contract_version &&
				    slot.payload_kind == req_ctx.identity.payload_kind &&
				    slot.terrain_version == req_ctx.identity.terrain_version &&
				    slot.source_version == req_ctx.identity.source_version)
				{
					if (slot.block_delta_content_version > highest_stale_ver) {
						highest_stale_ver = slot.block_delta_content_version;
						stale_slot = j;
					}
				}
			}

			if (stale_slot != 0) {
				res.layer = stale_slot;
				res.stale_previous = true;
				res.res_class = ResolutionClass::StalePrevious;
				res.selected_block_content_version = highest_stale_ver;
				return res;
			}

			res.layer = 0;
			res.res_class = ResolutionClass::NoContent;
			return res;
		} else {
			// AbsoluteHeightPageDebug
			uint32_t debug_slot = 0;
			for (uint32_t j = 1; j < 128; ++j) {
				const auto& slot = levels[lod].slots[j];
				if (slot.state == TerrainGpuPageState::Resident &&
				    slot.key == req_ctx.identity.block_key &&
				    slot.sample_patch == req_ctx.identity.sample_patch &&
				    slot.page_contract_version == req_ctx.identity.page_contract_version &&
				    slot.payload_kind == req_ctx.identity.payload_kind &&
				    slot.terrain_version == req_ctx.identity.terrain_version &&
				    slot.source_version == req_ctx.identity.source_version)
				{
					debug_slot = j;
					break;
				}
			}

			if (debug_slot != 0) {
				res.layer = debug_slot;
				res.exact_resident = true;
				res.res_class = ResolutionClass::AbsoluteResident;
				res.selected_block_content_version = 1;
				return res;
			}

			res.layer = 0;
			res.res_class = ResolutionClass::AbsoluteAnalyticFallback;
			return res;
		}
	};

	thread_local static std::array<std::array<SubmittedInstance, BlockClipmapProfile::MAX_CANDIDATES>, BlockClipmapProfile::MAX_LEVELS> submitted_lod_instances;
	std::array<uint32_t, BlockClipmapProfile::MAX_LEVELS> submitted_lod_instance_counts{};

	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		for (uint32_t c_idx = 0; c_idx < lod_candidate_counts[lod]; ++c_idx) {
			const auto& cand = lod_candidates[lod][c_idx];
			if (!cand.is_visible) continue;

			FrameDemandEntry* demand_entry = demand_table.find(cand.key, cand.sample_patch);
			Multinet::TerrainPageRequestContext req_ctx = demand_entry
				? demand_entry->request_context
				: (cand.sample_patch.is_valid()
					? Multinet::make_page_request_context(cand.key, cand.sample_patch, profile, publication, scale)
					: Multinet::make_page_request_context(cand.key, profile, publication, scale));
			Multinet::TerrainSourceRecord frame_rec = demand_entry ? demand_entry->source_record : Multinet::TerrainSourceRecord{};

			ResolutionResult resolved = resolve_layer(lod, req_ctx, frame_rec);
			SubmittedInstance inst;
			inst.key = cand.key;
			inst.presentation_key = cand.presentation_key;
			inst.sample_patch = cand.sample_patch;
			inst.block_to_active_frame = cand.block_to_active_frame;
			inst.local_origin = cand.local_origin;
			inst.local_aabb = cand.local_aabb;
			inst.gpu_layer = resolved.layer;
			inst.edge_mask = cand.raw_edge_mask;
			inst.is_coverage_parent = false;
			inst.page_contract_version = req_ctx.identity.page_contract_version;
			inst.payload_kind = req_ctx.identity.payload_kind;
			inst.requested_block_content_version = req_ctx.identity.block_content_version;
			inst.block_delta_content_version = resolved.selected_block_content_version;
			inst.resolution_class = resolved.res_class;

			if (submitted_lod_instance_counts[lod] < BlockClipmapProfile::MAX_CANDIDATES) {
				submitted_lod_instances[lod][submitted_lod_instance_counts[lod]++] = inst;
			}
			if (resolved.layer != 0) {
				levels[lod].slots[resolved.layer].last_referenced_frame = render_frame_id;
			}
		}
	}

	// Phase 5 — Build MultiMesh instance buffers and populate submission plan
	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		const double spacing = profile.get_lod_spacing(lod);
		uint32_t vis_count = 0;
		float* ptr = local_upload_buffer.data();

		for (uint32_t i = 0; i < submitted_lod_instance_counts[lod]; ++i) {
			const auto& inst = submitted_lod_instances[lod][i];
			if (vis_count >= BlockClipmapProfile::MAX_CANDIDATES) break;

			godot::Basis basis = inst.block_to_active_frame.scaled(
				godot::Vector3(static_cast<float>(spacing), 1.0f, static_cast<float>(spacing))
			);

			size_t base = vis_count * 16;
			ptr[base +  0] = basis.rows[0].x;
			ptr[base +  1] = basis.rows[1].x;
			ptr[base +  2] = basis.rows[2].x;
			ptr[base +  3] = inst.local_origin.x;

			ptr[base +  4] = basis.rows[0].y;
			ptr[base +  5] = basis.rows[1].y;
			ptr[base +  6] = basis.rows[2].y;
			ptr[base +  7] = inst.local_origin.y;

			ptr[base +  8] = basis.rows[0].z;
			ptr[base +  9] = basis.rows[1].z;
			ptr[base + 10] = basis.rows[2].z;
			ptr[base + 11] = inst.local_origin.z;

			const bool uses_sample_patch = inst.sample_patch.is_valid();
			const bool uses_coherent_unfolding = is_coherent_sample_patch(inst.sample_patch);
			const bool uses_logical_chart = is_logical_chart_sample_patch(inst.sample_patch);
			const bool uses_bounded_logical_chart = is_bounded_logical_chart_sample_patch(inst.sample_patch);
			// Sampling may be rooted on another face; tinting follows this block's
			// canonical owner so adjacent faces remain visible at a crossing.
			uint32_t face_bits = static_cast<uint32_t>(
				uses_sample_patch ? inst.sample_patch.anchor_face : inst.key.face) & 0x7u;
			uint32_t color_face_bits = (static_cast<uint32_t>(inst.key.face) & 0x7u) << 13u;
			uint32_t edge_bits = (static_cast<uint32_t>(inst.edge_mask) & 0xFu) << 3u;
			uint32_t orientation_bits = uses_sample_patch
				? (static_cast<uint32_t>(sample_patch_orientation(inst.sample_patch)) & 0x3u) << 7u
				: 0u;
			uint32_t patch_bit = uses_sample_patch ? (1u << 9u) : 0u;
			uint32_t coherent_bit = uses_coherent_unfolding ? (1u << 10u) : 0u;
			uint32_t logical_chart_bit = uses_logical_chart ? (1u << 11u) : 0u;
			uint32_t bounded_logical_chart_bit = uses_bounded_logical_chart ? (1u << 12u) : 0u;
			uint32_t camera_relative_render_bit =
				has_active_presentation_binding ? (1u << 16u) : 0u;
			float r_packed = static_cast<float>(
				face_bits | color_face_bits | edge_bits | orientation_bits | patch_bit | coherent_bit |
				logical_chart_bit | bounded_logical_chart_bit | camera_relative_render_bit);

			uint32_t mode_bits = static_cast<uint32_t>(source_mode) & 0x3u;
			uint32_t layer_bits = (static_cast<uint32_t>(inst.gpu_layer) & 0x7Fu) << 2u;
			float g_packed = static_cast<float>(mode_bits | layer_bits);

			ptr[base + 12] = r_packed;
			ptr[base + 13] = g_packed;
			ptr[base + 14] = uses_coherent_unfolding
				? static_cast<float>(inst.presentation_key.block_u)
				: uses_sample_patch
				? static_cast<float>(static_cast<double>(inst.sample_patch.anchor_u_mm) * 0.001)
				: static_cast<float>(inst.key.block_u);
			ptr[base + 15] = uses_coherent_unfolding
				? static_cast<float>(inst.presentation_key.block_v)
				: uses_sample_patch
				? static_cast<float>(static_cast<double>(inst.sample_patch.anchor_v_mm) * 0.001)
				: static_cast<float>(inst.key.block_v);

			if (last_submission_plan.lods[lod].count < BlockClipmapProfile::MAX_CANDIDATES) {
				last_submission_plan.lods[lod].instances[last_submission_plan.lods[lod].count++] = inst;
			}

			if (inst.gpu_layer == 0) {
				if (lod < 7) {
					last_submission_plan.lods[lod].lod_0_6_layer_zero_count++;
					last_streaming_diagnostics.lod_0_6_layer_zero_instances++;
				} else {
					last_submission_plan.lods[lod].lod_7_layer_zero_count++;
					last_streaming_diagnostics.lod_7_layer_zero_instances++;
				}
			}

			levels[lod].pending_visible_diagnostics[vis_count] = { inst.key, inst.gpu_layer };
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
		levels[lod].last_visible_count = vis_count;
		levels[lod].submitted_visible_count = vis_count;
		std::copy_n(
			levels[lod].pending_visible_diagnostics.begin(),
			vis_count,
			levels[lod].submitted_visible_diagnostics.begin()
		);
	}

	uint32_t frame_buffers_rewritten = 0;
	size_t frame_bytes_uploaded = 0;
	uint32_t total_instances = 0;

	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		const uint32_t vis_count = result.lods[lod].visible_count;
		const bool changed = result.lods[lod].buffer_changed;
		total_instances += vis_count;

		frame_cut_diag.lods[lod].submitted_instance_count = vis_count;
		frame_cut_diag.lods[lod].instance_buffer_changed = changed;

		if (changed) {
			frame_buffers_rewritten++;
			total_multimesh_buffer_rewrites_++;
			constexpr size_t FULL_FLOAT_COUNT = BlockClipmapProfile::MAX_CANDIDATES * 16;
			const size_t bytes = FULL_FLOAT_COUNT * sizeof(float);
			frame_cut_diag.lods[lod].instance_bytes_uploaded = bytes;
			frame_bytes_uploaded += bytes;
			cumulative_instance_bytes_uploaded_ += bytes;
		}
	}

	frame_cut_diag.total_instances_submitted = total_instances;
	frame_cut_diag.multimesh_buffers_rewritten = frame_buffers_rewritten;
	frame_cut_diag.total_instance_bytes_uploaded = frame_bytes_uploaded;
	last_cut_diagnostics_ = frame_cut_diag;

	// Apply slot retirement for blocks whose submitted instance resolved to ExactResident or ExactReadyEmpty
	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		for (uint32_t i = 0; i < last_submission_plan.lods[lod].count; ++i) {
			const auto& inst = last_submission_plan.lods[lod].instances[i];
			if (inst.resolution_class == ResolutionClass::ExactResident ||
			    inst.resolution_class == ResolutionClass::ExactReadyEmpty)
			{
				for (uint32_t j = 1; j < 128; ++j) {
					auto& slot = levels[lod].slots[j];
					if ((slot.state == TerrainGpuPageState::Resident || slot.state == TerrainGpuPageState::UploadPending) &&
					    slot.key == inst.key &&
					    slot.sample_patch == inst.sample_patch &&
					    j != inst.gpu_layer)
					{
						slot.state = TerrainGpuPageState::Retiring;
						slot.retire_after_frame = render_frame_id + RING_BUFFER_SIZE;
					}
				}
			}
		}
	}

	if (terrain_source && prewarm_active) {
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
	Multinet::TerrainRenderSource* terrain_source,
	const multinet::rendering::chp::CurvedHorizonView* chp_view,
	double delta_seconds
) {

	if (!is_initialized) return;
	if (!cam_state.is_visible) return;
	godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
	if (!rs) return;

	if (active_domain.is_valid() && !active_domain.is_finite() && cam_state.has_presentation_position) {
		const Multinet::SurfaceFrame& root = cam_state.has_unfolding_root
			? cam_state.unfolding_root_frame : cam_state.active_frame;
		const double root_presentation_x = cam_state.has_unfolding_root
			? cam_state.unfolding_root_presentation_x_m : cam_state.presentation_x_m;
		const double root_presentation_z = cam_state.has_unfolding_root
			? cam_state.unfolding_root_presentation_z_m : cam_state.presentation_z_m;
		TerrainSamplePatchKey encoded_root{};
		LogicalSampleChart logical_chart{};
		if (cam_state.has_logical_chart) {
			logical_chart.root_direction = cam_state.logical_chart_root_direction;
			logical_chart.presentation_x_angular_tangent = cam_state.logical_chart_presentation_x_tangent;
			logical_chart.presentation_z_angular_tangent = cam_state.logical_chart_presentation_z_tangent;
		}
		if (try_encode_sample_patch(root, 0, ORDINARY_BCCM_V1_PROFILE, encoded_root) &&
			(cam_state.has_logical_chart || try_build_logical_sample_chart(root, active_domain, logical_chart))) {
			const bool chart_changed = !has_bound_logical_chart_root_ ||
				encoded_root != bound_logical_chart_root_ ||
				root_presentation_x != bound_logical_chart_root_presentation_x_m_ ||
				root_presentation_z != bound_logical_chart_root_presentation_z_m_;
			if (chart_changed) {
				const uint32_t orientation = sample_patch_orientation(encoded_root);
				const godot::Vector3 root_direction(
					static_cast<float>(logical_chart.root_direction.x),
					static_cast<float>(logical_chart.root_direction.y),
					static_cast<float>(logical_chart.root_direction.z));
				const godot::Vector3 presentation_x_tangent(
					static_cast<float>(logical_chart.presentation_x_angular_tangent.x),
					static_cast<float>(logical_chart.presentation_x_angular_tangent.y),
					static_cast<float>(logical_chart.presentation_x_angular_tangent.z));
				const godot::Vector3 presentation_z_tangent(
					static_cast<float>(logical_chart.presentation_z_angular_tangent.x),
					static_cast<float>(logical_chart.presentation_z_angular_tangent.y),
					static_cast<float>(logical_chart.presentation_z_angular_tangent.z));
				if (cam_state.has_logical_chart) {
					// Analytic V5 moves its root every observer tick. Shared globals turn
					// that into four writes, not nine writes for every active LOD.
					set_v5_chart_globals(rs, root_presentation_x, root_presentation_z,
						root_direction, presentation_x_tangent, presentation_z_tangent);
				} else {
					for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
						godot::RID material = levels[lod].material_rid;
						if (!material.is_valid()) continue;
						rs->material_set_param(material, "unfolding_root_face", static_cast<uint32_t>(root.origin.face));
						rs->material_set_param(material, "unfolding_root_u_m", static_cast<float>(root.origin.u_m));
						rs->material_set_param(material, "unfolding_root_v_m", static_cast<float>(root.origin.v_m));
						rs->material_set_param(material, "unfolding_root_orientation", orientation);
						rs->material_set_param(material, "unfolding_root_presentation_x_m", static_cast<float>(root_presentation_x));
						rs->material_set_param(material, "unfolding_root_presentation_z_m", static_cast<float>(root_presentation_z));
						rs->material_set_param(material, "logical_chart_root_direction", root_direction);
						rs->material_set_param(material, "logical_chart_presentation_x_tangent", presentation_x_tangent);
						rs->material_set_param(material, "logical_chart_presentation_z_tangent", presentation_z_tangent);
					}
				}

				const double area_radius_m = cached_scale_.logical_area_radius_m > 0.0 ? cached_scale_.logical_area_radius_m : scale.logical_area_radius_m;
				const TerrainRootLatticeAnchors anchors = compute_root_anchors(logical_chart.root_direction, area_radius_m, cached_recipe_);

				for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
					godot::RID material = levels[lod].material_rid;
					if (!material.is_valid()) continue;
					rs->material_set_param(material, "terrain_root_cell_0", godot::Vector3i(anchors.cell[0][0], anchors.cell[0][1], anchors.cell[0][2]));
					rs->material_set_param(material, "terrain_root_cell_1", godot::Vector3i(anchors.cell[1][0], anchors.cell[1][1], anchors.cell[1][2]));
					rs->material_set_param(material, "terrain_root_cell_2", godot::Vector3i(anchors.cell[2][0], anchors.cell[2][1], anchors.cell[2][2]));
					rs->material_set_param(material, "terrain_root_cell_3", godot::Vector3i(anchors.cell[3][0], anchors.cell[3][1], anchors.cell[3][2]));
					rs->material_set_param(material, "terrain_root_cell_4", godot::Vector3i(anchors.cell[4][0], anchors.cell[4][1], anchors.cell[4][2]));
					rs->material_set_param(material, "terrain_root_cell_5", godot::Vector3i(anchors.cell[5][0], anchors.cell[5][1], anchors.cell[5][2]));
					rs->material_set_param(material, "terrain_root_cell_6", godot::Vector3i(anchors.cell[6][0], anchors.cell[6][1], anchors.cell[6][2]));
					rs->material_set_param(material, "terrain_root_cell_7", godot::Vector3i(anchors.cell[7][0], anchors.cell[7][1], anchors.cell[7][2]));

					rs->material_set_param(material, "terrain_root_fraction_0", godot::Vector3(anchors.fraction[0][0], anchors.fraction[0][1], anchors.fraction[0][2]));
					rs->material_set_param(material, "terrain_root_fraction_1", godot::Vector3(anchors.fraction[1][0], anchors.fraction[1][1], anchors.fraction[1][2]));
					rs->material_set_param(material, "terrain_root_fraction_2", godot::Vector3(anchors.fraction[2][0], anchors.fraction[2][1], anchors.fraction[2][2]));
					rs->material_set_param(material, "terrain_root_fraction_3", godot::Vector3(anchors.fraction[3][0], anchors.fraction[3][1], anchors.fraction[3][2]));
					rs->material_set_param(material, "terrain_root_fraction_4", godot::Vector3(anchors.fraction[4][0], anchors.fraction[4][1], anchors.fraction[4][2]));
					rs->material_set_param(material, "terrain_root_fraction_5", godot::Vector3(anchors.fraction[5][0], anchors.fraction[5][1], anchors.fraction[5][2]));
					rs->material_set_param(material, "terrain_root_fraction_6", godot::Vector3(anchors.fraction[6][0], anchors.fraction[6][1], anchors.fraction[6][2]));
					rs->material_set_param(material, "terrain_root_fraction_7", godot::Vector3(anchors.fraction[7][0], anchors.fraction[7][1], anchors.fraction[7][2]));
				}
				bound_logical_chart_root_ = encoded_root;
				bound_logical_chart_root_presentation_x_m_ = root_presentation_x;
				bound_logical_chart_root_presentation_z_m_ = root_presentation_z;
				has_bound_logical_chart_root_ = true;
			}
		}
	}

	const bool chp_effective = chp_view && chp_view->chp_effective && cam_state.has_presentation_binding;
	last_bound_chp_gpu_effective_ = chp_effective;
	if (chp_effective) {
		last_bound_chp_camera_altitude_m_ = static_cast<float>(chp_view->camera_surface_height_m);
	}
	const bool is_certified_profile = (profile.candidate_grid_radius == 4 && profile.inner_hole_radius == 2);
	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		godot::RID material = levels[lod].material_rid;
		if (!material.is_valid()) continue;
		rs->material_set_param(material, "lod_spacing", static_cast<float>(profile.get_lod_spacing(lod)));
		rs->material_set_param(material, "lod_block_size", static_cast<float>(profile.get_lod_block_size(lod)));
		rs->material_set_param(material, "parent_morph_enabled", is_certified_profile);
		rs->material_set_param(material, "parent_morph_view_offset_m", godot::Vector2(0.0f, 0.0f));
		rs->material_set_param(material, "current_lod_index", static_cast<uint32_t>(lod));
		rs->material_set_param(material, "active_ordinary_level_count", static_cast<uint32_t>(profile.level_count));
		rs->material_set_param(material, "chp_gpu_effective", chp_effective);
		if (chp_effective) {
			rs->material_set_param(material, "chp_function_class", static_cast<uint32_t>(chp_view->profile.requested.function_class));
			rs->material_set_param(material, "chp_radius_m", static_cast<float>(chp_view->profile.radius_m));
			rs->material_set_param(material, "chp_inverse_radius", static_cast<float>(chp_view->profile.inverse_radius));
			rs->material_set_param(material, "chp_inverse_radius_squared", static_cast<float>(chp_view->profile.inverse_radius_squared));
			rs->material_set_param(material, "chp_camera_altitude_m", static_cast<float>(chp_view->camera_surface_height_m));
			rs->material_set_param(material, "chp_certified_max_distance_m", static_cast<float>(chp_view->profile.certified_maximum_deformation_distance_m));
			rs->material_set_param(material, "chp_certified_max_u", static_cast<float>(chp_view->profile.certified_maximum_u));
			rs->material_set_param(material, "chp_debug_reconstruction_mode", chp_debug_reconstruction_mode);
			rs->material_set_param(material, "chp_debug_negative_height_color", chp_debug_negative_height_color);
			rs->material_set_param(material, "chp_debug_negative_height_exaggeration", chp_debug_negative_height_exaggeration);
		}
		rs->material_set_param(material, "bccm_debug_visual_mode", bccm_debug_visual_mode);
	}

	TerrainUpdateResult result = compute_update(p_camera_world_position, p_frustum, scale, cam_state, expectation, terrain_source, chp_view, delta_seconds);

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
		slot.sample_patch = upload.sample_patch;
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
		level.submitted_buffer_index = static_cast<uint8_t>(frame_index);
		
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
	Multinet::TerrainRenderSource* terrain_source,
	const multinet::rendering::chp::CurvedHorizonView* chp_view,
	double delta_seconds
) {
	if (!p_camera) return;
	godot::Vector3 cam_pos = p_camera->get_global_position();
	FrustumPlanes frustum = FrustumPlanes::extract_from_camera(p_camera);
	update_with_view(cam_pos, frustum, scale, cam_state, expectation, terrain_source, chp_view, delta_seconds);
}
#else
void BlockClipmapRenderer::update_with_view(
	const godot::Vector3& p_camera_world_position,
	const FrustumPlanes& p_frustum,
	const Multinet::WorldScaleManifest& scale,
	const BCCMCameraState& cam_state,
	const BCCMSourceExpectation& expectation,
	Multinet::TerrainRenderSource* terrain_source,
	const multinet::rendering::chp::CurvedHorizonView* chp_view,
	double delta_seconds
) {}

void BlockClipmapRenderer::update(
	godot::Camera3D* p_camera,
	const Multinet::WorldScaleManifest& scale,
	const BCCMCameraState& cam_state,
	const BCCMSourceExpectation& expectation,
	Multinet::TerrainRenderSource* terrain_source,
	const multinet::rendering::chp::CurvedHorizonView* chp_view,
	double delta_seconds
) {}
#endif


BlockPlacement BlockClipmapRenderer::build_block_placement(
	const TerrainRenderBlockKey& canonical_key,
	const Multinet::SurfaceFrame& active_frame,
	const Multinet::WorldScaleManifest& manifest,
	const Multinet::TerrainFallbackBounds& bounds,
	const Multinet::TerrainCommittedDeltaSnapshot* delta_snapshot,
	const Multinet::TerrainFallbackBounds* stale_bounds
) const {
	double block_size = profile.get_lod_block_size(canonical_key.lod);

	Multinet::SurfacePosition64 min_corner;
	min_corner.face = canonical_key.face;
	min_corner.u_m = canonical_key.block_u * block_size;
	min_corner.v_m = canonical_key.block_v * block_size;
	min_corner.altitude_m = 0.0;
	min_corner.topology_version = active_frame.topology_version;
	min_corner.projection_version = active_frame.projection_version;

	Multinet::FramePosition64 f_min{};
	Multinet::FramePosition64 fu{};
	Multinet::FramePosition64 fv_fp{};
	Multinet::FramePosition64 fh{};
	const auto to_frame = [this, &active_frame, &manifest](const Multinet::SurfacePosition64& position, Multinet::FramePosition64& out) {
		return active_domain.is_valid()
			? Multinet::try_domain_surface_to_frame(position, active_frame, active_domain, out)
			: Multinet::try_surface_to_frame(position, active_frame, manifest, out);
	};
	if (active_domain.is_valid() && active_domain.is_finite()) {
		// Partially intersecting edge blocks can start outside the rectangle.
		// Their vertices are clipped in the shader, so placement must remain a
		// plain finite-frame translation instead of rejecting the block corner.
		f_min = {
			min_corner.u_m - active_frame.origin.u_m,
			min_corner.altitude_m - active_frame.origin.altitude_m,
			min_corner.v_m - active_frame.origin.v_m
		};
		fu = { f_min.x + 1.0, f_min.y, f_min.z };
		fv_fp = { f_min.x, f_min.y, f_min.z + 1.0 };
		fh = { f_min.x, f_min.y + 1.0, f_min.z };
	} else {
		if (!to_frame(min_corner, f_min)) return BlockPlacement{ {}, {}, {}, false };

		Multinet::SurfacePosition64 pu = min_corner; pu.u_m += 1.0;
		Multinet::SurfacePosition64 pv = min_corner; pv.v_m += 1.0;
		Multinet::SurfacePosition64 ph = min_corner; ph.altitude_m += 1.0;
		if (!to_frame(pu, fu) || !to_frame(pv, fv_fp) || !to_frame(ph, fh)) {
			return BlockPlacement{ {}, {}, {}, false };
		}
	}

	godot::Vector3 vx(fu.x - f_min.x, fu.y - f_min.y, fu.z - f_min.z);
	godot::Vector3 vz(fv_fp.x - f_min.x, fv_fp.y - f_min.y, fv_fp.z - f_min.z);
	godot::Vector3 vy(fh.x - f_min.x, fh.y - f_min.y, fh.z - f_min.z);
	godot::Basis block_basis;
	block_basis.set_column(0, vx);
	block_basis.set_column(1, vy);
	block_basis.set_column(2, vz);

	float base_min = bounds.minimum_height - bounds.residual_bound - bounds.morph_allowance;
	float base_max = bounds.maximum_height + bounds.residual_bound + bounds.morph_allowance;

	float delta_min = 0.0f;
	float delta_max = 0.0f;

	if (delta_snapshot && delta_snapshot->contract_version > 0) {
		delta_min = std::min(delta_min, delta_snapshot->minimum_delta_m);
		delta_max = std::max(delta_max, delta_snapshot->maximum_delta_m);
	}

	if (stale_bounds) {
		delta_min = std::min(delta_min, stale_bounds->minimum_height);
		delta_max = std::max(delta_max, stale_bounds->maximum_height);
	}

	if (canonical_key.lod < profile.level_count) {
		for (uint32_t j = 1; j < 128; ++j) {
			const auto& slot = levels[canonical_key.lod].slots[j];
			if ((slot.state == TerrainGpuPageState::Resident || slot.state == TerrainGpuPageState::UploadPending) &&
			    slot.key == canonical_key) {
				delta_min = std::min(delta_min, slot.minimum_sample_m);
				delta_max = std::max(delta_max, slot.maximum_sample_m);
			}
		}
	}

	float lower = base_min + delta_min;
	float upper = base_max + delta_max;

	godot::AABB aabb(
		godot::Vector3(0.0f, lower, 0.0f),
		godot::Vector3(static_cast<float>(block_size), std::max(0.001f, upper - lower), static_cast<float>(block_size))
	);

	godot::Vector3 local_origin(static_cast<float>(f_min.x), static_cast<float>(f_min.y), static_cast<float>(f_min.z));
	if (has_active_presentation_binding) {
		block_basis = active_presentation_basis * block_basis;
		// Editor placement is camera-relative for finite worlds as well as closed
		// worlds. Keep the canonical block coordinate in double precision until
		// the final small relative translation reaches the MultiMesh buffer.
		local_origin = (active_presentation_origin - active_view_world_position) +
			active_presentation_basis.xform(local_origin);
	}

	return BlockPlacement{ block_basis, local_origin, aabb, true };
}

BlockPlacement BlockClipmapRenderer::build_presentation_block_placement(
	const TerrainPresentationBlockKey& presentation_key,
	const TerrainRenderBlockKey& canonical_key,
	double observer_presentation_x_m,
	double observer_presentation_z_m,
	const Multinet::TerrainFallbackBounds& bounds,
	const Multinet::TerrainCommittedDeltaSnapshot* delta_snapshot,
	const Multinet::TerrainFallbackBounds* stale_bounds
) const {
	if (!presentation_key.is_valid() || !canonical_key.is_valid() ||
		presentation_key.lod != canonical_key.lod ||
		!std::isfinite(observer_presentation_x_m) || !std::isfinite(observer_presentation_z_m)) {
		return BlockPlacement{ {}, {}, {}, false };
	}

	const double block_size = profile.get_lod_block_size(presentation_key.lod);
	const double min_x = static_cast<double>(presentation_key.block_u) * block_size - observer_presentation_x_m;
	const double min_z = static_cast<double>(presentation_key.block_v) * block_size - observer_presentation_z_m;
	if (!std::isfinite(block_size) || !(block_size > 0.0) ||
		!std::isfinite(min_x) || !std::isfinite(min_z)) {
		return BlockPlacement{ {}, {}, {}, false };
	}

	float base_min = bounds.minimum_height - bounds.residual_bound - bounds.morph_allowance;
	float base_max = bounds.maximum_height + bounds.residual_bound + bounds.morph_allowance;
	float delta_min = 0.0f;
	float delta_max = 0.0f;
	if (delta_snapshot && delta_snapshot->contract_version > 0) {
		delta_min = std::min(delta_min, delta_snapshot->minimum_delta_m);
		delta_max = std::max(delta_max, delta_snapshot->maximum_delta_m);
	}
	if (stale_bounds) {
		delta_min = std::min(delta_min, stale_bounds->minimum_height);
		delta_max = std::max(delta_max, stale_bounds->maximum_height);
	}
	if (canonical_key.lod < profile.level_count) {
		for (uint32_t j = 1; j < 128; ++j) {
			const auto& slot = levels[canonical_key.lod].slots[j];
			if ((slot.state == TerrainGpuPageState::Resident || slot.state == TerrainGpuPageState::UploadPending) &&
				slot.key == canonical_key) {
				delta_min = std::min(delta_min, slot.minimum_sample_m);
				delta_max = std::max(delta_max, slot.maximum_sample_m);
			}
		}
	}

	const float lower = base_min + delta_min;
	const float upper = base_max + delta_max;
	godot::AABB aabb(
		godot::Vector3(0.0f, lower, 0.0f),
		godot::Vector3(static_cast<float>(block_size), std::max(0.001f, upper - lower), static_cast<float>(block_size))
	);
	godot::Basis block_basis{};
	godot::Vector3 local_origin(static_cast<float>(min_x), 0.0f, static_cast<float>(min_z));
	if (has_active_presentation_binding) {
		block_basis = active_presentation_basis * block_basis;
		// Closed editor rendering must stay camera-relative all the way into the
		// vertex transform. Reintroducing huge world coordinates here makes the
		// single-precision view matrix round every tile separately.
		local_origin = (active_presentation_origin - active_view_world_position) +
			active_presentation_basis.xform(local_origin);
	}
	return BlockPlacement{ block_basis, local_origin, aabb, true };
}

#ifdef DEBUG_ENABLED
DebugBlockReplacementState BlockClipmapRenderer::get_debug_block_state(const TerrainRenderBlockKey& key) const {
	DebugBlockReplacementState out{};
	out.key = key;
	if (key.lod >= profile.level_count) return out;

	for (uint32_t i = 0; i < last_submission_plan.lods[key.lod].count; ++i) {
		const SubmittedInstance& instance = last_submission_plan.lods[key.lod].instances[i];
		if (instance.key == key) {
			out.submitted = true;
			out.selected_gpu_layer = instance.gpu_layer;
			out.resolution_class = instance.resolution_class;
			out.requested_content_version = instance.requested_block_content_version;
			out.selected_content_version = instance.block_delta_content_version;
			break;
		}
	}

	const auto& slots = levels[key.lod].slots;
	for (uint32_t i = 0; i < 128; ++i) {
		const auto& slot = slots[i];
		if (i != 0 && slot.key == key) {
			if (slot.state == TerrainGpuPageState::Resident) ++out.resident_same_block_count;
			if (slot.state == TerrainGpuPageState::UploadPending) ++out.upload_pending_same_block_count;
			if (slot.state == TerrainGpuPageState::Retiring) {
				++out.retiring_same_block_count;
				if (slot.block_delta_content_version >= out.retiring_content_version) {
					out.retiring_content_version = slot.block_delta_content_version;
					out.retire_after_frame = slot.retire_after_frame;
				}
			}
		}
	}

	if (out.selected_gpu_layer < 128) {
		const auto& selected = slots[out.selected_gpu_layer];
		out.selected_slot_state = selected.state;
		if (out.selected_gpu_layer == 0) {
			out.selected_content_version = selected.block_delta_content_version;
		}
	} else {
		out.selected_slot_state = TerrainGpuPageState::Free;
	}
	return out;
}
#endif

void BlockClipmapRenderer::get_diagnostic_snapshot(RendererDiagnosticSnapshot& out_snap) const {
	out_snap.streaming_diagnostics = last_streaming_diagnostics;
	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		auto& lod_snap = out_snap.lods[lod];
		const auto& level = levels[lod];

		lod_snap.candidate_count = level.last_candidate_count;
		lod_snap.visible_count = level.last_visible_count;
		lod_snap.visible_keys_count = level.last_visible_count;

		uint32_t resident_count = 0;
		for (uint32_t i = 1; i < 128; ++i) {
			if (level.slots[i].state == TerrainGpuPageState::Resident) ++resident_count;
		}
		lod_snap.resident_visible_keys_count = resident_count;
		lod_snap.ring_buffer_float_count = BlockClipmapProfile::MAX_CANDIDATES * 16;
		lod_snap.next_ring_terminal_keys_required = next_ring_terminal_keys_required;
		lod_snap.next_ring_terminal_keys_resident = next_ring_terminal_keys_resident;
		lod_snap.ring_transaction_pending = ring_transaction_pending;
		lod_snap.ring_transaction_age_ms = static_cast<float>(render_frame_id - ring_transaction_start_frame) * 16.666f;

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

		uint32_t safe_cand_count = std::min(level.last_candidate_count, static_cast<uint32_t>(BlockClipmapProfile::MAX_CANDIDATES));
		uint32_t safe_vis_count = std::min(level.submitted_visible_count, static_cast<uint32_t>(BlockClipmapProfile::MAX_CANDIDATES));
		lod_snap.candidate_keys.assign(level.diagnostic_candidate_keys.begin(), level.diagnostic_candidate_keys.begin() + safe_cand_count);
		lod_snap.submitted_visible_diagnostics.assign(level.submitted_visible_diagnostics.begin(), level.submitted_visible_diagnostics.begin() + safe_vis_count);
	}
}

DetailedRendererDiagnostics BlockClipmapRenderer::get_detailed_diagnostics() const {
	DetailedRendererDiagnostics diag{};
	diag.expected_source_version = last_expectation.source_version;
	diag.actual_source_version = last_snapshot.source_version;
	diag.publication_version = last_snapshot.committed_delta_version;
	diag.page_contract_version = last_snapshot.page_contract_version;
	diag.payload_kind = last_snapshot.payload_kind;
	diag.block_content_version = 1;

	diag.source_pending_count = last_source_diagnostics_.source_pending_count;
	diag.source_in_flight_count = last_source_diagnostics_.source_in_flight_count;
	diag.executor_submit_count = last_source_diagnostics_.executor_submit_count;
	diag.incompatible_jobs_cancelled = last_source_diagnostics_.incompatible_jobs_cancelled;
	diag.commit_pending_call_count = last_source_diagnostics_.commit_pending_call_count;
	diag.request_record_call_count = last_source_diagnostics_.request_record_call_count;
	diag.rejected_delta_publication_count = last_source_diagnostics_.rejected_delta_publication_count;

	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		const auto& level = levels[lod];
		for (uint32_t j = 1; j < 128; ++j) {
			const auto& slot = level.slots[j];
			if (slot.state == TerrainGpuPageState::Resident) {
				diag.resident_delta_layers++;
				if (slot.payload_kind == Multinet::TerrainPagePayloadKind::AdditiveHeightDeltaV1) {
					diag.resident_additive_delta_layers++;
				} else if (slot.payload_kind == Multinet::TerrainPagePayloadKind::AbsoluteHeightDebugV1) {
					diag.resident_absolute_debug_layers++;
				}
			} else if (slot.state == TerrainGpuPageState::UploadPending) {
				diag.upload_pending_delta_layers++;
				if (slot.payload_kind == Multinet::TerrainPagePayloadKind::AdditiveHeightDeltaV1) {
					diag.upload_pending_additive_delta_layers++;
				}
			}
		}
		for (uint32_t i = 0; i < level.submitted_visible_count; ++i) {
			const auto& inst = last_submission_plan.lods[lod].instances[i];
			if (source_mode == TerrainSourceMode::AnalyticBase) {
				diag.analytic_base_visible_instances++;
			} else if (source_mode == TerrainSourceMode::HybridAdditiveDelta) {
				diag.hybrid_visible_instances++;
				if (inst.resolution_class == ResolutionClass::ExactResident) {
					diag.hybrid_exact_resident_instances++;
				} else if (inst.resolution_class == ResolutionClass::StalePrevious) {
					diag.hybrid_using_stale_previous_instances++;
					diag.stale_delta_pages_retained++;
				} else if (inst.resolution_class == ResolutionClass::ExactReadyEmpty) {
					diag.hybrid_ready_empty_instances++;
				} else if (inst.gpu_layer == 0) {
					diag.hybrid_zero_delta_instances++;
				}
			} else if (source_mode == TerrainSourceMode::AbsoluteHeightPageDebug) {
				diag.absolute_debug_visible_instances++;
				if (inst.gpu_layer == 0) {
					diag.absolute_debug_missing_page_instances++;
				}
			}
		}
	}
	return diag;
}

} // namespace multinet::rendering

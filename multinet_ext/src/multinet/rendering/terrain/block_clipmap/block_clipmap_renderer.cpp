#include "multinet/rendering/terrain/block_clipmap/block_clipmap_renderer.h"

#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/basis.hpp>

namespace multinet::rendering {

BlockClipmapRenderer::~BlockClipmapRenderer() {
	cleanup();
}

godot::RID BlockClipmapRenderer::create_master_block_mesh() {
	godot::PackedVector3Array vertices;
	godot::PackedVector3Array normals;
	godot::PackedVector2Array uvs;
	godot::PackedInt32Array indices;

	const uint32_t verts_across = BlockClipmapProfile::VERTS_PER_EDGE; // 17
	const uint32_t quads_across = BlockClipmapProfile::QUADS_PER_EDGE; // 16

	vertices.resize(BlockClipmapProfile::TOTAL_VERTS);
	normals.resize(BlockClipmapProfile::TOTAL_VERTS);
	uvs.resize(BlockClipmapProfile::TOTAL_VERTS);

	const float block_size = profile.lod0_block_size; // 32.0m

	for (uint32_t z = 0; z <= quads_across; ++z) {
		for (uint32_t x = 0; x <= quads_across; ++x) {
			uint32_t idx = z * verts_across + x;
			// Master block geometry in local quad coords [0, 16] on XZ plane
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

			// Clockwise winding for Godot front-facing
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

void BlockClipmapRenderer::initialize(godot::RID p_scenario) {
	if (is_initialized) return;

	scenario_rid = p_scenario;
	godot::RenderingServer *rs = godot::RenderingServer::get_singleton();
	
	// Pre-allocate buffer for maximum instances to prevent allocations during bulk MultiMesh upload
	local_upload_buffer.resize(BlockClipmapProfile::MAX_CANDIDATES * 16);
	for (int i = 0; i < BlockClipmapProfile::MAX_LEVELS; ++i) {
		cached_buffers[i].resize(BlockClipmapProfile::MAX_CANDIDATES * 16);
		for (int j = 0; j < RING_BUFFER_SIZE; ++j) {
			multimesh_ring_buffers[i][j].resize(BlockClipmapProfile::MAX_CANDIDATES * 16);
		}
	}

	// 1. Create master block mesh (16x16 quads / 17x17 verts)
	master_mesh_rid = create_master_block_mesh();

	// 2. Create BCCM base shader material
	shader_data = create_bccm_shader_material();
	rs->mesh_surface_set_material(master_mesh_rid, 0, shader_data.material_rid);

	// 3. Allocate MultiMesh and Instance per LOD level
	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		LODLevelData &level = levels[lod];

		level.multimesh_rid = rs->multimesh_create();
		// args: multimesh, instances, transform_format, use_colors, use_custom_data
		rs->multimesh_allocate_data(level.multimesh_rid, BlockClipmapProfile::MAX_CANDIDATES, godot::RenderingServer::MULTIMESH_TRANSFORM_3D, false, true);
		rs->multimesh_set_mesh(level.multimesh_rid, master_mesh_rid);

		// Hide the MultiMesh bounds from Godot's editor so it doesn't pollute the 'F' key zoom bounds
		rs->multimesh_set_custom_aabb(level.multimesh_rid, godot::AABB(godot::Vector3(0, 0, 0), godot::Vector3(0.01f, 0.01f, 0.01f)));

		level.instance_rid = rs->instance_create();
		rs->instance_set_base(level.instance_rid, level.multimesh_rid);
		rs->instance_set_scenario(level.instance_rid, scenario_rid);
		
		// Prevent Godot from culling the entire MultiMeshInstance3D when looking away from origin.
		// We use ignore_culling to disable Godot's culler.
		rs->instance_set_ignore_culling(level.instance_rid, true);
		
		// Disable shadow casting on the terrain by default. Drawing 4 cascades of shadows for procedural terrain 
		// causes the vertex shader to run 6 times per chunk, which annihilates laptop GPUs!
		rs->instance_geometry_set_cast_shadows_setting(level.instance_rid, godot::RenderingServer::SHADOW_CASTING_SETTING_OFF);
		// Crucially, we MUST also set the instance's custom AABB to zero! Otherwise, even with culling ignored,
		// Godot's editor reads the implicit AABB and expands the entire scene bounds to millions of units, breaking the 'F' key!
		rs->instance_set_custom_aabb(level.instance_rid, godot::AABB(godot::Vector3(0, 0, 0), godot::Vector3(0.01f, 0.01f, 0.01f)));
	}

	is_initialized = true;
}

void BlockClipmapRenderer::cleanup() {
	if (!is_initialized) return;

	godot::RenderingServer *rs = godot::RenderingServer::get_singleton();
	
	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		LODLevelData &level = levels[lod];
		if (level.instance_rid.is_valid()) {
			rs->free_rid(level.instance_rid);
			level.instance_rid = godot::RID();
		}
		if (level.multimesh_rid.is_valid()) {
			rs->free_rid(level.multimesh_rid);
			level.multimesh_rid = godot::RID();
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

	is_initialized = false;
}

void BlockClipmapRenderer::update(godot::Camera3D *p_camera, const BCCMTerrainSettings &settings) {
	if (!is_initialized || !p_camera) return;

	godot::RenderingServer *rs = godot::RenderingServer::get_singleton();

	// Only update material parameters if they changed
	if (last_settings != settings) {
		float mapped_freq = settings.continental_frequency * 0.0001f;
		rs->material_set_param(shader_data.material_rid, "terrain_seed", settings.seed);
		rs->material_set_param(shader_data.material_rid, "min_elevation", settings.min_elevation_m);
		rs->material_set_param(shader_data.material_rid, "max_elevation", settings.max_elevation_m);
		rs->material_set_param(shader_data.material_rid, "continental_frequency", mapped_freq);

		for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
			rs->instance_set_visible(levels[lod].instance_rid, settings.is_visible);
		}

		last_settings = settings;
	}

	if (!settings.is_visible) {
		return;
	}

	godot::Vector3 cam_pos = p_camera->get_global_position();
	FrustumPlanes frustum = FrustumPlanes::extract_from_camera(p_camera);

	// Cycle ring buffer index to avoid COW stalls while uploading data
	frame_index = (frame_index + 1) % RING_BUFFER_SIZE;

	for (uint8_t lod = 0; lod < profile.level_count; ++lod) {
		const float block_size = profile.get_lod_block_size(lod);
		const float spacing = profile.get_lod_spacing(lod);

		// The center of LOD L must be snapped to the block size of LOD L+1 to ensure its outer boundary
		// aligns perfectly with the block boundaries of LOD L+1.
		// For the coarsest LOD, we just snap to its own block size.
		const float snap_size = (lod + 1 < profile.level_count) ? profile.get_lod_block_size(lod + 1) : block_size;
		
		int64_t center_world_x = floor_div_f(cam_pos.x, snap_size) * static_cast<int64_t>(snap_size);
		int64_t center_world_z = floor_div_f(cam_pos.z, snap_size) * static_cast<int64_t>(snap_size);

		int64_t center_bx = center_world_x / static_cast<int64_t>(block_size);
		int64_t center_bz = center_world_z / static_cast<int64_t>(block_size);

		int32_t hole_dx = 0;
		int32_t hole_dz = 0;
		
		if (lod > 0) {
			// The hole must perfectly track the active center of LOD L-1
			const float prev_block_size = profile.get_lod_block_size(lod - 1);
			const float prev_snap_size = block_size; // LOD L-1 snapped to LOD L
			int64_t prev_world_x = floor_div_f(cam_pos.x, prev_snap_size) * static_cast<int64_t>(prev_snap_size);
			int64_t prev_world_z = floor_div_f(cam_pos.z, prev_snap_size) * static_cast<int64_t>(prev_snap_size);
			
			hole_dx = static_cast<int32_t>((prev_world_x - center_world_x) / static_cast<int64_t>(block_size));
			hole_dz = static_cast<int32_t>((prev_world_z - center_world_z) / static_cast<int64_t>(block_size));
		}

		int32_t r = profile.candidate_grid_radius; // 8
		int32_t hole_r = profile.inner_hole_radius; // 4
		uint32_t cand_idx = 0;

		// Enumerate 16x16 candidate grid around center block
		for (int32_t dz = -r; dz < r; ++dz) {
			for (int32_t dx = -r; dx < r; ++dx) {
				// Dynamic hole omission tracking the finer level
				if (lod > 0) {
					int32_t hx = dx - hole_dx;
					int32_t hz = dz - hole_dz;
					if (hx >= -hole_r && hx < hole_r && hz >= -hole_r && hz < hole_r) {
						continue;
					}
				}

				if (cand_idx >= BlockClipmapProfile::MAX_CANDIDATES) break;

				int64_t bx = center_bx + dx;
				int64_t bz = center_bz + dz;

				TerrainClipmapBlockState &state = candidate_blocks[cand_idx];
				state.key = TerrainRenderBlockKey{ bx, bz, lod, 0, 0 };
				state.world_origin = godot::Vector3(static_cast<float>(bx) * block_size, 0.0f, static_cast<float>(bz) * block_size);
				state.world_aabb = godot::AABB(
					godot::Vector3(state.world_origin.x, settings.min_elevation_m, state.world_origin.z),
					godot::Vector3(block_size, settings.max_elevation_m - settings.min_elevation_m, block_size)
				);

				uint8_t edge_mask = 0;
				if (dx == r - 1) edge_mask |= 1; // +x
				if (dx == -r) edge_mask |= 2;    // -x
				if (dz == r - 1) edge_mask |= 4; // +z
				if (dz == -r) edge_mask |= 8;    // -z
				state.edge_mask = edge_mask;

				// Perform CPU AABB Frustum Culling
				state.is_visible = frustum.intersects_aabb(state.world_aabb);
				cand_idx++;
			}
		}
		
		LODLevelData &level = levels[lod];
		level.last_candidate_count = cand_idx;

		// Compact visible instances and upload buffer
		uint32_t vis_count = 0;
		float *ptr = local_upload_buffer.data();

		for (uint32_t i = 0; i < level.last_candidate_count; ++i) {
			const TerrainClipmapBlockState &state = candidate_blocks[i];
			if (state.is_visible) {
				// 3D Transform Row 0
				ptr[vis_count * 16 + 0] = spacing;
				ptr[vis_count * 16 + 1] = 0.0f;
				ptr[vis_count * 16 + 2] = 0.0f;
				ptr[vis_count * 16 + 3] = state.world_origin.x;

				// 3D Transform Row 1
				ptr[vis_count * 16 + 4] = 0.0f;
				ptr[vis_count * 16 + 5] = 1.0f;
				ptr[vis_count * 16 + 6] = 0.0f;
				ptr[vis_count * 16 + 7] = state.world_origin.y;

				// 3D Transform Row 2
				ptr[vis_count * 16 + 8] = 0.0f;
				ptr[vis_count * 16 + 9] = 0.0f;
				ptr[vis_count * 16 + 10] = spacing;
				ptr[vis_count * 16 + 11] = state.world_origin.z;

				// Custom data
				ptr[vis_count * 16 + 12] = static_cast<float>(state.edge_mask);
				ptr[vis_count * 16 + 13] = 0.0f;
				ptr[vis_count * 16 + 14] = 0.0f;
				ptr[vis_count * 16 + 15] = 0.0f;

				vis_count++;
			}
		}
		
		// Check if buffer changed to avoid CPU-GPU sync stalls
		bool buffer_changed = false;
		if (vis_count > 0) {
			const uint8_t* new_ptr = reinterpret_cast<const uint8_t*>(local_upload_buffer.data());
			uint8_t* cached_ptr = reinterpret_cast<uint8_t*>(cached_buffers[lod].data());
			
			// Only compare the valid part of the buffer (vis_count * 16 floats * 4 bytes)
			size_t byte_count = vis_count * 16 * sizeof(float);
			if (memcmp(new_ptr, cached_ptr, byte_count) != 0) {
				memcpy(cached_ptr, new_ptr, byte_count);
				buffer_changed = true;
			}
		} else if (level.last_visible_count > 0) {
			buffer_changed = true; // Went from some visible to none
		}

		if (buffer_changed || vis_count != level.last_visible_count) {
			// Bulk upload all visible instances using the ring buffer.
			// Because the array we are writing to is 3 frames old, the Render Server has
			// already released its reference. `ptrw()` will NOT trigger a Copy-On-Write deep copy.
			// This completely eliminates both CPU stalls and VRAM memory leak/fragmentation.
			godot::PackedFloat32Array &upload_array = multimesh_ring_buffers[lod][frame_index];
			if (vis_count > 0) {
				memcpy(upload_array.ptrw(), local_upload_buffer.data(), vis_count * 16 * sizeof(float));
			}
			
			rs->multimesh_set_buffer(level.multimesh_rid, upload_array);
			rs->multimesh_set_visible_instances(level.multimesh_rid, vis_count);
		}
		
		level.last_visible_count = vis_count;
	}
}

} // namespace multinet::rendering

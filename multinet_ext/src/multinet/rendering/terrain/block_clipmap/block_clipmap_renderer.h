#ifndef MULTINET_BLOCK_CLIPMAP_RENDERER_H
#define MULTINET_BLOCK_CLIPMAP_RENDERER_H

#include "multinet/rendering/terrain/block_clipmap/block_clipmap_profile.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_state.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_culling.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_shader.h"

#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/world3d.hpp>

#include <array>

namespace multinet::rendering {

struct BCCMTerrainSettings {
	uint32_t seed{ 1337 };
	float min_elevation_m{ -100.0f };
	float max_elevation_m{ 2000.0f };
	float continental_frequency{ 50.0f };

	bool is_visible{ true };

	bool operator==(const BCCMTerrainSettings &other) const {
		return seed == other.seed && 
			   min_elevation_m == other.min_elevation_m && 
			   max_elevation_m == other.max_elevation_m && 
			   continental_frequency == other.continental_frequency &&
			   is_visible == other.is_visible;
	}
	bool operator!=(const BCCMTerrainSettings &other) const { return !(*this == other); }
};

class BlockClipmapRenderer {
private:
	BlockClipmapProfile profile{};

	godot::RID master_mesh_rid;
	BCCMShaderData shader_data;

	struct LODLevelData {
		godot::RID multimesh_rid;
		godot::RID instance_rid;
		uint32_t last_candidate_count{ 0 };
		uint32_t last_visible_count{ 0 };
	};

	std::array<LODLevelData, BlockClipmapProfile::MAX_LEVELS> levels{};

	godot::RID scenario_rid;
	bool is_initialized{ false };

	BCCMTerrainSettings last_settings;

	std::vector<float> local_upload_buffer;
	std::array<std::vector<float>, BlockClipmapProfile::MAX_LEVELS> cached_buffers{};
	
	static constexpr uint32_t RING_BUFFER_SIZE = 3;
	uint32_t frame_index{ 0 };
	std::array<std::array<godot::PackedFloat32Array, RING_BUFFER_SIZE>, BlockClipmapProfile::MAX_LEVELS> multimesh_ring_buffers;

	// Pre-allocated fixed-capacity array for current level evaluation (Zero hot-path heap allocation)
	std::array<TerrainClipmapBlockState, BlockClipmapProfile::MAX_CANDIDATES> candidate_blocks{};

	godot::RID create_master_block_mesh();

public:
	BlockClipmapRenderer() = default;
	~BlockClipmapRenderer();

	void initialize(godot::RID p_scenario);
	void cleanup();

	void update(godot::Camera3D *p_camera, const BCCMTerrainSettings &settings);

	uint32_t get_candidate_count(uint8_t lod) const {
		return lod < profile.level_count ? levels[lod].last_candidate_count : 0;
	}
	uint32_t get_visible_count(uint8_t lod) const {
		return lod < profile.level_count ? levels[lod].last_visible_count : 0;
	}
	uint32_t get_submitted_streams() const { return is_initialized ? profile.level_count : 0; }
	bool initialized() const { return is_initialized; }
};

} // namespace multinet::rendering

#endif // MULTINET_BLOCK_CLIPMAP_RENDERER_H

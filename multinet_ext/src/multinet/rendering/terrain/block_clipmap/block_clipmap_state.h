#ifndef MULTINET_BLOCK_CLIPMAP_STATE_H
#define MULTINET_BLOCK_CLIPMAP_STATE_H

#include "multinet/rendering/terrain/block_clipmap/block_clipmap_ids.h"
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/aabb.hpp>
#include <godot_cpp/variant/basis.hpp>

namespace multinet::rendering {

struct SurfaceBlockPlacement {
	TerrainRenderBlockKey key;

	godot::Basis block_to_active_frame;
	godot::Vector3 local_origin;
	godot::AABB local_aabb;

	uint64_t frame_epoch{0};

	uint32_t topology_version{1};
	uint32_t projection_version{1};
	uint32_t terrain_version{1};
	uint32_t source_version{1};
};

struct TerrainClipmapBlockState {
	SurfaceBlockPlacement placement;
	uint8_t edge_mask{ 0 }; // bitmask: +u(1), -u(2), +v(4), -v(8)
	uint32_t gpu_layer{ 255 };
	bool is_visible{ false };
};

} // namespace multinet::rendering

#endif // MULTINET_BLOCK_CLIPMAP_STATE_H

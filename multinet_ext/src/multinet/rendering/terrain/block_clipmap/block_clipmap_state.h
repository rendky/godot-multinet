#ifndef MULTINET_BLOCK_CLIPMAP_STATE_H
#define MULTINET_BLOCK_CLIPMAP_STATE_H

#include "multinet/rendering/terrain/block_clipmap/block_clipmap_ids.h"
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/aabb.hpp>

namespace multinet::rendering {

struct TerrainClipmapBlockState {
	TerrainRenderBlockKey key;
	godot::Vector3 world_origin;
	godot::AABB world_aabb;
	bool is_visible{ false };
	uint8_t edge_mask{ 0 }; // bitmask: +x(1), -x(2), +z(4), -z(8)
};

} // namespace multinet::rendering

#endif // MULTINET_BLOCK_CLIPMAP_STATE_H

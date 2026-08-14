#include "godot/settlement_adapter.h"
#include "godot/structure_adapter.h"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/scene_tree.hpp>

#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>

namespace godot {

MultinetSettlementNode3D::MultinetSettlementNode3D() {
	set_notify_transform(true);
}

void MultinetSettlementNode3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_seed", "seed"), &MultinetSettlementNode3D::set_seed);
	ClassDB::bind_method(D_METHOD("get_seed"), &MultinetSettlementNode3D::get_seed);

	ClassDB::bind_method(D_METHOD("set_block_id", "id"), &MultinetSettlementNode3D::set_block_id);
	ClassDB::bind_method(D_METHOD("get_block_id"), &MultinetSettlementNode3D::get_block_id);

	ClassDB::bind_method(D_METHOD("set_size_x_m", "size_x"), &MultinetSettlementNode3D::set_size_x_m);
	ClassDB::bind_method(D_METHOD("get_size_x_m"), &MultinetSettlementNode3D::get_size_x_m);

	ClassDB::bind_method(D_METHOD("set_size_z_m", "size_z"), &MultinetSettlementNode3D::set_size_z_m);
	ClassDB::bind_method(D_METHOD("get_size_z_m"), &MultinetSettlementNode3D::get_size_z_m);

	ClassDB::bind_method(D_METHOD("generate_settlement"), &MultinetSettlementNode3D::generate_settlement);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "seed"), "set_seed", "get_seed");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "block_id"), "set_block_id", "get_block_id");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "size_x_m"), "set_size_x_m", "get_size_x_m");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "size_z_m"), "set_size_z_m", "get_size_z_m");
}

void MultinetSettlementNode3D::_notification(int p_what) {
	if (p_what == NOTIFICATION_READY) {
		generate_settlement();
	} else if (p_what == NOTIFICATION_TRANSFORM_CHANGED) {
		Vector3 current_scale = get_scale();
		if (!current_scale.is_equal_approx(Vector3(1.0f, 1.0f, 1.0f))) {
			size_x_m *= current_scale.x;
			size_z_m *= current_scale.z;
			set_scale(Vector3(1.0f, 1.0f, 1.0f));
			if (is_inside_tree()) {
				generate_settlement();
			}
		}
	}
}

void MultinetSettlementNode3D::set_seed(uint32_t p_seed) {
	if (seed == p_seed) return;
	seed = p_seed;
	if (is_inside_tree()) generate_settlement();
}

uint32_t MultinetSettlementNode3D::get_seed() const { return seed; }

void MultinetSettlementNode3D::set_block_id(uint64_t p_id) {
	if (block_id == p_id) return;
	block_id = p_id;
	if (is_inside_tree()) generate_settlement();
}

uint64_t MultinetSettlementNode3D::get_block_id() const { return block_id; }

void MultinetSettlementNode3D::set_size_x_m(float p_size_x) {
	if (size_x_m == p_size_x) return;
	size_x_m = p_size_x;
	if (is_inside_tree()) generate_settlement();
}

float MultinetSettlementNode3D::get_size_x_m() const { return size_x_m; }

void MultinetSettlementNode3D::set_size_z_m(float p_size_z) {
	if (size_z_m == p_size_z) return;
	size_z_m = p_size_z;
	if (is_inside_tree()) generate_settlement();
}

float MultinetSettlementNode3D::get_size_z_m() const { return size_z_m; }

void MultinetSettlementNode3D::generate_settlement() {
	// 1. Generate Block
	Multinet::SettlementGenerator::BlockGenerationResult result{};
	Multinet::Vec3f extents = { size_x_m, 0.0f, size_z_m };
	Multinet::FramePosition64 center = { 0.0, 0.0, 0.0 };
	
	Multinet::SettlementGenerator::generate_block(seed, block_id, center, extents, result);

	TypedArray<Node> children = get_children();
	int active_building_index = 0;

	// 2. Spawn or update buildings using node pooling for massive performance gains
	for (size_t i = 0; i < result.compound_count; ++i) {
		const Multinet::BuildingDevelopmentRequest& req = result.buildings[i];
		
		MultinetStructureNode3D* building = nullptr;
		
		// Find the next available structure node to reuse
		while (active_building_index < children.size() && !building) {
			Node* child = Object::cast_to<Node>(children[active_building_index]);
			building = Object::cast_to<MultinetStructureNode3D>(child);
			
			if (!building && child) {
				// If it's a leftover BlockGround mesh or something else, delete it
				child->queue_free();
			}
			active_building_index++;
		}
		
		// If we ran out of pooled nodes, create a new one
		if (!building) {
			building = memnew(MultinetStructureNode3D);
			add_child(building);
			building->set_owner(get_tree()->get_edited_scene_root());
		}
		
		// We don't use raycasting here! The PCDL architecture dictates that LandformConstraints 
		// flatten the terrain, rather than raycasting buildings down.
		building->set_building_id(req.building_key.path_hash);
		building->set_program(static_cast<int>(req.program_type));
		building->set_size_x_m(req.extents_m.x * 2.0f);
		building->set_size_y_m(req.extents_m.y * 2.0f);
		building->set_size_z_m(req.extents_m.z * 2.0f);
		
		building->set_position(Vector3(req.position.x, req.position.y, req.position.z));
	}

	// 3. Queue free any excess pooled nodes we didn't use this frame
	while (active_building_index < children.size()) {
		Node* child = Object::cast_to<Node>(children[active_building_index]);
		if (child && !child->is_queued_for_deletion()) {
			child->queue_free();
		}
		active_building_index++;
	}
}

} // namespace godot

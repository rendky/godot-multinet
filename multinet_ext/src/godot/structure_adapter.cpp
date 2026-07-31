#include "godot/structure_adapter.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>

namespace godot {

MultinetStructureNode3D::MultinetStructureNode3D() {
	// Do not use add_child in the constructor to avoid Godot memory crashes on delete
}

void MultinetStructureNode3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_building_id", "id"), &MultinetStructureNode3D::set_building_id);
	ClassDB::bind_method(D_METHOD("get_building_id"), &MultinetStructureNode3D::get_building_id);

	ClassDB::bind_method(D_METHOD("set_program", "program"), &MultinetStructureNode3D::set_program);
	ClassDB::bind_method(D_METHOD("get_program"), &MultinetStructureNode3D::get_program);

	ClassDB::bind_method(D_METHOD("set_size_x_m", "size_x"), &MultinetStructureNode3D::set_size_x_m);
	ClassDB::bind_method(D_METHOD("get_size_x_m"), &MultinetStructureNode3D::get_size_x_m);

	ClassDB::bind_method(D_METHOD("set_size_y_m", "size_y"), &MultinetStructureNode3D::set_size_y_m);
	ClassDB::bind_method(D_METHOD("get_size_y_m"), &MultinetStructureNode3D::get_size_y_m);

	ClassDB::bind_method(D_METHOD("set_size_z_m", "size_z"), &MultinetStructureNode3D::set_size_z_m);
	ClassDB::bind_method(D_METHOD("get_size_z_m"), &MultinetStructureNode3D::get_size_z_m);

	ClassDB::bind_method(D_METHOD("update_structure"), &MultinetStructureNode3D::update_structure);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "building_id"), "set_building_id", "get_building_id");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "program"), "set_program", "get_program");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "size_x_m"), "set_size_x_m", "get_size_x_m");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "size_y_m"), "set_size_y_m", "get_size_y_m");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "size_z_m"), "set_size_z_m", "get_size_z_m");
}

void MultinetStructureNode3D::_notification(int p_what) {
	if (p_what == NOTIFICATION_READY) {
		update_structure();
	}
}

void MultinetStructureNode3D::set_building_id(uint64_t p_id) {
	if (building_id == p_id) return;
	building_id = p_id;
	update_structure();
}

uint64_t MultinetStructureNode3D::get_building_id() const {
	return building_id;
}

void MultinetStructureNode3D::set_program(int p_program) {
	if (program == p_program) return;
	program = p_program;
	update_structure();
}

int MultinetStructureNode3D::get_program() const {
	return program;
}

void MultinetStructureNode3D::set_size_x_m(float p_size_x) {
	if (size_x_m == p_size_x) return;
	size_x_m = p_size_x;
	update_structure();
}

float MultinetStructureNode3D::get_size_x_m() const {
	return size_x_m;
}

void MultinetStructureNode3D::set_size_y_m(float p_size_y) {
	if (size_y_m == p_size_y) return;
	size_y_m = p_size_y;
	update_structure();
}

float MultinetStructureNode3D::get_size_y_m() const {
	return size_y_m;
}

void MultinetStructureNode3D::set_size_z_m(float p_size_z) {
	if (size_z_m == p_size_z) return;
	size_z_m = p_size_z;
	update_structure();
}

float MultinetStructureNode3D::get_size_z_m() const {
	return size_z_m;
}

static Ref<BoxMesh> shared_box_mesh;
static Ref<BoxShape3D> shared_box_shape;

void MultinetStructureNode3D::update_structure() {
	if (shared_box_mesh.is_null()) {
		shared_box_mesh.instantiate();
		shared_box_mesh->set_size(Vector3(1.0f, 1.0f, 1.0f));
	}
	if (shared_box_shape.is_null()) {
		shared_box_shape.instantiate();
		shared_box_shape->set_size(Vector3(1.0f, 1.0f, 1.0f));
	}

	if (!mesh_instance) {
		mesh_instance = memnew(MeshInstance3D);
		mesh_instance->set_name("StructureMesh");
		add_child(mesh_instance);

		static_body = memnew(StaticBody3D);
		static_body->set_name("StructureStaticBody");
		add_child(static_body);

		collision_shape = memnew(CollisionShape3D);
		collision_shape->set_name("StructureCollisionShape");
		static_body->add_child(collision_shape);
	}

	if (mesh_instance) {
		mesh_instance->set_mesh(shared_box_mesh);
		mesh_instance->set_scale(Vector3(size_x_m, size_y_m, size_z_m));
		mesh_instance->set_position(Vector3(0.0f, size_y_m * 0.5f, 0.0f));
	}

	if (collision_shape) {
		collision_shape->set_shape(shared_box_shape);
		collision_shape->set_scale(Vector3(size_x_m, size_y_m, size_z_m));
		collision_shape->set_position(Vector3(0.0f, size_y_m * 0.5f, 0.0f));
	}
}

} // namespace godot

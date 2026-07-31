#include "godot/water_adapter.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>

namespace godot {

MultinetWaterBody3D::MultinetWaterBody3D() {
	mesh_instance = memnew(MeshInstance3D);
	mesh_instance->set_name("WaterMesh");
	add_child(mesh_instance);

	collision_shape = memnew(CollisionShape3D);
	collision_shape->set_name("WaterCollisionShape");
	add_child(collision_shape);
}

void MultinetWaterBody3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_water_body_id", "id"), &MultinetWaterBody3D::set_water_body_id);
	ClassDB::bind_method(D_METHOD("get_water_body_id"), &MultinetWaterBody3D::get_water_body_id);

	ClassDB::bind_method(D_METHOD("set_surface_elevation_m", "elevation"), &MultinetWaterBody3D::set_surface_elevation_m);
	ClassDB::bind_method(D_METHOD("get_surface_elevation_m"), &MultinetWaterBody3D::get_surface_elevation_m);

	ClassDB::bind_method(D_METHOD("set_size_x_m", "size_x"), &MultinetWaterBody3D::set_size_x_m);
	ClassDB::bind_method(D_METHOD("get_size_x_m"), &MultinetWaterBody3D::get_size_x_m);

	ClassDB::bind_method(D_METHOD("set_size_z_m", "size_z"), &MultinetWaterBody3D::set_size_z_m);
	ClassDB::bind_method(D_METHOD("get_size_z_m"), &MultinetWaterBody3D::get_size_z_m);

	ClassDB::bind_method(D_METHOD("set_depth_m", "depth"), &MultinetWaterBody3D::set_depth_m);
	ClassDB::bind_method(D_METHOD("get_depth_m"), &MultinetWaterBody3D::get_depth_m);

	ClassDB::bind_method(D_METHOD("set_density_kgm3", "density"), &MultinetWaterBody3D::set_density_kgm3);
	ClassDB::bind_method(D_METHOD("get_density_kgm3"), &MultinetWaterBody3D::get_density_kgm3);

	ClassDB::bind_method(D_METHOD("update_water_body"), &MultinetWaterBody3D::update_water_body);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "water_body_id"), "set_water_body_id", "get_water_body_id");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "surface_elevation_m"), "set_surface_elevation_m", "get_surface_elevation_m");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "size_x_m"), "set_size_x_m", "get_size_x_m");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "size_z_m"), "set_size_z_m", "get_size_z_m");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "depth_m"), "set_depth_m", "get_depth_m");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "density_kgm3"), "set_density_kgm3", "get_density_kgm3");
}

void MultinetWaterBody3D::_notification(int p_what) {
	if (p_what == NOTIFICATION_READY) {
		update_water_body();
	}
}

void MultinetWaterBody3D::set_water_body_id(uint64_t p_id) {
	if (water_body_id == p_id) return;
	water_body_id = p_id;
	update_water_body();
}

uint64_t MultinetWaterBody3D::get_water_body_id() const {
	return water_body_id;
}

void MultinetWaterBody3D::set_surface_elevation_m(float p_elev) {
	if (surface_elevation_m == p_elev) return;
	surface_elevation_m = p_elev;
	update_water_body();
}

float MultinetWaterBody3D::get_surface_elevation_m() const {
	return surface_elevation_m;
}

void MultinetWaterBody3D::set_size_x_m(float p_size_x) {
	if (size_x_m == p_size_x) return;
	size_x_m = p_size_x;
	update_water_body();
}

float MultinetWaterBody3D::get_size_x_m() const {
	return size_x_m;
}

void MultinetWaterBody3D::set_size_z_m(float p_size_z) {
	if (size_z_m == p_size_z) return;
	size_z_m = p_size_z;
	update_water_body();
}

float MultinetWaterBody3D::get_size_z_m() const {
	return size_z_m;
}

void MultinetWaterBody3D::set_depth_m(float p_depth) {
	if (depth_m == p_depth) return;
	depth_m = p_depth;
	update_water_body();
}

float MultinetWaterBody3D::get_depth_m() const {
	return depth_m;
}

void MultinetWaterBody3D::set_density_kgm3(float p_density) {
	if (density_kgm3 == p_density) return;
	density_kgm3 = p_density;
	update_water_body();
}

float MultinetWaterBody3D::get_density_kgm3() const {
	return density_kgm3;
}

void MultinetWaterBody3D::update_water_body() {
	set_position(Vector3(0.0f, surface_elevation_m, 0.0f));

	Ref<PlaneMesh> plane;
	plane.instantiate();
	plane->set_size(Vector2(size_x_m, size_z_m));
	mesh_instance->set_mesh(plane);

	Ref<BoxShape3D> box;
	box.instantiate();
	box->set_size(Vector3(size_x_m, depth_m, size_z_m));

	collision_shape->set_shape(box);
	collision_shape->set_position(Vector3(0.0f, -depth_m * 0.5f, 0.0f));
}

} // namespace godot

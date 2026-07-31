#include "godot/terrain_adapter.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/world3d.hpp>

namespace godot {

MultinetBCCMNode3D::MultinetBCCMNode3D() {
}

MultinetBCCMNode3D::~MultinetBCCMNode3D() {
	free_rendering();
}

void MultinetBCCMNode3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_seed", "seed"), &MultinetBCCMNode3D::set_seed);
	ClassDB::bind_method(D_METHOD("get_seed"), &MultinetBCCMNode3D::get_seed);
	ClassDB::bind_method(D_METHOD("set_min_elevation_m", "min_elevation_m"), &MultinetBCCMNode3D::set_min_elevation_m);
	ClassDB::bind_method(D_METHOD("get_min_elevation_m"), &MultinetBCCMNode3D::get_min_elevation_m);
	ClassDB::bind_method(D_METHOD("set_max_elevation_m", "max_elevation_m"), &MultinetBCCMNode3D::set_max_elevation_m);
	ClassDB::bind_method(D_METHOD("get_max_elevation_m"), &MultinetBCCMNode3D::get_max_elevation_m);
	ClassDB::bind_method(D_METHOD("set_frequency", "frequency"), &MultinetBCCMNode3D::set_frequency);
	ClassDB::bind_method(D_METHOD("get_frequency"), &MultinetBCCMNode3D::get_frequency);
	ClassDB::bind_method(D_METHOD("set_camera_target", "path"), &MultinetBCCMNode3D::set_camera_target);
	ClassDB::bind_method(D_METHOD("get_camera_target"), &MultinetBCCMNode3D::get_camera_target);

	ClassDB::bind_method(D_METHOD("get_candidate_count", "lod"), &MultinetBCCMNode3D::get_candidate_count);
	ClassDB::bind_method(D_METHOD("get_visible_count", "lod"), &MultinetBCCMNode3D::get_visible_count);
	ClassDB::bind_method(D_METHOD("get_submitted_streams"), &MultinetBCCMNode3D::get_submitted_streams);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "seed"), "set_seed", "get_seed");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "min_elevation_m", PROPERTY_HINT_RANGE, "-10000.0, 10000.0, 1.0"), "set_min_elevation_m", "get_min_elevation_m");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_elevation_m", PROPERTY_HINT_RANGE, "-10000.0, 10000.0, 1.0"), "set_max_elevation_m", "get_max_elevation_m");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "frequency", PROPERTY_HINT_RANGE, "0.0, 100.0, 0.01"), "set_frequency", "get_frequency");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "camera_target"), "set_camera_target", "get_camera_target");
}

void MultinetBCCMNode3D::_notification(int p_what) {
	if (p_what == NOTIFICATION_READY) {
		init_rendering();
		set_process(true);
	} else if (p_what == NOTIFICATION_PROCESS) {
		Camera3D *cam = nullptr;
		if (!camera_target.is_empty()) {
			Node *target = get_node_or_null(camera_target);
			if (target) {
				cam = Object::cast_to<Camera3D>(target);
			}
		}
		if (!cam && get_viewport()) {
			cam = get_viewport()->get_camera_3d();
		}
		
		if (cam) {
			if (!bccm_renderer.initialized() && get_world_3d().is_valid()) {
				bccm_renderer.initialize(get_world_3d()->get_scenario());
			}
			multinet::rendering::BCCMTerrainSettings settings;
			settings.seed = seed;
			settings.min_elevation_m = min_elevation_m;
			settings.max_elevation_m = max_elevation_m;
			settings.continental_frequency = continental_frequency;
			settings.is_visible = is_visible_in_tree();

			bccm_renderer.update(cam, settings);
		}
	} else if (p_what == NOTIFICATION_EXIT_TREE) {
		free_rendering();
	}
}

void MultinetBCCMNode3D::init_rendering() {
	if (bccm_renderer.initialized()) return;
	if (get_world_3d().is_valid()) {
		bccm_renderer.initialize(get_world_3d()->get_scenario());
	}
}

void MultinetBCCMNode3D::free_rendering() {
	bccm_renderer.cleanup();
}

void MultinetBCCMNode3D::set_seed(uint32_t p_seed) { seed = p_seed; }
uint32_t MultinetBCCMNode3D::get_seed() const { return seed; }

void MultinetBCCMNode3D::set_min_elevation_m(float p_elev) { min_elevation_m = p_elev; }
float MultinetBCCMNode3D::get_min_elevation_m() const { return min_elevation_m; }

void MultinetBCCMNode3D::set_max_elevation_m(float p_elev) { max_elevation_m = p_elev; }
float MultinetBCCMNode3D::get_max_elevation_m() const { return max_elevation_m; }

void MultinetBCCMNode3D::set_frequency(float p_freq) { continental_frequency = p_freq; }
float MultinetBCCMNode3D::get_frequency() const { return continental_frequency; }

void MultinetBCCMNode3D::set_camera_target(const godot::NodePath &p_path) { camera_target = p_path; }
godot::NodePath MultinetBCCMNode3D::get_camera_target() const { return camera_target; }

uint32_t MultinetBCCMNode3D::get_candidate_count(int p_lod) const { return bccm_renderer.get_candidate_count(static_cast<uint8_t>(p_lod)); }
uint32_t MultinetBCCMNode3D::get_visible_count(int p_lod) const { return bccm_renderer.get_visible_count(static_cast<uint8_t>(p_lod)); }
uint32_t MultinetBCCMNode3D::get_submitted_streams() const { return bccm_renderer.get_submitted_streams(); }

} // namespace godot

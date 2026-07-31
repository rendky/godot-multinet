#ifndef MULTINET_TERRAIN_ADAPTER_H
#define MULTINET_TERRAIN_ADAPTER_H

#include <godot_cpp/classes/node3d.hpp>
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_renderer.h"

namespace godot {

class Camera3D;

class MultinetBCCMNode3D : public Node3D {
	GDCLASS(MultinetBCCMNode3D, Node3D);

private:
	uint32_t seed{ 1337 };
	float min_elevation_m{ -100.0f };
	float max_elevation_m{ 2000.0f };
	float continental_frequency{ 50.0f };

	godot::NodePath camera_target;

	multinet::rendering::BlockClipmapRenderer bccm_renderer;

	void init_rendering();
	void free_rendering();

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	MultinetBCCMNode3D();
	~MultinetBCCMNode3D();

	void set_seed(uint32_t p_seed);
	uint32_t get_seed() const;

	void set_min_elevation_m(float p_elev);
	float get_min_elevation_m() const;

	void set_max_elevation_m(float p_elev);
	float get_max_elevation_m() const;

	void set_frequency(float p_freq);
	float get_frequency() const;

	void set_camera_target(const godot::NodePath &p_path);
	godot::NodePath get_camera_target() const;

	uint32_t get_candidate_count(int p_lod) const;
	uint32_t get_visible_count(int p_lod) const;
	uint32_t get_submitted_streams() const;
};

} // namespace godot

#endif // MULTINET_TERRAIN_ADAPTER_H

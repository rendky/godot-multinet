#ifndef MULTINET_WATER_ADAPTER_H
#define MULTINET_WATER_ADAPTER_H

#include <godot_cpp/classes/area3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/box_shape3d.hpp>
#include <godot_cpp/classes/plane_mesh.hpp>

#include "multinet/world/hydrology/water_body.h"

namespace godot {

class MultinetWaterBody3D : public Area3D {
	GDCLASS(MultinetWaterBody3D, Area3D);

private:
	uint64_t water_body_id{ 1 };
	float surface_elevation_m{ 0.0f };
	float size_x_m{ 1024.0f };
	float size_z_m{ 1024.0f };
	float depth_m{ 50.0f };
	float density_kgm3{ 1000.0f };

	MeshInstance3D *mesh_instance{ nullptr };
	CollisionShape3D *collision_shape{ nullptr };

protected:
	static void _bind_methods();

public:
	MultinetWaterBody3D();
	~MultinetWaterBody3D() = default;

	void _notification(int p_what);

	void set_water_body_id(uint64_t p_id);
	uint64_t get_water_body_id() const;

	void set_surface_elevation_m(float p_elev);
	float get_surface_elevation_m() const;

	void set_size_x_m(float p_size_x);
	float get_size_x_m() const;

	void set_size_z_m(float p_size_z);
	float get_size_z_m() const;

	void set_depth_m(float p_depth);
	float get_depth_m() const;

	void set_density_kgm3(float p_density);
	float get_density_kgm3() const;

	void update_water_body();
};

} // namespace godot

#endif // MULTINET_WATER_ADAPTER_H

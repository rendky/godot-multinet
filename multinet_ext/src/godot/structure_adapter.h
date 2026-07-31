#ifndef MULTINET_STRUCTURE_ADAPTER_H
#define MULTINET_STRUCTURE_ADAPTER_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/static_body3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/box_mesh.hpp>
#include <godot_cpp/classes/box_shape3d.hpp>

#include "multinet/world/structure/structure_package.h"

namespace godot {

class MultinetStructureNode3D : public Node3D {
	GDCLASS(MultinetStructureNode3D, Node3D);

private:
	uint64_t building_id{ 1 };
	int program{ 0 }; // BuildingProgram cast to int for Godot enum property
	float size_x_m{ 10.0f };
	float size_y_m{ 6.0f };
	float size_z_m{ 12.0f };

	MeshInstance3D *mesh_instance{ nullptr };
	StaticBody3D *static_body{ nullptr };
	CollisionShape3D *collision_shape{ nullptr };

protected:
	static void _bind_methods();

public:
	MultinetStructureNode3D();
	~MultinetStructureNode3D() = default;

	void _notification(int p_what);

	void set_building_id(uint64_t p_id);
	uint64_t get_building_id() const;

	void set_program(int p_program);
	int get_program() const;

	void set_size_x_m(float p_size_x);
	float get_size_x_m() const;

	void set_size_y_m(float p_size_y);
	float get_size_y_m() const;

	void set_size_z_m(float p_size_z);
	float get_size_z_m() const;

	void update_structure();
};

} // namespace godot

#endif // MULTINET_STRUCTURE_ADAPTER_H

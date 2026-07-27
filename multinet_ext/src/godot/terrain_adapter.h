#ifndef MULTINET_TERRAIN_ADAPTER_H
#define MULTINET_TERRAIN_ADAPTER_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/static_body3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/height_map_shape3d.hpp>

#include "multinet/world/terrain/heightfield_generator.h"
#include "multinet/world/terrain/region_tile.h"
#include "multinet/world/terrain/terrain_recipe.h"

namespace godot {

class MultinetTerrainChunk3D : public Node3D {
	GDCLASS(MultinetTerrainChunk3D, Node3D);

private:
	int64_t cell_x{ 0 };
	int64_t cell_y{ 0 };
	int64_t cell_z{ 0 };

	uint32_t seed{ 0xDEADBEEF };
	float min_elevation_m{ -200.0f };
	float max_elevation_m{ 500.0f };
	float continental_frequency{ 0.003f };

	MeshInstance3D *mesh_instance{ nullptr };
	StaticBody3D *static_body{ nullptr };
	CollisionShape3D *collision_shape{ nullptr };

protected:
	static void _bind_methods();

public:
	MultinetTerrainChunk3D();
	~MultinetTerrainChunk3D() = default;

	void _notification(int p_what);

	void set_cell_x(int64_t p_val);
	int64_t get_cell_x() const;

	void set_cell_z(int64_t p_val);
	int64_t get_cell_z() const;

	void set_seed(uint32_t p_seed);
	uint32_t get_seed() const;

	void set_min_elevation_m(float p_elev);
	float get_min_elevation_m() const;

	void set_max_elevation_m(float p_elev);
	float get_max_elevation_m() const;

	void set_frequency(float p_freq);
	float get_frequency() const;

	void generate_chunk();
};

} // namespace godot

#endif // MULTINET_TERRAIN_ADAPTER_H

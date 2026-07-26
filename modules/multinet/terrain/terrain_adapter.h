#ifndef MULTINET_TERRAIN_ADAPTER_H
#define MULTINET_TERRAIN_ADAPTER_H

#include "scene/3d/mesh_instance_3d.h"
#include "scene/3d/physics/static_body_3d.h"
#include "scene/3d/physics/collision_shape_3d.h"
#include "scene/resources/mesh.h"
#include "scene/resources/3d/height_map_shape_3d.h"

#include "modules/multinet/terrain/heightfield_generator.h"
#include "modules/multinet/terrain/region_tile.h"
#include "modules/multinet/terrain/terrain_recipe.h"

namespace Multinet {

class MultinetTerrainChunk3D : public Node3D {
	GDCLASS(MultinetTerrainChunk3D, Node3D);

private:
	int64_t cell_x{ 0 };
	int64_t cell_y{ 0 };
	int64_t cell_z{ 0 };

	uint32_t seed{ 0xDEADBEEF };
	float max_elevation_m{ 500.0f };
	float continental_frequency{ 0.0001f };

	MeshInstance3D *mesh_instance{ nullptr };
	StaticBody3D *static_body{ nullptr };
	CollisionShape3D *collision_shape{ nullptr };

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	MultinetTerrainChunk3D();
	~MultinetTerrainChunk3D() = default;

	void set_cell_x(int64_t p_val);
	int64_t get_cell_x() const;

	void set_cell_z(int64_t p_val);
	int64_t get_cell_z() const;

	void set_seed(uint32_t p_seed);
	uint32_t get_seed() const;

	void set_max_elevation_m(float p_elev);
	float get_max_elevation_m() const;

	void generate_chunk();
};

} // namespace Multinet

#endif // MULTINET_TERRAIN_ADAPTER_H

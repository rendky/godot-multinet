#ifndef MULTINET_SETTLEMENT_ADAPTER_H
#define MULTINET_SETTLEMENT_ADAPTER_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/box_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include "multinet/world/settlement/settlement_generator.h"

namespace godot {

class MultinetSettlementNode3D : public Node3D {
	GDCLASS(MultinetSettlementNode3D, Node3D)

private:
	uint32_t seed{ 12345 };
	uint64_t block_id{ 1 };
	float size_x_m{ 100.0f };
	float size_z_m{ 100.0f };

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	MultinetSettlementNode3D();
	~MultinetSettlementNode3D() = default;

	void set_seed(uint32_t p_seed);
	uint32_t get_seed() const;

	void set_block_id(uint64_t p_id);
	uint64_t get_block_id() const;

	void set_size_x_m(float p_size_x);
	float get_size_x_m() const;

	void set_size_z_m(float p_size_z);
	float get_size_z_m() const;

	void generate_settlement();
};

} // namespace godot

#endif // MULTINET_SETTLEMENT_ADAPTER_H

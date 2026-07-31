#ifndef MULTINET_STRUCTURE_LAYOUT_SOLVER_H
#define MULTINET_STRUCTURE_LAYOUT_SOLVER_H

#include "structure_package.h"
#include "../../core/squirrel_noise5.h"
#include <algorithm>

namespace Multinet {

class StructureLayoutSolver {
private:
	static void generate_room_nodes(const RoomRecord& p_room, CompiledBuildingPackage& r_pkg) noexcept {
		if (r_pkg.node_count + 4 > CompiledBuildingPackage::MAX_NODES) return;

		// We place 4 structural nodes at the corners of the room for a simple load-bearing system.
		Vec3f center = {
			static_cast<float>(p_room.local_bounds.position.x),
			static_cast<float>(p_room.local_bounds.position.y),
			static_cast<float>(p_room.local_bounds.position.z)
		};
		Vec3f extents = {
			static_cast<float>(p_room.local_bounds.size.x * 0.5f),
			p_room.ceiling_height,
			static_cast<float>(p_room.local_bounds.size.z * 0.5f)
		};

		// 4 Base nodes
		uint16_t base_index = static_cast<uint16_t>(r_pkg.node_count);
		for (int i = 0; i < 4; ++i) {
			StructuralNode& node = r_pkg.structural_nodes[r_pkg.node_count];
			node.id = r_pkg.package_id + r_pkg.node_count * 10;
			
			float dx = (i % 2 == 0) ? -extents.x : extents.x;
			float dz = (i / 2 == 0) ? -extents.z : extents.z;
			
			node.local_position = Vec3f{ center.x + dx, center.y, center.z + dz };
			node.floor_index = p_room.floor_index;
			node.support_flags = 1; // 1 = Foundation
			r_pkg.node_count++;
		}
		
		// 4 Roof nodes (for simplicity we just duplicate the base nodes vertically)
		if (r_pkg.node_count + 4 > CompiledBuildingPackage::MAX_NODES) return;
		
		uint16_t roof_index = static_cast<uint16_t>(r_pkg.node_count);
		for (int i = 0; i < 4; ++i) {
			StructuralNode& node = r_pkg.structural_nodes[r_pkg.node_count];
			node.id = r_pkg.package_id + r_pkg.node_count * 10;
			
			float dx = (i % 2 == 0) ? -extents.x : extents.x;
			float dz = (i / 2 == 0) ? -extents.z : extents.z;
			
			node.local_position = Vec3f{ center.x + dx, center.y + extents.y, center.z + dz };
			node.floor_index = p_room.floor_index + 1;
			node.support_flags = 2; // 2 = Roof Support
			r_pkg.node_count++;
		}
		
		// Generate 4 Columns connecting base to roof
		for (int i = 0; i < 4; ++i) {
			if (r_pkg.member_count >= CompiledBuildingPackage::MAX_MEMBERS) break;
			StructuralMember& member = r_pkg.structural_members[r_pkg.member_count];
			member.id = r_pkg.package_id + r_pkg.member_count * 100;
			member.node_a = r_pkg.structural_nodes[base_index + i].id;
			member.node_b = r_pkg.structural_nodes[roof_index + i].id;
			member.type = StructuralMemberType::Column;
			r_pkg.member_count++;
		}
	}

public:
	[[nodiscard]] static constexpr bool solve_layout(const BuildingDevelopmentRequest& p_request, CompiledBuildingPackage& r_pkg) noexcept {
		r_pkg.package_id = p_request.building_key.path_hash;
		r_pkg.building_key = p_request.building_key;
		r_pkg.program = p_request.program_type;
		r_pkg.seed = p_request.seed;
		
		// Local bounds (Origin is center of the building for layout purposes)
		r_pkg.bounds.position = WorldPosition64{ 0.0, 0.0, 0.0 };
		r_pkg.bounds.size = WorldPosition64{ p_request.extents_m.x * 2.0, p_request.extents_m.y * 2.0, p_request.extents_m.z * 2.0 };
		
		r_pkg.room_count = 0;
		r_pkg.node_count = 0;
		r_pkg.member_count = 0;
		r_pkg.portal_count = 0;

		float w = static_cast<float>(r_pkg.bounds.size.x);
		float d = static_cast<float>(r_pkg.bounds.size.z);
		float h = 3.0f; // Single story height

		if (p_request.program_type == BuildingProgram::Warehouse) {
			// Warehouse is a single massive room (Clear span)
			RoomRecord& room = r_pkg.rooms[r_pkg.room_count++];
			room.room_id = p_request.seed + 1;
			room.function = RoomFunction::Storage;
			room.local_bounds.position = r_pkg.bounds.position;
			room.local_bounds.size = r_pkg.bounds.size;
			room.ceiling_height = p_request.extents_m.y * 2.0f; // High ceiling
			
			generate_room_nodes(room, r_pkg);
			r_pkg.preferred_system = StructuralSystem::TrussFrame;
		} 
		else if (p_request.program_type == BuildingProgram::Shop) {
			// Shop: Split front (Retail) and back (Storage)
			float split = w * 0.6f; 
			
			RoomRecord& front = r_pkg.rooms[r_pkg.room_count++];
			front.room_id = p_request.seed + 1;
			front.function = RoomFunction::Retail;
			front.local_bounds.position = WorldPosition64{ -(w - split) * 0.5, 0.0, 0.0 };
			front.local_bounds.size = WorldPosition64{ split, h, d };
			front.ceiling_height = h;
			
			RoomRecord& back = r_pkg.rooms[r_pkg.room_count++];
			back.room_id = p_request.seed + 2;
			back.function = RoomFunction::Storage;
			back.local_bounds.position = WorldPosition64{ split * 0.5, 0.0, 0.0 };
			back.local_bounds.size = WorldPosition64{ w - split, h, d };
			back.ceiling_height = h;
			
			generate_room_nodes(front, r_pkg);
			generate_room_nodes(back, r_pkg);
			r_pkg.preferred_system = StructuralSystem::SteelFrame;
			
			// Portal between front and back
			StructurePortal& door = r_pkg.portals[r_pkg.portal_count++];
			door.portal_id = p_request.seed + 3;
			door.room_a = front.room_id;
			door.room_b = back.room_id;
			door.type = PortalType::InteriorDoor;
		}
		else {
			// Dwelling: Basic 3 room BSP layout
			float split_x = w * 0.5f;
			float split_z = d * 0.5f;
			
			RoomRecord& entry = r_pkg.rooms[r_pkg.room_count++];
			entry.room_id = p_request.seed + 1;
			entry.function = RoomFunction::Entry;
			entry.local_bounds.position = WorldPosition64{ -split_x * 0.5, 0.0, 0.0 };
			entry.local_bounds.size = WorldPosition64{ split_x, h, d };
			entry.ceiling_height = h;
			
			RoomRecord& living = r_pkg.rooms[r_pkg.room_count++];
			living.room_id = p_request.seed + 2;
			living.function = RoomFunction::Living;
			living.local_bounds.position = WorldPosition64{ split_x * 0.5, 0.0, -split_z * 0.5 };
			living.local_bounds.size = WorldPosition64{ split_x, h, split_z };
			living.ceiling_height = h;
			
			RoomRecord& sleeping = r_pkg.rooms[r_pkg.room_count++];
			sleeping.room_id = p_request.seed + 3;
			sleeping.function = RoomFunction::Sleeping;
			sleeping.local_bounds.position = WorldPosition64{ split_x * 0.5, 0.0, split_z * 0.5 };
			sleeping.local_bounds.size = WorldPosition64{ split_x, h, split_z };
			sleeping.ceiling_height = h;
			
			generate_room_nodes(entry, r_pkg);
			generate_room_nodes(living, r_pkg);
			generate_room_nodes(sleeping, r_pkg);
			r_pkg.preferred_system = StructuralSystem::LoadBearingMasonry;
		}
		
		return true;
	}
};

} // namespace Multinet

#endif // MULTINET_STRUCTURE_LAYOUT_SOLVER_H

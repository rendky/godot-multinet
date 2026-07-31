#ifndef MULTINET_STRUCTURE_PACKAGE_H
#define MULTINET_STRUCTURE_PACKAGE_H

#include "../../core/schema/feature_key.h"
#include "structure_types.h"
#include <array>
#include <cstddef>
#include <span>
#include <type_traits>

namespace Multinet {

struct CompiledBuildingPackage {
	static constexpr size_t MAX_ROOMS = 8;
	static constexpr size_t MAX_PORTALS = 8;
	static constexpr size_t MAX_NODES = 32;
	static constexpr size_t MAX_MEMBERS = 64;

	BuildingID package_id{ 0 };
	FeatureKey building_key{};
	BuildingProgram program{ BuildingProgram::Dwelling };
	StructuralSystem preferred_system{ StructuralSystem::LoadBearingMasonry };

	AABB64 bounds{};

	std::array<RoomRecord, MAX_ROOMS> rooms{};
	size_t room_count{ 0 };

	std::array<StructurePortal, MAX_PORTALS> portals{};
	size_t portal_count{ 0 };

	std::array<StructuralNode, MAX_NODES> structural_nodes{};
	size_t node_count{ 0 };

	std::array<StructuralMember, MAX_MEMBERS> structural_members{};
	size_t member_count{ 0 };

	uint32_t seed{ 0xDEADBEEF };
	uint32_t version{ 1 };

	[[nodiscard]] constexpr std::span<const RoomRecord> get_rooms() const noexcept {
		return std::span<const RoomRecord>{ rooms.data(), room_count };
	}

	[[nodiscard]] constexpr std::span<const StructurePortal> get_portals() const noexcept {
		return std::span<const StructurePortal>{ portals.data(), portal_count };
	}

	[[nodiscard]] constexpr std::span<const StructuralNode> get_nodes() const noexcept {
		return std::span<const StructuralNode>{ structural_nodes.data(), node_count };
	}

	[[nodiscard]] constexpr std::span<const StructuralMember> get_members() const noexcept {
		return std::span<const StructuralMember>{ structural_members.data(), member_count };
	}
};

static_assert(std::is_trivially_copyable_v<CompiledBuildingPackage>, "CompiledBuildingPackage must be trivially copyable POD");

} // namespace Multinet

#endif // MULTINET_STRUCTURE_PACKAGE_H

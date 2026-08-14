#ifndef MULTINET_STRUCTURE_TYPES_H
#define MULTINET_STRUCTURE_TYPES_H

#include <cstdint>
#include <span>
#include "multinet/core/coordinates.h"

namespace Multinet {

using BuildingID          = uint64_t;
using PlotID              = uint64_t;
using RoomID              = uint64_t;
using PortalID            = uint64_t;
using StructuralNodeID    = uint64_t;
using StructuralMemberID  = uint64_t;
using MaterialID          = uint32_t;

enum class BuildingProgram : uint16_t {
	Dwelling,
	Apartment,
	Shop,
	Market,
	Office,
	Warehouse,
	Workshop,
	School,
	Civic,
	Religious,
	Barracks,
	Fortification,
	FarmBuilding,
	Station,
	Utility
};

enum class StructuralSystem : uint8_t {
	LoadBearingMasonry,
	TimberFrame,
	ReinforcedConcreteFrame,
	SteelFrame,
	TrussFrame,
	ArchAndVault,
	Mixed
};

enum class RoomFunction : uint16_t {
	Entry,
	Corridor,
	Stair,
	Ramp,
	Living,
	Sleeping,
	Kitchen,
	Hygiene,
	Storage,
	Office,
	Assembly,
	Retail,
	Service,
	Workshop,
	Classroom,
	Secure,
	Utility
};

enum class StructuralMemberType : uint8_t {
	Foundation,
	Column,
	Beam,
	Brace,
	TrussChord,
	TrussWeb,
	LoadBearingWall,
	SlabProxy,
	Arch,
	RoofMember,
	Connection
};

enum class PortalType : uint8_t {
	ExteriorEntrance,
	InteriorDoor,
	Gate,
	Stair,
	Ramp,
	ServiceEntrance
};

struct RoomRecord {
	RoomID room_id{ 0 };
	RoomFunction function{ RoomFunction::Living };

	AABB64 local_bounds{};

	float floor_height{ 0.0f };
	float ceiling_height{ 3.0f };

	uint16_t floor_index{ 0 };
	uint16_t occupancy_capacity{ 4 };

	uint32_t access_mask{ 0 };
	uint32_t flags{ 0 };
};

struct StructurePortal {
	PortalID portal_id{ 0 };

	RoomID room_a{ 0 };
	RoomID room_b{ 0 };

	PortalType type{ PortalType::InteriorDoor };

	FramePosition64 local_position{};

	float width{ 1.0f };
	float height{ 2.1f };

	uint32_t access_mask{ 0 };
	uint32_t state_flags{ 0 };
};

struct StructuralNode {
	StructuralNodeID id{ 0 };

	Vec3f local_position{};

	uint16_t floor_index{ 0 };
	uint16_t support_flags{ 0 };

	float tributary_mass{ 0.0f };

	uint32_t connection_offset{ 0 };
	uint16_t connection_count{ 0 };
	uint16_t flags{ 0 };
};

struct StructuralMember {
	StructuralMemberID id{ 0 };

	StructuralNodeID node_a{ 0 };
	StructuralNodeID node_b{ 0 };

	StructuralMemberType type{ StructuralMemberType::Beam };
	MaterialID material{ 0 };

	Vec3f section_dimensions{};

	float rest_length{ 0.0f };
	float estimated_capacity{ 0.0f };

	uint32_t geometry_binding{ 0 };
	uint32_t flags{ 0 };
};

} // namespace Multinet

#endif // MULTINET_STRUCTURE_TYPES_H

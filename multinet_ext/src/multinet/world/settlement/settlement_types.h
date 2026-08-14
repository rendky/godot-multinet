#ifndef MULTINET_SETTLEMENT_TYPES_H
#define MULTINET_SETTLEMENT_TYPES_H

#include "../../core/coordinates.h"
#include "../../core/schema/feature_key.h"
#include <cstdint>
#include <type_traits>
#include "../structure/structure_types.h"

namespace Multinet {

using SettlementID = uint64_t;
using DistrictID = uint64_t;
using LandUnitID = uint64_t;
using CompoundID = uint64_t;
using BlockID = uint64_t;
using RoadSegmentID = uint64_t;

enum class SettlementClass : uint8_t {
	Farmstead = 0,
	Hamlet = 1,
	Village = 2,
	Town = 3,
	City = 4
};

enum class RoadHierarchy : uint8_t {
	Footpath = 0,
	ServiceLane = 1,
	ResidentialStreet = 2,
	CollectorRoad = 3,
	ArterialRoad = 4
};

struct BlockRecord {
	BlockID block_id{ 0 };
	FeatureKey key{};
	AABB64 bounds{};
	uint16_t compound_count{ 0 };
	uint8_t zoning_category{ 0 };
	uint8_t flags{ 0 };
};

struct CompoundSubdivisionRequest {
	AABB64 parent_bounds{};
	uint32_t seed{ 0 };
	uint8_t depth{ 0 };
	uint8_t max_depth{ 4 };
	float min_compound_size_m{ 15.0f };
};

struct CompoundRecord {
	CompoundID compound_id{ 0 };
	FeatureKey key{};
	AABB64 bounds{};
	uint8_t zoning_class{ 0 };
	uint16_t building_count{ 0 };
	uint32_t flags{ 0 };
};

struct RoadSegmentRecord {
	RoadSegmentID road_id{ 0 };
	FeatureKey key{};
	FramePosition64 start_pos{};
	FramePosition64 end_pos{};
	RoadHierarchy hierarchy{ RoadHierarchy::Footpath };
	float width_m{ 2.0f };
};

struct BuildingDevelopmentRequest {
	CompoundID compound_id{ 0 };
	FeatureKey building_key{};
	FramePosition64 position{};
	Vec3f extents_m{};
	BuildingProgram program_type{ BuildingProgram::Dwelling };
	uint32_t seed{ 0 };
};

static_assert(std::is_trivially_copyable_v<BlockRecord>, "BlockRecord must be POD");
static_assert(std::is_trivially_copyable_v<CompoundRecord>, "CompoundRecord must be POD");
static_assert(std::is_trivially_copyable_v<RoadSegmentRecord>, "RoadSegmentRecord must be POD");
static_assert(std::is_trivially_copyable_v<BuildingDevelopmentRequest>, "BuildingDevelopmentRequest must be POD");

} // namespace Multinet

#endif // MULTINET_SETTLEMENT_TYPES_H

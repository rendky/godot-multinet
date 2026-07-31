#ifndef MULTINET_TERRAIN_LANDFORM_CONTRACTS_H
#define MULTINET_TERRAIN_LANDFORM_CONTRACTS_H

#include "../../core/coordinates.h"
#include <cstdint>
#include <type_traits>

namespace Multinet {

enum class LandformType : uint8_t {
	RoadCorridor = 0,
	BuildingPad = 1,
	CompoundTerrace = 2,
	FieldTerrace = 3,
	DitchSwale = 4,
	EmbankmentLevee = 5,
	CanalBed = 6,
	RetainingEdge = 7,
	ExcavationFill = 8
};

struct LandformConstraint {
	LandformType type{ LandformType::BuildingPad };
	WorldPosition64 center{};
	Vec3f extents_m{};
	float target_elevation_m{ 0.0f };
	uint32_t priority{ 0 };
};

struct ProtectedTerrainFeature {
	uint64_t feature_id{ 0 };
	AABB64 bounds{};
	uint32_t protection_flags{ 0 };
};

struct TerrainModificationResult {
	bool accepted{ false };
	uint32_t previous_version{ 0 };
	uint32_t new_version{ 0 };
	AABB64 dirty_bounds{};
};

struct TopologyPromotionRequest {
	uint64_t request_id{ 0 };
	AABB64 volume_bounds{};
	uint8_t promotion_type{ 0 }; // 0: Basement, 1: Tunnel, 2: Underpass, 3: Deep Culvert
};

static_assert(std::is_trivially_copyable_v<LandformConstraint>, "LandformConstraint must be POD");
static_assert(std::is_trivially_copyable_v<ProtectedTerrainFeature>, "ProtectedTerrainFeature must be POD");
static_assert(std::is_trivially_copyable_v<TerrainModificationResult>, "TerrainModificationResult must be POD");
static_assert(std::is_trivially_copyable_v<TopologyPromotionRequest>, "TopologyPromotionRequest must be POD");

} // namespace Multinet

#endif // MULTINET_TERRAIN_LANDFORM_CONTRACTS_H

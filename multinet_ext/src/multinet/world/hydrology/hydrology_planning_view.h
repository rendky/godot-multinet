#ifndef MULTINET_HYDROLOGY_PLANNING_VIEW_H
#define MULTINET_HYDROLOGY_PLANNING_VIEW_H

#include "../../core/coordinates.h"
#include <cstdint>
#include <type_traits>

namespace Multinet {

struct RiverReachPlanningSummary {
	uint64_t reach_id{ 0 };
	Vec3f start_pos{};
	Vec3f end_pos{};
	float bankfull_width_m{ 0.0f };
	float mean_depth_m{ 0.0f };
	uint8_t discharge_class{ 0 };
};

struct HydrologyPlanningView {
	uint64_t water_body_id{ 0 };
	AABB64 broad_bounds{};
	float mean_water_level_m{ 0.0f };
	float flood_envelope_margin_m{ 0.0f };
	bool is_bridge_suitable{ true };
	bool is_ford_suitable{ false };
	uint32_t hydrology_version{ 0 };
	uint32_t terrain_bed_version{ 0 };
};

static_assert(std::is_trivially_copyable_v<RiverReachPlanningSummary>, "RiverReachPlanningSummary must be POD");
static_assert(std::is_trivially_copyable_v<HydrologyPlanningView>, "HydrologyPlanningView must be POD");

} // namespace Multinet

#endif // MULTINET_HYDROLOGY_PLANNING_VIEW_H

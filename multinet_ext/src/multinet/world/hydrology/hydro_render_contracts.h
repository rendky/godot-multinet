#ifndef MULTINET_HYDRO_RENDER_CONTRACTS_H
#define MULTINET_HYDRO_RENDER_CONTRACTS_H

#include "multinet/core/coordinates.h"
#include "multinet/world/hydrology/hydro_types.h"

namespace Multinet {

// ============================================================================
// Gate: WATER-RENDER-01 (Render Pipeline Data Handoff Records)
// ============================================================================

struct WaterRenderRecord {
	WaterBodyID        body{ 0 };
	WaterSurfaceRegime regime{ WaterSurfaceRegime::Lake };

	AABB64 world_bounds{};

	double surface_elevation_m{ 0.0 };
	float  mean_depth_m{ 0.0f };

	Vec2f mean_flow_ms{};

	float turbidity{ 0.0f };
	float temperature_k{ 288.15f };
	float salinity_psu{ 0.0f };

	WaveProfileID wave_profile{ 0 };

	uint32_t shoreline_version{ 1 };
	uint32_t state_version{ 1 };
	uint32_t quality_flags{ 0 };
};

struct WetSurfaceRenderRecord {
	AABB64 bounds{};
	float  wetness{ 0.0f };
	float  film_depth_m{ 0.0f };
	Vec2f  flow_direction{};
	float  sediment{ 0.0f };
	uint32_t material_id{ 0 };
};

struct FoamSourceRecord {
	WorldPosition64 position{};
	Vec3f           direction{};
	float           width_m{ 1.0f };
	float           intensity{ 1.0f };
	float           persistence{ 1.0f };
	uint32_t        seed{ 0xDEADBEEF };
};

} // namespace Multinet

#endif // MULTINET_HYDRO_RENDER_CONTRACTS_H

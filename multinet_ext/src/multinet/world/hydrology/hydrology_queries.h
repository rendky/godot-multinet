#ifndef MULTINET_HYDROLOGY_QUERIES_H
#define MULTINET_HYDROLOGY_QUERIES_H

#include "multinet/core/coordinates.h"
#include "multinet/world/hydrology/hydro_types.h"

namespace Multinet {

// ============================================================================
// Gate: WATER-JOLT-01 (Zero-Allocation Buoyancy & Drag Requests for Physics)
// Gate: WATER-NAV-01 (Fordability & Navigation Hazards)
// Gate: WATER-CPUQUERY-01 (CPU Wave Query Contract)
// Gate: WATER-SHORE-01 (Shoreline Conformance Segment)
// ============================================================================

struct WaterBodyForceRequest {
	uint64_t    jolt_body_id{ 0 };
	WaterBodyID water_body{ 0 };

	Vec3f sample_position{};
	Vec3f water_velocity{};

	float submerged_fraction{ 0.0f };
	float displaced_volume_m3{ 0.0f };
	float fluid_density_kgm3{ 1000.0f };

	float linear_drag{ 0.0f };
	float angular_drag{ 0.0f };
	float wave_acceleration{ 0.0f };

	uint32_t flags{ 0 };
};

struct WaterHazardSample {
	float depth_m{ 0.0f };
	float speed_ms{ 0.0f };
	float acceleration_ms2{ 0.0f };

	float temperature_k{ 288.15f };
	float turbidity{ 0.0f };

	float slip_risk{ 0.0f };
	float sweep_risk{ 0.0f };
	float drowning_risk{ 0.0f };
	float contamination_risk{ 0.0f };

	uint32_t flags{ 0 };
};

struct CrowdWaterRegionRecord {
	AABB64 bounds{};

	float mean_depth_m{ 0.0f };
	float max_depth_m{ 0.0f };
	float mean_speed_ms{ 0.0f };

	float fordability{ 1.0f }; // 1.0 = fully fordable, 0.0 = impassable
	float evacuation_cost{ 0.0f };
	float growth_rate{ 0.0f };

	uint32_t version{ 1 };
};

struct WaveQueryInput {
	WaterBodyID     body{ 0 };
	WorldPosition64 position{};
	double          time_s{ 0.0 };
	uint32_t        quality_lane{ 0 };
};

struct WaveQueryResult {
	double surface_elevation_m{ 0.0 };
	Vec3f  normal{ 0.0f, 1.0f, 0.0f };
	Vec3f  surface_velocity{};
	Vec3f  surface_acceleration{};

	float compression{ 0.0f };
	float steepness{ 0.0f };
	float breaking_tendency{ 0.0f };
};

struct WaterSurfaceSample {
	WaterBodyID        body{ 0 };
	WaterSurfaceRegime regime{ WaterSurfaceRegime::Lake };

	double surface_elevation_m{ 0.0 };
	float  depth_m{ 0.0f };

	Vec2f mean_flow_ms{};
	Vec2f orbital_flow_ms{};

	float slope{ 0.0f };
	float compression{ 0.0f };
	float breaking_tendency{ 0.0f };
	float foam_tendency{ 0.0f };

	float temperature_k{ 288.15f };
	float turbidity{ 0.0f };
	float salinity_psu{ 0.0f };

	WaveProfileID wave_profile{ 0 };
	uint32_t      state_version{ 1 };
};

struct ShorelineSegmentRecord {
	WaterBodyID body{ 0 };

	WorldPosition64 p0{};
	WorldPosition64 p1{};

	float mean_water_elevation_m{ 0.0f };
	float bank_slope{ 0.0f };
	float wet_width_m{ 0.0f };
	float foam_tendency{ 0.0f };
	float sediment_load{ 0.0f };

	uint32_t material_left{ 0 };
	uint32_t material_right{ 0 };
	uint32_t state_version{ 1 };
};

[[nodiscard]] inline float calculate_water_depth(double p_water_surface_elevation_m, double p_terrain_bed_elevation_m) noexcept {
	if (p_water_surface_elevation_m <= p_terrain_bed_elevation_m) {
		return 0.0f; // Ground is above water surface -> dry bed
	}
	return static_cast<float>(p_water_surface_elevation_m - p_terrain_bed_elevation_m);
}

} // namespace Multinet

#endif // MULTINET_HYDROLOGY_QUERIES_H

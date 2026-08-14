#ifndef MULTINET_WATER_BODY_H
#define MULTINET_WATER_BODY_H

#include "multinet/core/coordinates.h"
#include "multinet/world/hydrology/hydro_types.h"

namespace Multinet {

// ============================================================================
// Gate: WATER-MASS-01 (Water Mass Conservation Ledger & Water Body State)
// Gate: WATER-OCEAN-01 (Sea & Ocean Water Bodies)
// ============================================================================

struct WaterBodyRecord {
	WaterBodyID   id{ 0 };
	WaterBodyKind kind{ WaterBodyKind::Lake };

	FramePosition64 centroid{};
	AABB64          bounds{};

	double volume_m3{ 0.0 };
	double surface_elevation_m{ 0.0 };

	float mean_depth_m{ 0.0f };
	float max_depth_m{ 0.0f };
	float temperature_k{ 288.15f }; // ~15°C
	float turbidity{ 0.0f };
	float salinity_psu{ 0.0f };

	Vec2f mean_surface_flow_ms{};
	WaveProfileID wave_profile{ 0 };

	uint32_t shoreline_version{ 1 };
	uint32_t state_version{ 1 };
	uint32_t flags{ 0 };
};

struct WaterBodyStorageSample {
	float  elevation_m{ 0.0f };
	double area_m2{ 0.0 };
	double volume_m3{ 0.0 };
};

struct HydroMassLedger {
	double initial_mass_kg{ 0.0 };

	double precipitation_in_kg{ 0.0 };
	double snowmelt_in_kg{ 0.0 };
	double upstream_in_kg{ 0.0 };
	double pump_in_kg{ 0.0 };

	double evaporation_out_kg{ 0.0 };
	double infiltration_out_kg{ 0.0 };
	double downstream_out_kg{ 0.0 };
	double ocean_out_kg{ 0.0 };
	double pump_out_kg{ 0.0 };

	double current_storage_kg{ 0.0 };
	double numerical_residual_kg{ 0.0 };

	[[nodiscard]] constexpr double compute_residual() const noexcept {
		return initial_mass_kg + precipitation_in_kg + snowmelt_in_kg + upstream_in_kg + pump_in_kg
			- evaporation_out_kg - infiltration_out_kg - downstream_out_kg - ocean_out_kg - pump_out_kg
			- current_storage_kg;
	}
};

} // namespace Multinet

#endif // MULTINET_WATER_BODY_H

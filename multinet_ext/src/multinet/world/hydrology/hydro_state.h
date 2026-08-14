#ifndef MULTINET_HYDRO_STATE_H
#define MULTINET_HYDRO_STATE_H

#include "multinet/core/coordinates.h"
#include "multinet/world/hydrology/hydro_types.h"

#include <span>

namespace Multinet {

// ============================================================================
// Gate: WATER-MASS-01 (Runtime State Layouts, SoA Pools & Soil/Snow Input)
// ============================================================================

struct LocalPatchFrame {
	FramePosition64 world_origin{};
	Vec3f           local_up{ 0.0f, 1.0f, 0.0f };
	Vec3f           local_east{ 1.0f, 0.0f, 0.0f };
	Vec3f           local_north{ 0.0f, 0.0f, 1.0f };
	float           meters_per_cell{ 1.0f };
};

struct WaterPatchRecord {
	WaterPatchID    id{ 0 };
	WaterBodyID     owner_body{ 0 };

	LocalPatchFrame frame{};

	uint16_t cells_x{ 32 };
	uint16_t cells_z{ 32 };
	float    cell_size_m{ 1.0f };

	uint32_t solver_kind{ 0 };
	uint32_t state_version{ 1 };
	uint32_t flags{ 0 };
};

struct WaterPatchSoA {
	std::span<float> depth;
	std::span<float> surface_elevation;

	std::span<float> velocity_x;
	std::span<float> velocity_z;

	std::span<float> wet_fraction;
	std::span<float> discharge;

	std::span<float> temperature;
	std::span<float> turbidity;

	std::span<float> suspended_sand;
	std::span<float> suspended_silt;
	std::span<float> suspended_clay;

	std::span<uint32_t> boundary_flags;
	std::span<uint32_t> material_profile;
};

struct SoilHydroState {
	float surface_moisture{ 0.0f };
	float root_zone_moisture{ 0.0f };
	float deep_moisture{ 0.0f };

	float surface_saturation{ 0.0f };
	float water_table_depth_m{ 5.0f };

	float infiltration_capacity_ms{ 0.00001f };
	float drainage_rate_ms{ 0.000001f };
	float permeability{ 0.5f };

	uint32_t soil_profile{ 0 };
	uint32_t version{ 1 };
};

struct SurfaceRetentionState {
	float retained_depth_m{ 0.0f };
	float retention_capacity_m{ 0.005f }; // 5mm
	float wetness{ 0.0f };
	float absorption_rate_ms{ 0.00001f };
	float evaporation_rate_ms{ 0.000001f };
};

struct SnowHydroState {
	float snow_water_equivalent_m{ 0.0f };
	float liquid_water_fraction{ 0.0f };
	float density_kgm3{ 250.0f };
	float surface_temperature_k{ 273.15f };
	float albedo_age{ 0.0f };
	float impurity_load{ 0.0f };
	uint32_t version{ 1 };
};

struct PrecipitationHydroInput {
	CatchmentID catchment{ 0 };
	AABB64      world_bounds{};

	double water_equivalent_kg{ 0.0 };
	Vec3f  mean_momentum{};

	float             temperature_k{ 288.15f };
	PrecipitationKind kind{ PrecipitationKind::Rain };
	uint32_t          forcing_version{ 1 };
};

} // namespace Multinet

#endif // MULTINET_HYDRO_STATE_H

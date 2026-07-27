#ifndef MULTINET_HYDRO_SEDIMENT_H
#define MULTINET_HYDRO_SEDIMENT_H

#include "multinet/core/coordinates.h"
#include "multinet/world/hydrology/hydro_types.h"

#include <array>

namespace Multinet {

// ============================================================================
// Gate: WATER-MASS-01 (Sediment Transport & Subgrid Bathymetry)
// ============================================================================

static constexpr uint32_t HYDRO_SUBGRID_SAMPLES = 8;

struct SedimentCellState {
	float suspended_sand_kg{ 0.0f };
	float suspended_silt_kg{ 0.0f };
	float suspended_clay_kg{ 0.0f };

	float available_sand_kg{ 0.0f };
	float available_silt_kg{ 0.0f };
	float available_clay_kg{ 0.0f };

	float erosion_rate_kgs{ 0.0f };
	float deposition_rate_kgs{ 0.0f };
	float bed_delta_m{ 0.0f };
};

struct HydroSubgridCell {
	float center_bed_m{ 0.0f };

	float face_bed_n_m{ 0.0f };
	float face_bed_s_m{ 0.0f };
	float face_bed_e_m{ 0.0f };
	float face_bed_w_m{ 0.0f };

	float minimum_bed_m{ 0.0f };
	float maximum_bed_m{ 0.0f };

	std::array<float, HYDRO_SUBGRID_SAMPLES> water_level_m{};
	std::array<float, HYDRO_SUBGRID_SAMPLES> wetted_area_fraction{};
	std::array<float, HYDRO_SUBGRID_SAMPLES> conveyance{};

	Vec2f    preferred_flow_direction{};
	uint32_t terrain_version{ 1 };
};

} // namespace Multinet

#endif // MULTINET_HYDRO_SEDIMENT_H

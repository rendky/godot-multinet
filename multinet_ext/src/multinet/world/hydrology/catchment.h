#ifndef MULTINET_CATCHMENT_H
#define MULTINET_CATCHMENT_H

#include "multinet/world/hydrology/hydro_types.h"

namespace Multinet {

// ============================================================================
// Gate: WATER-CATCHMENT-01 (Catchment Hydrology & Travel-Time Bands)
// ============================================================================

struct TravelTimeBand {
	float travel_time_hours{ 0.0f };
	float area_fraction{ 0.0f };
	float runoff_coefficient{ 1.0f };
};

struct CatchmentState {
	CatchmentID id{ 0 };

	double area_m2{ 0.0 };
	double retained_water_m3{ 0.0 };
	double runoff_storage_m3{ 0.0 };

	float mean_slope{ 0.0f };
	float mean_soil_moisture{ 0.0f };
	float impervious_fraction{ 0.0f };
	float vegetation_roughness{ 0.0f };

	float runoff_rate_m3s{ 0.0f };
	float baseflow_rate_m3s{ 0.0f };

	uint32_t routing_profile{ 0 };
	uint32_t version{ 1 };
};

} // namespace Multinet

#endif // MULTINET_CATCHMENT_H

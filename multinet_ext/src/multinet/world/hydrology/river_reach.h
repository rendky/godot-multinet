#ifndef MULTINET_RIVER_REACH_H
#define MULTINET_RIVER_REACH_H

#include "multinet/world/hydrology/hydro_types.h"

namespace Multinet {

// ============================================================================
// Gate: WATER-RIVER-01 (Channel Routing & Cross-Sections)
// ============================================================================

struct HydroReachRecord {
	HydroReachID id{ 0 };
	HydroNodeID  upstream{ 0 };
	HydroNodeID  downstream{ 0 };

	double length_m{ 0.0 };
	double bed_slope{ 0.001 };

	float bankfull_width_m{ 10.0f };
	float bankfull_depth_m{ 2.0f };
	float roughness{ 0.035f }; // Manning's n
	float sinuosity{ 1.0f };

	float discharge_m3s{ 0.0f };
	float velocity_ms{ 0.0f };
	float sediment_load_kg{ 0.0f };

	uint32_t cross_section_offset{ 0 };
	uint16_t cross_section_count{ 0 };
	uint16_t flags{ 0 };
};

struct RiverCrossSectionSample {
	double   chainage_m{ 0.0 };
	float    lateral_offset_m{ 0.0f };
	float    bed_elevation_m{ 0.0f };
	float    roughness{ 0.035f };
	uint16_t material{ 0 };
	uint16_t flags{ 0 };
};

} // namespace Multinet

#endif // MULTINET_RIVER_REACH_H

#ifndef MULTINET_HYDRO_BARRIERS_H
#define MULTINET_HYDRO_BARRIERS_H

#include "multinet/core/coordinates.h"
#include "multinet/world/hydrology/hydro_types.h"

namespace Multinet {

// ============================================================================
// Gate: WATER-SEAM-01 (Drainage Elements, Structure Barriers & Interior Flooding)
// ============================================================================

struct DrainageElementRecord {
	DrainageElementID   id{ 0 };
	DrainageElementKind kind{ DrainageElementKind::Pipe };

	HydroNodeID upstream{ 0 };
	HydroNodeID downstream{ 0 };

	float capacity_m3s{ 1.0f };
	float current_flow_m3s{ 0.0f };
	float blockage{ 0.0f };
	float damage{ 0.0f };

	uint32_t profile{ 0 };
	uint32_t flags{ 0 };
};

struct HydroBarrierRecord {
	HydroBarrierID id{ 0 };
	AABB64         bounds{};

	float crest_elevation_m{ 0.0f };
	float permeability{ 0.0f };
	float breach_fraction{ 0.0f };
	float roughness{ 0.035f };

	uint32_t structure_id{ 0 };
	uint32_t version{ 1 };
	uint32_t flags{ 0 };
};

struct FloodCompartmentRecord {
	FloodCompartmentID id{ 0 };

	double floor_area_m2{ 0.0 };
	double current_volume_m3{ 0.0 };

	float floor_elevation_m{ 0.0f };
	float ceiling_elevation_m{ 3.0f };
	float water_elevation_m{ 0.0f };

	uint32_t first_portal{ 0 };
	uint16_t portal_count{ 0 };
	uint16_t flags{ 0 };
};

struct FloodPortalRecord {
	FloodCompartmentID a{ 0 };
	FloodCompartmentID b{ 0 };

	float sill_elevation_m{ 0.0f };
	float width_m{ 1.0f };
	float height_m{ 2.0f };
	float permeability{ 1.0f };
	float open_fraction{ 1.0f };
};

} // namespace Multinet

#endif // MULTINET_HYDRO_BARRIERS_H

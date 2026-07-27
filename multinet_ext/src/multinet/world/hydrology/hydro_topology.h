#ifndef MULTINET_HYDRO_TOPOLOGY_H
#define MULTINET_HYDRO_TOPOLOGY_H

#include "multinet/core/coordinates.h"
#include "multinet/world/hydrology/hydro_types.h"

namespace Multinet {

// ============================================================================
// Gate: WATER-SEAM-01 (Hydrograph Topology Node Record)
// ============================================================================

struct HydroNodeRecord {
	HydroNodeID     id{ 0 };
	HydroNodeKind   kind{ HydroNodeKind::Junction };
	WorldPosition64 centroid{};

	double storage_m3{ 0.0 };
	double water_surface_elevation_m{ 0.0 };

	uint32_t first_out_edge{ 0 };
	uint16_t out_edge_count{ 0 };
	uint16_t flags{ 0 };
};

} // namespace Multinet

#endif // MULTINET_HYDRO_TOPOLOGY_H

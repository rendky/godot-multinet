#ifndef MULTINET_HYDRO_TYPES_H
#define MULTINET_HYDRO_TYPES_H

#include <cstdint>

namespace Multinet {

// ============================================================================
// Gate: WATER-MASS-01 (Stable Hydro Identities & Core Enums)
// ============================================================================

using CatchmentID        = uint64_t;
using HydroNodeID        = uint64_t;
using HydroReachID       = uint64_t;
using WaterBodyID        = uint64_t;
using DrainageNetworkID  = uint64_t;
using DrainageElementID  = uint64_t;
using FloodCompartmentID = uint64_t;
using WaterPatchID       = uint64_t;
using HydroBarrierID     = uint64_t;
using HydroControlID     = uint64_t;
using HydroEventID       = uint64_t;
using HydroMaterialID    = uint32_t;
using WaveProfileID      = uint32_t;

enum class HydroNodeKind : uint8_t {
	Source,
	Junction,
	WaterBody,
	Wetland,
	Reservoir,
	Estuary,
	OceanBoundary,
	Sink,
	InteriorCompartment
};

enum class WaterBodyKind : uint8_t {
	Puddle,
	Pond,
	Lake,
	Reservoir,
	Wetland,
	RiverPool,
	CanalBasin,
	Estuary,
	Sea,
	Ocean,
	InteriorPool
};

enum class WaterSurfaceRegime : uint8_t {
	Damp,
	ThinFilm,
	Puddle,
	Basin,
	River,
	Lake,
	Reservoir,
	Estuary,
	Sea,
	Ocean,
	Waterfall,
	Flood,
	Interior
};

enum class PrecipitationKind : uint8_t {
	Rain,
	Drizzle,
	Snow,
	Sleet,
	Hail,
	FreezingRain
};

enum class DrainageElementKind : uint8_t {
	Inlet,
	Pipe,
	Gutter,
	Channel,
	Culvert,
	Pump,
	Weir,
	Gate,
	Outlet,
	Sump
};

} // namespace Multinet

#endif // MULTINET_HYDRO_TYPES_H

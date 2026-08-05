#ifndef MULTINET_SURFACE_FRAME_H
#define MULTINET_SURFACE_FRAME_H

#include "surface_address.h"
#include "multinet/core/coordinates.h" // For Vec3d

namespace Multinet {

struct Basis3d {
	Vec3d u_axis{ 1.0, 0.0, 0.0 };
	Vec3d up_axis{ 0.0, 1.0, 0.0 };
	Vec3d v_axis{ 0.0, 0.0, 1.0 };
};

struct SurfaceFrame {
	SurfacePosition64 origin{};
	Basis3d tangent_basis{};

	uint64_t frame_epoch{ 0 };
	uint32_t topology_version{ 1 };
	uint32_t projection_version{ 1 };
};

} // namespace Multinet

#endif // MULTINET_SURFACE_FRAME_H

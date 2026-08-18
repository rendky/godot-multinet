#ifndef MULTINET_RENDERING_CHP_BOUNDS_H
#define MULTINET_RENDERING_CHP_BOUNDS_H

#include "multinet/rendering/chp/chp_profile.h"
#include "multinet/rendering/chp/chp_view.h"

namespace multinet::rendering::chp {

struct CHPCurvedCoverageBounds {
	double minimum_x_m{ 0.0 };
	double maximum_x_m{ 0.0 };
	double minimum_y_m{ 0.0 };
	double maximum_y_m{ 0.0 };
	double minimum_z_m{ 0.0 };
	double maximum_z_m{ 0.0 };
	bool valid{ false };
};

[[nodiscard]] bool try_build_conservative_curved_bounds(
	const ResolvedCurvedHorizonProfile& profile,
	double signed_camera_altitude_m,
	double flat_min_x_m,
	double flat_max_x_m,
	double flat_min_z_m,
	double flat_max_z_m,
	double height_min_m,
	double height_max_m,
	CHPCurvedCoverageBounds& out_bounds
) noexcept;

[[nodiscard]] bool try_build_conservative_curved_bounds(
	const CurvedHorizonView& view,
	double flat_min_x_m,
	double flat_max_x_m,
	double flat_min_z_m,
	double flat_max_z_m,
	double height_min_m,
	double height_max_m,
	CHPCurvedCoverageBounds& out_bounds
) noexcept;

} // namespace multinet::rendering::chp

#endif // MULTINET_RENDERING_CHP_BOUNDS_H

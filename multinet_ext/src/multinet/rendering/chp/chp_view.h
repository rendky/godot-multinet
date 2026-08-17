#ifndef MULTINET_RENDERING_CHP_VIEW_H
#define MULTINET_RENDERING_CHP_VIEW_H

#include "multinet/core/spatial/surface_frame.h"
#include "multinet/rendering/chp/chp_profile.h"

namespace multinet::rendering::chp {

struct CurvedHorizonView {
	Multinet::WorldPresentationManifest manifest{};
	ResolvedCurvedHorizonProfile profile{};
	Multinet::SurfacePosition64 camera_surface{};
	Multinet::FramePosition64 camera_in_frame{};
	double camera_surface_height_m{ 0.0 }; // Signed canonical camera altitude
	double signed_camera_surface_altitude_m{ 0.0 }; // Explicit signed alias
	double horizon_observer_height_m{ 0.0 }; // Nonnegative clamp for LOS/arc horizon
	double horizon_line_of_sight_m{ 0.0 };
	double horizon_surface_arc_m{ 0.0 };
	double nominal_admitted_intrinsic_radius_m{ 0.0 };
	uint64_t surface_frame_epoch{ 0 };
	uint32_t camera_epoch{ 0 };
	uint32_t source_epoch{ 0 };
	bool chp_effective{ false };

	[[nodiscard]] bool is_valid() const noexcept {
		return manifest.is_valid() && camera_surface.is_valid() &&
			std::isfinite(camera_in_frame.x) && std::isfinite(camera_in_frame.y) && std::isfinite(camera_in_frame.z) &&
			std::isfinite(camera_surface_height_m) &&
			std::isfinite(signed_camera_surface_altitude_m) &&
			std::isfinite(horizon_observer_height_m) && horizon_observer_height_m >= 0.0 &&
			std::isfinite(horizon_line_of_sight_m) && horizon_line_of_sight_m >= 0.0 &&
			std::isfinite(horizon_surface_arc_m) && horizon_surface_arc_m >= 0.0 &&
			std::isfinite(nominal_admitted_intrinsic_radius_m) && nominal_admitted_intrinsic_radius_m >= 0.0 &&
			surface_frame_epoch != 0 && camera_epoch != 0 && source_epoch != 0 &&
			(!chp_effective || profile.is_valid());
	}
};

[[nodiscard]] bool try_build_curved_horizon_view(
	const Multinet::WorldDomainManifest& domain,
	const Multinet::WorldPresentationManifest& presentation,
	const ResolvedCurvedHorizonProfile& profile,
	const Multinet::SurfacePosition64& camera_surface,
	const Multinet::FramePosition64& camera_in_frame,
	uint64_t surface_frame_epoch,
	uint32_t camera_epoch,
	uint32_t source_epoch,
	CurvedHorizonView& out_view
) noexcept;

} // namespace multinet::rendering::chp

#endif // MULTINET_RENDERING_CHP_VIEW_H

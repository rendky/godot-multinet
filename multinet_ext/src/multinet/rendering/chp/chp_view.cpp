#include "multinet/rendering/chp/chp_view.h"

#include "multinet/rendering/chp/chp_kernel.h"

#include <algorithm>
#include <cmath>

namespace multinet::rendering::chp {

namespace {
constexpr double PI = 3.141592653589793238462643383279502884;

[[nodiscard]] bool finite_frame_position(const Multinet::FramePosition64& value) noexcept {
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}
}

bool try_build_curved_horizon_view(
	const Multinet::WorldDomainManifest& domain,
	const Multinet::WorldPresentationManifest& presentation,
	const ResolvedCurvedHorizonProfile& profile,
	const Multinet::SurfacePosition64& camera_surface,
	const Multinet::FramePosition64& camera_in_frame,
	uint64_t surface_frame_epoch,
	uint32_t camera_epoch,
	uint32_t source_epoch,
	CurvedHorizonView& out_view
) noexcept {
	out_view = {};
	if (!domain.is_valid() || !presentation.is_valid() ||
		presentation.domain_manifest_hash != domain.domain_manifest_hash ||
		!camera_surface.is_valid() ||
		camera_surface.topology_version != domain.topology_version ||
		camera_surface.projection_version != domain.projection_version ||
		!finite_frame_position(camera_in_frame) || surface_frame_epoch == 0 ||
		camera_epoch == 0 || source_epoch == 0) return false;

	const bool enabled = presentation.chp_enabled;
	if (enabled) {
		if (!profile.is_valid() || presentation.chp_kernel_version != CHP_KERNEL_CONTRACT_VERSION_1) return false;
		const double manifest_radius_m = static_cast<double>(presentation.resolved_chp_radius_mm) * 0.001;
		if (!std::isfinite(manifest_radius_m) || manifest_radius_m <= 0.0 ||
			std::abs(profile.radius_m - manifest_radius_m) > 0.0005) return false;
	} else if (presentation.chp_kernel_version != 0) {
		return false;
	}

	const double signed_camera_surface_altitude_m = camera_surface.altitude_m;
	const double horizon_observer_height_m = std::max(0.0, signed_camera_surface_altitude_m);

	out_view.manifest = presentation;
	out_view.profile = profile;
	out_view.camera_surface = camera_surface;
	out_view.camera_in_frame = camera_in_frame;
	out_view.signed_camera_surface_altitude_m = signed_camera_surface_altitude_m;
	out_view.camera_surface_height_m = signed_camera_surface_altitude_m;
	out_view.horizon_observer_height_m = horizon_observer_height_m;
	out_view.surface_frame_epoch = surface_frame_epoch;
	out_view.camera_epoch = camera_epoch;
	out_view.source_epoch = source_epoch;
	out_view.chp_effective = enabled;

	const double radius_m = static_cast<double>(presentation.resolved_chp_radius_mm) * 0.001;
	if (radius_m > 0.0 && std::isfinite(radius_m)) {
		const double h = horizon_observer_height_m;
		out_view.horizon_line_of_sight_m = std::sqrt(2.0 * radius_m * h + h * h);
		out_view.horizon_surface_arc_m = h > 0.0
			? radius_m * std::acos(std::clamp(radius_m / (radius_m + h), -1.0, 1.0))
			: 0.0;
	}
	if (enabled) {
		out_view.nominal_admitted_intrinsic_radius_m = std::min(
			profile.certified_maximum_deformation_distance_m,
			out_view.horizon_surface_arc_m);
	}
	return out_view.is_valid();
}

} // namespace multinet::rendering::chp

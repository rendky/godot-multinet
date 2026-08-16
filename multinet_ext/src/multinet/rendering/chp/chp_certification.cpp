#include "multinet/rendering/chp/chp_certification.h"

#include <algorithm>
#include <cmath>

namespace multinet::rendering::chp {

namespace {
constexpr double PI = 3.141592653589793238462643383279502884;
constexpr double HALF_PI = PI * 0.5;
constexpr double THETA_MARGIN = 1.0e-9;

struct ErrorMeasurement {
	double position_error_m{ 0.0 };
	double visual_up_error_radians{ 0.0 };
	bool valid{ false };
};

[[nodiscard]] double vector_distance(const Multinet::Vec3d& a, const Multinet::Vec3d& b) noexcept {
	const double dx = a.x - b.x;
	const double dy = a.y - b.y;
	const double dz = a.z - b.z;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

[[nodiscard]] ErrorMeasurement measure(
	const CurvedHorizonProfile& requested,
	double radius_m,
	double theta
) noexcept {
	const CHPIntrinsicSample sample{
		radius_m * theta, 0.0, 0.0, 0.0, 0.0
	};
	ResolvedCurvedHorizonProfile probe{};
	probe.requested = requested;
	probe.radius_m = radius_m;
	probe.inverse_radius = 1.0 / radius_m;
	probe.inverse_radius_squared = probe.inverse_radius * probe.inverse_radius;
	probe.certified_maximum_theta = HALF_PI - THETA_MARGIN;
	probe.certified_maximum_deformation_distance_m = radius_m * probe.certified_maximum_theta;
	probe.certified_maximum_u = probe.certified_maximum_theta * probe.certified_maximum_theta;
	CHPEvaluation approximate{};
	CHPEvaluation exact{};
	if (!try_evaluate_curved(probe, sample, approximate) ||
		!try_evaluate_exact_sphere(radius_m, sample, exact)) return {};
	const double cosine = std::clamp(
		approximate.height_axis.x * exact.height_axis.x +
		approximate.height_axis.y * exact.height_axis.y +
		approximate.height_axis.z * exact.height_axis.z,
		-1.0, 1.0);
	return { vector_distance(approximate.base_position_m, exact.base_position_m), std::acos(cosine), true };
}

[[nodiscard]] bool within_budget(const ErrorMeasurement& measurement, const CurvedHorizonProfile& profile) noexcept {
	return measurement.valid &&
		measurement.position_error_m <= profile.maximum_base_position_error_m &&
		measurement.visual_up_error_radians <= profile.maximum_visual_up_error_radians;
}
}

bool try_resolve_curved_horizon_profile(
	const Multinet::WorldPresentationManifest& presentation,
	const CurvedHorizonProfile& requested,
	ResolvedCurvedHorizonProfile& out_profile
) noexcept {
	out_profile = {};
	if (!presentation.is_valid() || !presentation.chp_enabled ||
		presentation.chp_kernel_version != CHP_KERNEL_CONTRACT_VERSION_1 ||
		presentation.resolved_chp_radius_mm == 0 || !requested.is_valid()) return false;

	const double radius_m = static_cast<double>(presentation.resolved_chp_radius_mm) * 0.001;
	if (!std::isfinite(radius_m) || radius_m <= 0.0) return false;
	const double requested_theta = requested.requested_maximum_deformation_distance_m / radius_m;
	const double upper_theta = std::min(requested_theta, HALF_PI - THETA_MARGIN);
	if (!std::isfinite(requested_theta) || requested_theta <= 0.0 || !(upper_theta > 0.0)) return false;

	double certified_theta = upper_theta;
	const ErrorMeasurement upper_measurement = measure(requested, radius_m, upper_theta);
	if (!within_budget(upper_measurement, requested)) {
		double low = 0.0;
		double high = upper_theta;
		for (int iteration = 0; iteration < 64; ++iteration) {
			const double mid = (low + high) * 0.5;
			const ErrorMeasurement mid_measurement = measure(requested, radius_m, mid);
			if (within_budget(mid_measurement, requested)) low = mid;
			else high = mid;
		}
		certified_theta = low;
	}

	const ErrorMeasurement final_measurement = measure(requested, radius_m, certified_theta);
	if (!within_budget(final_measurement, requested)) return false;
	out_profile.requested = requested;
	out_profile.radius_m = radius_m;
	out_profile.inverse_radius = 1.0 / radius_m;
	out_profile.inverse_radius_squared = out_profile.inverse_radius * out_profile.inverse_radius;
	out_profile.certified_maximum_theta = certified_theta;
	out_profile.certified_maximum_deformation_distance_m = radius_m * certified_theta;
	out_profile.certified_maximum_u = certified_theta * certified_theta;
	out_profile.base_position_error_at_limit_m = final_measurement.position_error_m;
	out_profile.visual_up_error_at_limit_radians = final_measurement.visual_up_error_radians;
	out_profile.distance_was_clamped = requested_theta > upper_theta ||
		out_profile.certified_maximum_deformation_distance_m + 1.0e-9 < requested.requested_maximum_deformation_distance_m;
	return out_profile.is_valid();
}

} // namespace multinet::rendering::chp

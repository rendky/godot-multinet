#ifndef MULTINET_RENDERING_CHP_PROFILE_H
#define MULTINET_RENDERING_CHP_PROFILE_H

#include "multinet/core/coordinates.h"
#include "multinet/core/spatial/world_manifests.h"

#include <cmath>
#include <cstdint>

namespace multinet::rendering::chp {

enum class CHPFunctionClass : uint8_t {
	QuadraticVerticalFallback = 0,
	SphericalPolynomial4 = 1,
	SphericalPolynomial6 = 2
};

static_assert(static_cast<uint8_t>(CHPFunctionClass::QuadraticVerticalFallback) == 0);
static_assert(static_cast<uint8_t>(CHPFunctionClass::SphericalPolynomial4) == 1);
static_assert(static_cast<uint8_t>(CHPFunctionClass::SphericalPolynomial6) == 2);

constexpr uint32_t CHP_PROFILE_VERSION_1 = 1;
constexpr uint32_t CHP_KERNEL_CONTRACT_VERSION_1 = Multinet::CHP_KERNEL_CONTRACT_VERSION_1;

[[nodiscard]] constexpr bool is_known_function_class(CHPFunctionClass value) noexcept {
	return value == CHPFunctionClass::QuadraticVerticalFallback ||
		value == CHPFunctionClass::SphericalPolynomial4 ||
		value == CHPFunctionClass::SphericalPolynomial6;
}

struct CurvedHorizonProfile {
	CHPFunctionClass function_class{ CHPFunctionClass::SphericalPolynomial6 };
	double requested_maximum_deformation_distance_m{ 0.0 };
	double maximum_base_position_error_m{ 1.0 };
	double maximum_visual_up_error_radians{ 1.0e-4 };
	uint32_t profile_version{ CHP_PROFILE_VERSION_1 };
	uint32_t flags{ 0 };

	[[nodiscard]] bool is_valid() const noexcept {
		return is_known_function_class(function_class) &&
			profile_version == CHP_PROFILE_VERSION_1 &&
			std::isfinite(requested_maximum_deformation_distance_m) &&
			requested_maximum_deformation_distance_m > 0.0 &&
			std::isfinite(maximum_base_position_error_m) &&
			maximum_base_position_error_m > 0.0 &&
			std::isfinite(maximum_visual_up_error_radians) &&
			maximum_visual_up_error_radians > 0.0;
	}
};

struct ResolvedCurvedHorizonProfile {
	CurvedHorizonProfile requested{};
	double radius_m{ 0.0 };
	double inverse_radius{ 0.0 };
	double inverse_radius_squared{ 0.0 };
	double certified_maximum_deformation_distance_m{ 0.0 };
	double certified_maximum_theta{ 0.0 };
	double certified_maximum_u{ 0.0 };
	double base_position_error_at_limit_m{ 0.0 };
	double visual_up_error_at_limit_radians{ 0.0 };
	bool distance_was_clamped{ false };

	[[nodiscard]] bool is_valid() const noexcept {
		constexpr double half_pi = 1.57079632679489661923;
		return requested.is_valid() &&
			std::isfinite(radius_m) && radius_m > 0.0 &&
			std::isfinite(inverse_radius) && inverse_radius > 0.0 &&
			std::isfinite(inverse_radius_squared) && inverse_radius_squared > 0.0 &&
			std::isfinite(certified_maximum_deformation_distance_m) && certified_maximum_deformation_distance_m >= 0.0 &&
			std::isfinite(certified_maximum_theta) && certified_maximum_theta >= 0.0 && certified_maximum_theta < half_pi &&
			std::isfinite(certified_maximum_u) && certified_maximum_u >= 0.0 &&
			std::isfinite(base_position_error_at_limit_m) && base_position_error_at_limit_m >= 0.0 &&
			std::isfinite(visual_up_error_at_limit_radians) && visual_up_error_at_limit_radians >= 0.0;
	}
};

struct CHPIntrinsicSample {
	double x_m{ 0.0 };
	double z_m{ 0.0 };
	double height_m{ 0.0 };
	double height_dx{ 0.0 };
	double height_dz{ 0.0 };
};

struct CHPEvaluation {
	Multinet::Vec3d base_position_m{};
	Multinet::Vec3d height_axis{};
	Multinet::Vec3d position_m{};
	Multinet::Vec3d tangent_x{};
	Multinet::Vec3d tangent_z{};
	Multinet::Vec3d normal{};
	double d2_m2{ 0.0 };
	double theta{ 0.0 };
	double u{ 0.0 };
	bool valid{ false };
};

} // namespace multinet::rendering::chp

#endif // MULTINET_RENDERING_CHP_PROFILE_H

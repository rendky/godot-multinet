#include "multinet/rendering/chp/chp_kernel.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace multinet::rendering::chp {

namespace {

constexpr double PI = 3.141592653589793238462643383279502884;
constexpr double HALF_PI = PI * 0.5;

[[nodiscard]] bool finite_sample(const CHPIntrinsicSample& sample) noexcept {
	return std::isfinite(sample.x_m) && std::isfinite(sample.z_m) &&
		std::isfinite(sample.height_m) && std::isfinite(sample.height_dx) && std::isfinite(sample.height_dz);
}

[[nodiscard]] bool finite_vec(const Multinet::Vec3d& value) noexcept {
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

[[nodiscard]] double dot(const Multinet::Vec3d& a, const Multinet::Vec3d& b) noexcept {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

[[nodiscard]] Multinet::Vec3d add(const Multinet::Vec3d& a, const Multinet::Vec3d& b) noexcept {
	return { a.x + b.x, a.y + b.y, a.z + b.z };
}

[[nodiscard]] Multinet::Vec3d subtract(const Multinet::Vec3d& a, const Multinet::Vec3d& b) noexcept {
	return { a.x - b.x, a.y - b.y, a.z - b.z };
}

[[nodiscard]] Multinet::Vec3d multiply(const Multinet::Vec3d& value, double scalar) noexcept {
	return { value.x * scalar, value.y * scalar, value.z * scalar };
}

[[nodiscard]] Multinet::Vec3d cross(const Multinet::Vec3d& a, const Multinet::Vec3d& b) noexcept {
	return {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	};
}

[[nodiscard]] bool normalize_checked(const Multinet::Vec3d& value, Multinet::Vec3d& out) noexcept {
	const double length_squared = dot(value, value);
	if (!std::isfinite(length_squared) || length_squared <= 1.0e-30) return false;
	const double inverse_length = 1.0 / std::sqrt(length_squared);
	out = multiply(value, inverse_length);
	return finite_vec(out);
}

[[nodiscard]] bool normalized_derivative(
	const Multinet::Vec3d& raw,
	const Multinet::Vec3d& normalized,
	const Multinet::Vec3d& derivative,
	Multinet::Vec3d& out
) noexcept {
	const double raw_length_squared = dot(raw, raw);
	if (!std::isfinite(raw_length_squared) || raw_length_squared <= 1.0e-30) return false;
	const double raw_length = std::sqrt(raw_length_squared);
	out = multiply(
		subtract(derivative, multiply(normalized, dot(normalized, derivative))),
		1.0 / raw_length);
	return finite_vec(out);
}

struct PolynomialTerms {
	double a{ 0.0 };
	double b{ 0.0 };
	double c{ 0.0 };
	double da_du{ 0.0 };
	double db_du{ 0.0 };
	double dc_du{ 0.0 };
};

[[nodiscard]] bool make_polynomial_terms(
	CHPFunctionClass function_class,
	double u,
	PolynomialTerms& out
) noexcept {
	if (!is_known_function_class(function_class) || !std::isfinite(u) || u < 0.0) return false;
	const double u2 = u * u;
	if (function_class == CHPFunctionClass::SphericalPolynomial4) {
		out.a = 1.0 - u / 6.0 + u2 / 120.0;
		out.b = u / 2.0 - u2 / 24.0;
		out.c = 1.0 - u / 2.0 + u2 / 24.0;
		out.da_du = -1.0 / 6.0 + u / 60.0;
		out.db_du = 1.0 / 2.0 - u / 12.0;
		out.dc_du = -1.0 / 2.0 + u / 12.0;
		return true;
	}
	if (function_class == CHPFunctionClass::SphericalPolynomial6) {
		const double u3 = u2 * u;
		out.a = 1.0 - u / 6.0 + u2 / 120.0 - u3 / 5040.0;
		out.b = u / 2.0 - u2 / 24.0 + u3 / 720.0;
		out.c = 1.0 - u / 2.0 + u2 / 24.0 - u3 / 720.0;
		out.da_du = -1.0 / 6.0 + u / 60.0 - u2 / 1680.0;
		out.db_du = 1.0 / 2.0 - u / 12.0 + u2 / 240.0;
		out.dc_du = -1.0 / 2.0 + u / 12.0 - u2 / 240.0;
		return true;
	}
	return false;
}

[[nodiscard]] bool finish_evaluation(
	const Multinet::Vec3d& base_position,
	const Multinet::Vec3d& height_axis,
	const Multinet::Vec3d& tangent_x,
	const Multinet::Vec3d& tangent_z,
	const CHPIntrinsicSample& sample,
	double d2,
	double theta,
	double u,
	CHPEvaluation& out
) noexcept {
	Multinet::Vec3d normal{};
	if (!normalize_checked(cross(tangent_z, tangent_x), normal)) return false;
	out = {};
	out.base_position_m = base_position;
	out.height_axis = height_axis;
	out.position_m = add(base_position, multiply(height_axis, sample.height_m));
	out.tangent_x = tangent_x;
	out.tangent_z = tangent_z;
	out.normal = normal;
	out.d2_m2 = d2;
	out.theta = theta;
	out.u = u;
	out.valid = finite_vec(out.position_m) && finite_vec(out.tangent_x) && finite_vec(out.tangent_z) &&
		finite_vec(out.normal) && finite_vec(out.height_axis);
	return out.valid;
}

} // namespace

const char *get_function_class_name(CHPFunctionClass value) noexcept {
	switch (value) {
		case CHPFunctionClass::QuadraticVerticalFallback: return "QuadraticVerticalFallback";
		case CHPFunctionClass::SphericalPolynomial4: return "SphericalPolynomial4";
		case CHPFunctionClass::SphericalPolynomial6: return "SphericalPolynomial6";
		default: return "Unknown";
	}
}

bool try_evaluate_flat(const CHPIntrinsicSample& sample, CHPEvaluation& out_evaluation) noexcept {
	out_evaluation = {};
	if (!finite_sample(sample)) return false;
	const Multinet::Vec3d base{ sample.x_m, 0.0, sample.z_m };
	const Multinet::Vec3d up{ 0.0, 1.0, 0.0 };
	const Multinet::Vec3d tangent_x{ 1.0, sample.height_dx, 0.0 };
	const Multinet::Vec3d tangent_z{ 0.0, sample.height_dz, 1.0 };
	return finish_evaluation(base, up, tangent_x, tangent_z, sample,
		sample.x_m * sample.x_m + sample.z_m * sample.z_m, 0.0, 0.0, out_evaluation);
}

bool try_evaluate_curved(
	const ResolvedCurvedHorizonProfile& profile,
	const CHPIntrinsicSample& sample,
	CHPEvaluation& out_evaluation
) noexcept {
	out_evaluation = {};
	if (!profile.is_valid() || !finite_sample(sample)) return false;
	const double x = sample.x_m;
	const double z = sample.z_m;
	const double d2 = x * x + z * z;
	const double u = d2 * profile.inverse_radius_squared;
	const double theta = std::sqrt(u);
	if (!std::isfinite(d2) || !std::isfinite(u) || !std::isfinite(theta) || theta >= HALF_PI) return false;

	if (profile.requested.function_class == CHPFunctionClass::QuadraticVerticalFallback) {
		const double drop = d2 * 0.5 * profile.inverse_radius;
		const Multinet::Vec3d base{ x, -drop, z };
		const Multinet::Vec3d up{ 0.0, 1.0, 0.0 };
		const Multinet::Vec3d tangent_x{ 1.0, sample.height_dx - x * profile.inverse_radius, 0.0 };
		const Multinet::Vec3d tangent_z{ 0.0, sample.height_dz - z * profile.inverse_radius, 1.0 };
		return finish_evaluation(base, up, tangent_x, tangent_z, sample, d2, theta, u, out_evaluation);
	}

	PolynomialTerms terms{};
	if (!make_polynomial_terms(profile.requested.function_class, u, terms)) return false;
	const double ux = 2.0 * x * profile.inverse_radius_squared;
	const double uz = 2.0 * z * profile.inverse_radius_squared;
	const double a_x = terms.a + x * terms.da_du * ux;
	const double a_z = terms.a + z * terms.da_du * uz;
	const double p0_y_x = -profile.radius_m * terms.db_du * ux;
	const double p0_y_z = -profile.radius_m * terms.db_du * uz;
	const Multinet::Vec3d base{
		terms.a * x,
		-profile.radius_m * terms.b,
		terms.a * z
	};
	const Multinet::Vec3d dbase_dx{ a_x, p0_y_x, z * terms.da_du * ux };
	const Multinet::Vec3d dbase_dz{ x * terms.da_du * uz, p0_y_z, a_z };

	const Multinet::Vec3d raw_axis{
		terms.a * x * profile.inverse_radius,
		terms.c,
		terms.a * z * profile.inverse_radius
	};
	Multinet::Vec3d axis{};
	if (!normalize_checked(raw_axis, axis)) return false;
	const Multinet::Vec3d draw_dx{
		a_x * profile.inverse_radius,
		terms.dc_du * ux,
		z * terms.da_du * ux * profile.inverse_radius
	};
	const Multinet::Vec3d draw_dz{
		x * terms.da_du * uz * profile.inverse_radius,
		terms.dc_du * uz,
		a_z * profile.inverse_radius
	};
	Multinet::Vec3d daxis_dx{};
	Multinet::Vec3d daxis_dz{};
	if (!normalized_derivative(raw_axis, axis, draw_dx, daxis_dx) ||
		!normalized_derivative(raw_axis, axis, draw_dz, daxis_dz)) return false;

	const Multinet::Vec3d tangent_x = add(add(dbase_dx, multiply(axis, sample.height_dx)), multiply(daxis_dx, sample.height_m));
	const Multinet::Vec3d tangent_z = add(add(dbase_dz, multiply(axis, sample.height_dz)), multiply(daxis_dz, sample.height_m));
	return finish_evaluation(base, axis, tangent_x, tangent_z, sample, d2, theta, u, out_evaluation);
}

bool try_evaluate_exact_sphere(
	double radius_m,
	const CHPIntrinsicSample& sample,
	CHPEvaluation& out_evaluation
) noexcept {
	out_evaluation = {};
	if (!(radius_m > 0.0) || !std::isfinite(radius_m) || !finite_sample(sample)) return false;
	const double inverse_radius = 1.0 / radius_m;
	const double d2 = sample.x_m * sample.x_m + sample.z_m * sample.z_m;
	const double distance = std::sqrt(d2);
	const double theta = distance * inverse_radius;
	if (!std::isfinite(d2) || !std::isfinite(distance) || !std::isfinite(theta) || theta >= HALF_PI) return false;

	double sinc = 1.0;
	double cosine = 1.0;
	double sine = 0.0;
	double dsinc_dtheta = 0.0;
	if (distance > 1.0e-12) {
		if (std::abs(theta) < 1.0e-4) {
			const double theta2 = theta * theta;
			const double theta4 = theta2 * theta2;
			const double theta6 = theta4 * theta2;
			sinc = 1.0 - theta2 / 6.0 + theta4 / 120.0 - theta6 / 5040.0;
			cosine = 1.0 - theta2 / 2.0 + theta4 / 24.0 - theta6 / 720.0;
			sine = theta * sinc;
			dsinc_dtheta = -theta / 3.0 + theta * theta2 / 30.0 - theta * theta4 / 840.0;
		} else {
			sine = std::sin(theta);
			cosine = std::cos(theta);
			sinc = sine / theta;
			dsinc_dtheta = (theta * cosine - sine) / (theta * theta);
		}
	}

	const Multinet::Vec3d base{ sinc * sample.x_m, radius_m * (cosine - 1.0), sinc * sample.z_m };
	const Multinet::Vec3d raw_axis{ sinc * sample.x_m * inverse_radius, cosine, sinc * sample.z_m * inverse_radius };
	Multinet::Vec3d axis{};
	if (!normalize_checked(raw_axis, axis)) return false;

	Multinet::Vec3d dbase_dx{};
	Multinet::Vec3d dbase_dz{};
	Multinet::Vec3d draw_dx{};
	Multinet::Vec3d draw_dz{};
	if (distance <= 1.0e-12) {
		dbase_dx = { 1.0, 0.0, 0.0 };
		dbase_dz = { 0.0, 0.0, 1.0 };
		draw_dx = { inverse_radius, 0.0, 0.0 };
		draw_dz = { 0.0, 0.0, inverse_radius };
	} else {
		const double dsinc_dd = dsinc_dtheta * inverse_radius;
		const double dsinc_dx = dsinc_dd * sample.x_m / distance;
		const double dsinc_dz = dsinc_dd * sample.z_m / distance;
		const double dcos_dx = -sine * sample.x_m / (radius_m * distance);
		const double dcos_dz = -sine * sample.z_m / (radius_m * distance);
		dbase_dx = { sinc + sample.x_m * dsinc_dx, radius_m * dcos_dx, sample.z_m * dsinc_dx };
		dbase_dz = { sample.x_m * dsinc_dz, radius_m * dcos_dz, sinc + sample.z_m * dsinc_dz };
		draw_dx = { (sinc + sample.x_m * dsinc_dx) * inverse_radius, dcos_dx, sample.z_m * dsinc_dx * inverse_radius };
		draw_dz = { sample.x_m * dsinc_dz * inverse_radius, dcos_dz, (sinc + sample.z_m * dsinc_dz) * inverse_radius };
	}

	Multinet::Vec3d daxis_dx{};
	Multinet::Vec3d daxis_dz{};
	if (!normalized_derivative(raw_axis, axis, draw_dx, daxis_dx) ||
		!normalized_derivative(raw_axis, axis, draw_dz, daxis_dz)) return false;
	const Multinet::Vec3d tangent_x = add(add(dbase_dx, multiply(axis, sample.height_dx)), multiply(daxis_dx, sample.height_m));
	const Multinet::Vec3d tangent_z = add(add(dbase_dz, multiply(axis, sample.height_dz)), multiply(daxis_dz, sample.height_m));
	return finish_evaluation(base, axis, tangent_x, tangent_z, sample, d2, theta, d2 * inverse_radius * inverse_radius, out_evaluation);
}

} // namespace multinet::rendering::chp

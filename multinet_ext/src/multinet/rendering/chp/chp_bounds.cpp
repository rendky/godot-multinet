#include "multinet/rendering/chp/chp_bounds.h"
#include <algorithm>
#include <cmath>

namespace multinet::rendering::chp {

namespace {

struct Interval {
	double lo{ 0.0 };
	double hi{ 0.0 };

	[[nodiscard]] static constexpr Interval point(double v) noexcept {
		return { v, v };
	}

	[[nodiscard]] static constexpr Interval range(double a, double b) noexcept {
		return { std::min(a, b), std::max(a, b) };
	}
};

[[nodiscard]] inline Interval add(Interval a, Interval b) noexcept {
	return { a.lo + b.lo, a.hi + b.hi };
}

[[nodiscard]] inline Interval sub(Interval a, Interval b) noexcept {
	return { a.lo - b.hi, a.hi - b.lo };
}

[[nodiscard]] inline Interval scale(Interval a, double s) noexcept {
	if (s >= 0.0) {
		return { a.lo * s, a.hi * s };
	} else {
		return { a.hi * s, a.lo * s };
	}
}

[[nodiscard]] inline Interval mul(Interval a, Interval b) noexcept {
	const double p1 = a.lo * b.lo;
	const double p2 = a.lo * b.hi;
	const double p3 = a.hi * b.lo;
	const double p4 = a.hi * b.hi;
	return {
		std::min({ p1, p2, p3, p4 }),
		std::max({ p1, p2, p3, p4 })
	};
}

[[nodiscard]] inline Interval square(Interval a) noexcept {
	if (a.lo <= 0.0 && a.hi >= 0.0) {
		return { 0.0, std::max(a.lo * a.lo, a.hi * a.hi) };
	} else if (a.lo > 0.0) {
		return { a.lo * a.lo, a.hi * a.hi };
	} else {
		return { a.hi * a.hi, a.lo * a.lo };
	}
}

[[nodiscard]] inline Interval sqrt_interval(Interval a) noexcept {
	return {
		std::sqrt(std::max(0.0, a.lo)),
		std::sqrt(std::max(0.0, a.hi))
	};
}

[[nodiscard]] inline Interval reciprocal(Interval a) noexcept {
	if (a.lo <= 0.0) {
		return { 0.0, 0.0 }; // Undefined / non-positive
	}
	return { 1.0 / a.hi, 1.0 / a.lo };
}

} // namespace

bool try_build_conservative_curved_bounds(
	const ResolvedCurvedHorizonProfile& profile,
	double signed_camera_altitude_m,
	double flat_min_x_m,
	double flat_max_x_m,
	double flat_min_z_m,
	double flat_max_z_m,
	double height_min_m,
	double height_max_m,
	CHPCurvedCoverageBounds& out_bounds
) noexcept {
	out_bounds = CHPCurvedCoverageBounds{};
	if (!profile.is_valid()) {
		return false;
	}

	if (!std::isfinite(flat_min_x_m) || !std::isfinite(flat_max_x_m) || flat_min_x_m > flat_max_x_m ||
	    !std::isfinite(flat_min_z_m) || !std::isfinite(flat_max_z_m) || flat_min_z_m > flat_max_z_m ||
	    !std::isfinite(height_min_m) || !std::isfinite(height_max_m) || height_min_m > height_max_m ||
	    !std::isfinite(signed_camera_altitude_m))
	{
		return false;
	}

	const Interval X = Interval::range(flat_min_x_m, flat_max_x_m);
	const Interval Z = Interval::range(flat_min_z_m, flat_max_z_m);
	const Interval H = Interval::range(height_min_m, height_max_m);

	const Interval D2 = add(square(X), square(Z));

	// Certified Envelope Check:
	// The maximum radial distance must remain within the certified deformation limit.
	// We allow a strict numerical roundoff tolerance (1.0e-7).
	constexpr double ENVELOPE_TOLERANCE = 1.0 + 1.0e-7;
	const double max_distance_limit = profile.certified_maximum_deformation_distance_m * ENVELOPE_TOLERANCE;
	const double max_d2_limit = max_distance_limit * max_distance_limit;
	if (D2.hi > max_d2_limit) {
		return false;
	}

	Interval P_x{};
	Interval P_y{};
	Interval P_z{};

	const double R = profile.radius_m;
	const double inv_R = profile.inverse_radius;
	const double inv_R2 = profile.inverse_radius_squared;

	if (profile.requested.function_class == CHPFunctionClass::QuadraticVerticalFallback) {
		P_x = X;
		P_z = Z;
		// P_y = -D2 / (2R) + H - altitude
		const Interval drop = scale(D2, 0.5 * inv_R);
		P_y = sub(sub(H, drop), Interval::point(signed_camera_altitude_m));
	} else {
		// Polynomial4 or Polynomial6
		const Interval U = scale(D2, inv_R2);
		const Interval U2 = square(U);

		Interval A{};
		Interval B{};
		Interval C{};

		if (profile.requested.function_class == CHPFunctionClass::SphericalPolynomial4) {
			// a = 1.0 - u/6.0 + u2/120.0;
			// b = u/2.0 - u2/24.0;
			// c = 1.0 - u/2.0 + u2/24.0;
			A = add(sub(Interval::point(1.0), scale(U, 1.0 / 6.0)), scale(U2, 1.0 / 120.0));
			B = sub(scale(U, 0.5), scale(U2, 1.0 / 24.0));
			C = add(sub(Interval::point(1.0), scale(U, 0.5)), scale(U2, 1.0 / 24.0));
		} else {
			// SphericalPolynomial6
			// a = 1.0 - u/6.0 + u2/120.0 - u3/5040.0;
			// b = u/2.0 - u2/24.0 + u3/720.0;
			// c = 1.0 - u/2.0 + u2/24.0 - u3/720.0;
			const Interval U3 = mul(U2, U);
			A = sub(add(sub(Interval::point(1.0), scale(U, 1.0 / 6.0)), scale(U2, 1.0 / 120.0)), scale(U3, 1.0 / 5040.0));
			B = add(sub(scale(U, 0.5), scale(U2, 1.0 / 24.0)), scale(U3, 1.0 / 720.0));
			C = sub(add(sub(Interval::point(1.0), scale(U, 0.5)), scale(U2, 1.0 / 24.0)), scale(U3, 1.0 / 720.0));
		}

		// Base position:
		// base_x = a * x
		// base_y = -R * b
		// base_z = a * z
		const Interval base_x = mul(A, X);
		const Interval base_y = scale(B, -R);
		const Interval base_z = mul(A, Z);

		// Raw axis:
		// raw_x = a * x / R
		// raw_y = c
		// raw_z = a * z / R
		const Interval raw_x = scale(base_x, inv_R);
		const Interval raw_y = C;
		const Interval raw_z = scale(base_z, inv_R);

		const Interval raw_len2 = add(add(square(raw_x), square(raw_y)), square(raw_z));
		if (raw_len2.lo <= 1.0e-15) {
			return false;
		}

		const Interval inv_len = reciprocal(sqrt_interval(raw_len2));
		const Interval axis_x = mul(raw_x, inv_len);
		const Interval axis_y = mul(raw_y, inv_len);
		const Interval axis_z = mul(raw_z, inv_len);

		// P = base + H * axis - (0, altitude, 0)
		P_x = add(base_x, mul(H, axis_x));
		P_y = sub(add(base_y, mul(H, axis_y)), Interval::point(signed_camera_altitude_m));
		P_z = add(base_z, mul(H, axis_z));
	}

	// Principled FP32 outwards padding:
	// Pad bounds to ensure strict containment of all single-precision GPU operations.
	// Machine epsilon (FP32) is ~5.96e-8. Over ~20 operations with large coordinates,
	// relative scale term (1e-6 * max_extent) + fixed epsilon (1e-3 m) guarantees containment.
	const auto calc_pad = [](Interval I) noexcept {
		const double max_val = std::max(std::abs(I.lo), std::abs(I.hi));
		return std::max(1.0e-4, 1.0e-6 * max_val + 1.0e-3);
	};

	const double pad_x = calc_pad(P_x);
	const double pad_y = calc_pad(P_y);
	const double pad_z = calc_pad(P_z);

	out_bounds.minimum_x_m = P_x.lo - pad_x;
	out_bounds.maximum_x_m = P_x.hi + pad_x;
	out_bounds.minimum_y_m = P_y.lo - pad_y;
	out_bounds.maximum_y_m = P_y.hi + pad_y;
	out_bounds.minimum_z_m = P_z.lo - pad_z;
	out_bounds.maximum_z_m = P_z.hi + pad_z;
	out_bounds.valid = true;
	return true;
}

bool try_build_conservative_curved_bounds(
	const CurvedHorizonView& view,
	double flat_min_x_m,
	double flat_max_x_m,
	double flat_min_z_m,
	double flat_max_z_m,
	double height_min_m,
	double height_max_m,
	CHPCurvedCoverageBounds& out_bounds
) noexcept {
	if (!view.chp_effective || !view.profile.is_valid()) {
		out_bounds = CHPCurvedCoverageBounds{};
		return false;
	}
	return try_build_conservative_curved_bounds(
		view.profile,
		view.signed_camera_surface_altitude_m,
		flat_min_x_m, flat_max_x_m,
		flat_min_z_m, flat_max_z_m,
		height_min_m, height_max_m,
		out_bounds
	);
}

} // namespace multinet::rendering::chp

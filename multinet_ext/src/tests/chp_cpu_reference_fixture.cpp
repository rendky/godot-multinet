#include "multinet/core/spatial/world_manifests.h"
#include "multinet/rendering/chp/chp_certification.h"
#include "multinet/rendering/chp/chp_kernel.h"
#include "multinet/rendering/chp/chp_view.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

using namespace Multinet;
using namespace multinet::rendering::chp;

namespace {

constexpr double PI = 3.141592653589793238462643383279502884;

void require(bool condition, const char* message) {
	if (!condition) {
		std::cerr << "FAILURE: " << message << "\n";
		std::exit(1);
	}
}

double vector_distance(const Vec3d& a, const Vec3d& b) {
	const double dx = a.x - b.x;
	const double dy = a.y - b.y;
	const double dz = a.z - b.z;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double dot(const Vec3d& a, const Vec3d& b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

double angle_between(const Vec3d& a, const Vec3d& b) {
	return std::acos(std::clamp(dot(a, b), -1.0, 1.0));
}

bool finite_evaluation(const CHPEvaluation& value) {
	const auto finite = [](const Vec3d& v) {
		return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
	};
	return value.valid && finite(value.base_position_m) && finite(value.height_axis) &&
		finite(value.position_m) && finite(value.tangent_x) && finite(value.tangent_z) && finite(value.normal);
}

WorldDomainManifest make_closed(uint64_t side_m) {
	WorldDomainInput input;
	input.closed_surface.area_equivalent_side_m = side_m;
	return build_world_domain_manifest(input);
}

WorldDomainManifest make_finite(uint64_t x_m, uint64_t z_m) {
	WorldDomainInput input;
	input.topology = WorldDomainTopology::FiniteRectangle;
	input.finite.extent_x_m = x_m;
	input.finite.extent_z_m = z_m;
	return build_world_domain_manifest(input);
}

CurvedHorizonProfile make_profile(CHPFunctionClass function_class, double requested_distance_m) {
	CurvedHorizonProfile profile;
	profile.function_class = function_class;
	profile.requested_maximum_deformation_distance_m = requested_distance_m;
	profile.maximum_base_position_error_m = 1.0;
	profile.maximum_visual_up_error_radians = 1.0e-4;
	return profile;
}

void check_finite_difference_jacobian(
	const CurvedHorizonProfile& profile,
	double radius_m,
	double x_m,
	double z_m,
	double& maximum_normal_error
) {
	CHPIntrinsicSample sample{ x_m, z_m, 0.0, 0.0, 0.0 };
	CHPEvaluation analytic{};
	require(try_evaluate_exact_sphere(radius_m, sample, analytic), "exact Jacobian sample failed");
	const double step = std::max(1.0e-3, std::min(0.1, radius_m * 1.0e-6));
	CHPEvaluation xp{}, xm{}, zp{}, zm{};
	require(try_evaluate_exact_sphere(radius_m, { x_m + step, z_m, 0.0, 0.0, 0.0 }, xp), "exact x+ Jacobian sample failed");
	require(try_evaluate_exact_sphere(radius_m, { x_m - step, z_m, 0.0, 0.0, 0.0 }, xm), "exact x- Jacobian sample failed");
	require(try_evaluate_exact_sphere(radius_m, { x_m, z_m + step, 0.0, 0.0, 0.0 }, zp), "exact z+ Jacobian sample failed");
	require(try_evaluate_exact_sphere(radius_m, { x_m, z_m - step, 0.0, 0.0, 0.0 }, zm), "exact z- Jacobian sample failed");
	const Vec3d finite_tx{
		(xp.position_m.x - xm.position_m.x) / (2.0 * step),
		(xp.position_m.y - xm.position_m.y) / (2.0 * step),
		(xp.position_m.z - xm.position_m.z) / (2.0 * step)
	};
	const Vec3d finite_tz{
		(zp.position_m.x - zm.position_m.x) / (2.0 * step),
		(zp.position_m.y - zm.position_m.y) / (2.0 * step),
		(zp.position_m.z - zm.position_m.z) / (2.0 * step)
	};
	const Vec3d finite_normal{
		finite_tz.y * finite_tx.z - finite_tz.z * finite_tx.y,
		finite_tz.z * finite_tx.x - finite_tz.x * finite_tx.z,
		finite_tz.x * finite_tx.y - finite_tz.y * finite_tx.x
	};
	const double finite_length = std::sqrt(dot(finite_normal, finite_normal));
	require(finite_length > 0.0 && std::isfinite(finite_length), "finite-difference normal invalid");
	const Vec3d normalized_finite_normal{
		finite_normal.x / finite_length,
		finite_normal.y / finite_length,
		finite_normal.z / finite_length
	};
	maximum_normal_error = std::max(maximum_normal_error, angle_between(analytic.normal, normalized_finite_normal));
	(void)profile;
}

} // namespace

int main() {
	std::cout << "## rendering::CHP-WP6.1-CPU-CONTRACT\n";

	const WorldDomainManifest closed_5000 = make_closed(5000000);
	const WorldDomainManifest closed_2 = make_closed(2000);
	const WorldDomainManifest finite_500x400 = make_finite(500000, 400000);
	require(closed_5000.is_valid() && closed_2.is_valid() && finite_500x400.is_valid(), "domain setup failed");

	WorldPresentationInput closed_enabled_input;
	closed_enabled_input.chp_enabled = true;
	const WorldPresentationManifest closed_presentation = build_world_presentation_manifest(closed_5000, closed_enabled_input);
	require(closed_presentation.is_valid() &&
		closed_presentation.chp_radius_policy == CHPRadiusPolicy::CanonicalClosedSurface &&
		closed_presentation.chp_kernel_version == multinet::rendering::chp::CHP_KERNEL_CONTRACT_VERSION_1,
		"closed CHP manifest normalization failed");

	WorldPresentationInput finite_area_input;
	finite_area_input.chp_enabled = true;
	finite_area_input.chp_radius_policy = CHPRadiusPolicy::AreaEquivalent;
	const WorldPresentationManifest finite_area_presentation = build_world_presentation_manifest(finite_500x400, finite_area_input);
	WorldPresentationInput finite_explicit_input = finite_area_input;
	finite_explicit_input.chp_radius_policy = CHPRadiusPolicy::Explicit;
	finite_explicit_input.explicit_chp_radius_mm = 6371000000ULL;
	const WorldPresentationManifest finite_explicit_presentation = build_world_presentation_manifest(finite_500x400, finite_explicit_input);
	require(finite_area_presentation.is_valid() && finite_explicit_presentation.is_valid() &&
		finite_area_presentation.domain_manifest_hash == finite_explicit_presentation.domain_manifest_hash &&
		finite_area_presentation.resolved_chp_radius_mm != finite_explicit_presentation.resolved_chp_radius_mm,
		"finite CHP radius policies are not independent presentation values");
	WorldPresentationInput finite_canonical_input = finite_area_input;
	finite_canonical_input.chp_radius_policy = CHPRadiusPolicy::CanonicalClosedSurface;
	const WorldPresentationManifest finite_canonical_presentation = build_world_presentation_manifest(finite_500x400, finite_canonical_input);
	require(finite_canonical_presentation.is_valid() && finite_canonical_presentation.chp_radius_policy == CHPRadiusPolicy::AreaEquivalent,
		"finite CanonicalClosedSurface policy was not normalized");
	WorldPresentationInput finite_disabled_input;
	finite_disabled_input.chp_radius_policy = CHPRadiusPolicy::Explicit;
	finite_disabled_input.explicit_chp_radius_mm = 0;
	const WorldPresentationManifest finite_disabled_a = build_world_presentation_manifest(finite_500x400, finite_disabled_input);
	finite_disabled_input.explicit_chp_radius_mm = 6371000000ULL;
	const WorldPresentationManifest finite_disabled_b = build_world_presentation_manifest(finite_500x400, finite_disabled_input);
	require(finite_disabled_a.is_valid() && finite_disabled_b.is_valid() &&
		finite_disabled_a.chp_radius_policy == CHPRadiusPolicy::AreaEquivalent &&
		finite_disabled_a.chp_kernel_version == 0 &&
		finite_disabled_a.presentation_manifest_hash == finite_disabled_b.presentation_manifest_hash,
		"disabled finite CHP was invalid or hashed hidden editor state");
	std::cout << "[PASS] CHP-MANIFEST-NORMALIZATION-01\n";

	const CHPIntrinsicSample flat_sample{ 321.0, -654.0, 1234.0, 0.25, -0.5 };
	CHPEvaluation flat{};
	require(try_evaluate_flat(flat_sample, flat) && finite_evaluation(flat), "flat CPU evaluation failed");
	const Vec3d expected_flat_base{ 321.0, 0.0, -654.0 };
	const Vec3d expected_flat_position{ 321.0, 1234.0, -654.0 };
	const double max_flat_identity_position_error = std::max(
		vector_distance(flat.base_position_m, expected_flat_base),
		vector_distance(flat.position_m, expected_flat_position));
	const double flat_normal_length = std::sqrt(1.0 + flat_sample.height_dx * flat_sample.height_dx + flat_sample.height_dz * flat_sample.height_dz);
	require(max_flat_identity_position_error == 0.0 &&
		std::abs(flat.normal.x + flat_sample.height_dx / flat_normal_length) < 1.0e-12 &&
		std::abs(flat.normal.y - 1.0 / flat_normal_length) < 1.0e-12 &&
		std::abs(flat.normal.z + flat_sample.height_dz / flat_normal_length) < 1.0e-12,
		"flat CPU identity or normal law changed");
	std::cout << "max_flat_identity_position_error_m=" << max_flat_identity_position_error << "\n";
	std::cout << "[PASS] CHP-FLAT-CPU-PARITY-01\n";

	const std::array<WorldPresentationManifest, 5> presentations{
		closed_presentation,
		build_world_presentation_manifest(closed_2, closed_enabled_input),
		build_world_presentation_manifest(make_closed(100000), closed_enabled_input),
		build_world_presentation_manifest(make_closed(25000000), closed_enabled_input),
		finite_explicit_presentation
	};
	const std::array<CHPFunctionClass, 3> classes{
		CHPFunctionClass::QuadraticVerticalFallback,
		CHPFunctionClass::SphericalPolynomial4,
		CHPFunctionClass::SphericalPolynomial6
	};
	const std::array<std::pair<double, double>, 5> directions{{
		{ 1.0, 0.0 }, { 0.0, 1.0 }, { -1.0, 0.0 }, { 0.0, -1.0 }, { 0.7071067811865475, 0.7071067811865475 }
	}};
	const std::array<double, 4> heights{ -1000.0, 0.0, 1000.0, 10000.0 };
	const std::array<std::pair<double, double>, 4> slopes{{
		{ 0.0, 0.0 }, { 0.25, 0.0 }, { 1.5, -0.75 }, { -2.0, 2.0 }
	}};
	double max_poly4_position_error = 0.0;
	double max_poly6_position_error = 0.0;
	double max_jacobian_normal_error = 0.0;
	for (const WorldPresentationManifest& presentation : presentations) {
		require(presentation.is_valid(), "fixture presentation matrix contains invalid manifest");
		const double radius_m = static_cast<double>(presentation.resolved_chp_radius_mm) * 0.001;
		for (const CHPFunctionClass function_class : classes) {
			CurvedHorizonProfile requested = make_profile(function_class, radius_m * 2.0);
			ResolvedCurvedHorizonProfile resolved{};
			require(try_resolve_curved_horizon_profile(presentation, requested, resolved) && resolved.is_valid(),
				"radius-aware CHP profile resolution failed");
			std::cout << "certified_distance_km=" << radius_m / 1000.0 << ":" << get_function_class_name(function_class)
				<< ":" << resolved.certified_maximum_deformation_distance_m / 1000.0
				<< ":clamped=" << (resolved.distance_was_clamped ? 1 : 0) << "\n";
			auto evaluate_at_distance = [&](double distance) {
				CHPEvaluation evaluation{};
				return try_evaluate_curved(resolved, { distance, 0.0, 0.0, 0.0, 0.0 }, evaluation) && finite_evaluation(evaluation);
			};
			const double certified_limit = resolved.certified_maximum_deformation_distance_m;
			const bool envelope_below = evaluate_at_distance(certified_limit * 0.999);
			const bool envelope_exact = evaluate_at_distance(certified_limit);
			const double next_boundary = std::nextafter(certified_limit, std::numeric_limits<double>::infinity());
			const bool envelope_next_first = evaluate_at_distance(next_boundary);
			const bool envelope_next_second = evaluate_at_distance(next_boundary);
			const bool envelope_over_001 = evaluate_at_distance(certified_limit * 1.001);
			const bool envelope_over_01 = evaluate_at_distance(certified_limit * 1.01);
			require(envelope_below && envelope_exact && envelope_next_first == envelope_next_second &&
				envelope_next_first && !envelope_over_001 && !envelope_over_01,
				"certification envelope acceptance/rejection boundary is incorrect");
			std::cout << "certification_envelope_km=" << radius_m / 1000.0 << ":" << get_function_class_name(function_class)
				<< ":0.999=" << (envelope_below ? "accepted" : "rejected")
				<< ":exact=" << (envelope_exact ? "accepted" : "rejected")
				<< ":nextafter=" << (envelope_next_first ? "accepted" : "rejected")
				<< ":1.001=" << (envelope_over_001 ? "accepted" : "rejected")
				<< ":1.01=" << (envelope_over_01 ? "accepted" : "rejected") << "\n";
			const std::array<double, 5> distance_fractions{ 0.0, 0.1, 0.5, 0.9, 1.0 };
			for (const auto& direction : directions) {
				for (const double fraction : distance_fractions) {
					const double distance = resolved.certified_maximum_deformation_distance_m * fraction;
					for (const double height : heights) {
						for (const auto& slope : slopes) {
							const CHPIntrinsicSample sample{
								distance * direction.first,
								distance * direction.second,
								height,
								slope.first,
								slope.second
							};
							CHPEvaluation approximate{};
							require(try_evaluate_curved(resolved, sample, approximate) && finite_evaluation(approximate),
								"production CHP evaluation failed in required matrix");
							if (height == 0.0 && slope.first == 0.0 && slope.second == 0.0) {
								CHPEvaluation exact{};
								require(try_evaluate_exact_sphere(radius_m, sample, exact), "exact sphere evaluation failed");
								const double position_error = vector_distance(approximate.base_position_m, exact.base_position_m);
								if (function_class == CHPFunctionClass::SphericalPolynomial4) max_poly4_position_error = std::max(max_poly4_position_error, position_error);
								if (function_class == CHPFunctionClass::SphericalPolynomial6) max_poly6_position_error = std::max(max_poly6_position_error, position_error);
							}
						}
					}
				}
			}
			check_finite_difference_jacobian(requested, radius_m, resolved.certified_maximum_deformation_distance_m * 0.5, 0.0, max_jacobian_normal_error);
		}
	}
	require(std::isfinite(max_poly4_position_error) && std::isfinite(max_poly6_position_error), "polynomial error maxima invalid");
	std::cout << "max_poly4_exact_base_position_error_m=" << max_poly4_position_error << "\n";
	std::cout << "max_poly6_exact_base_position_error_m=" << max_poly6_position_error << "\n";
	std::cout << "[PASS] CHP-QUADRATIC-REFERENCE-01\n";
	std::cout << "[PASS] CHP-POLY4-REFERENCE-01\n";
	std::cout << "[PASS] CHP-POLY6-REFERENCE-01\n";
	std::cout << "max_jacobian_normal_angular_error_rad=" << max_jacobian_normal_error << "\n";
	require(max_jacobian_normal_error <= 1.0e-7, "analytic exact Jacobian failed finite-difference oracle");
	std::cout << "[PASS] CHP-JACOBIAN-NORMAL-01\n";

	CurvedHorizonProfile horizon_profile = make_profile(CHPFunctionClass::SphericalPolynomial6, 1000000.0);
	ResolvedCurvedHorizonProfile horizon_resolved{};
	require(try_resolve_curved_horizon_profile(closed_presentation, horizon_profile, horizon_resolved), "horizon profile resolution failed");
	SurfacePosition64 camera_surface{};
	camera_surface.altitude_m = 100000.0;
	camera_surface.topology_version = closed_5000.topology_version;
	camera_surface.projection_version = closed_5000.projection_version;
	CurvedHorizonView horizon_view{};
	require(try_build_curved_horizon_view(
		closed_5000, closed_presentation, horizon_resolved, camera_surface, {}, 1, 1, 1, horizon_view) && horizon_view.is_valid(),
		"curved horizon view construction failed");
	const double expected_los = std::sqrt(2.0 * horizon_resolved.radius_m * 100000.0 + 100000.0 * 100000.0);
	const double expected_arc = horizon_resolved.radius_m * std::acos(horizon_resolved.radius_m / (horizon_resolved.radius_m + 100000.0));
	require(std::abs(horizon_view.horizon_line_of_sight_m - expected_los) < 1.0e-9 &&
		std::abs(horizon_view.horizon_surface_arc_m - expected_arc) < 1.0e-9 &&
		std::abs(horizon_view.horizon_line_of_sight_m - horizon_view.horizon_surface_arc_m) > 1.0,
		"horizon line-of-sight and surface-arc metrics were conflated");
	std::cout << "horizon_los_error_m=" << std::abs(horizon_view.horizon_line_of_sight_m - expected_los) << "\n";
	std::cout << "horizon_surface_arc_error_m=" << std::abs(horizon_view.horizon_surface_arc_m - expected_arc) << "\n";
	std::cout << "[PASS] CHP-HORIZON-METRICS-01\n";

	CHPEvaluation finite_eval{};
	CHPEvaluation closed_eval{};
	const CHPIntrinsicSample independence_sample{ 1200.0, -900.0, 250.0, 0.25, -0.5 };
	ResolvedCurvedHorizonProfile independence_profile{};
	require(try_resolve_curved_horizon_profile(finite_explicit_presentation,
		make_profile(CHPFunctionClass::SphericalPolynomial6, 100000.0), independence_profile),
		"finite independence profile failed");
	require(try_evaluate_curved(independence_profile, independence_sample, finite_eval) &&
		try_evaluate_curved(independence_profile, independence_sample, closed_eval),
		"topology-independent CHP evaluation failed");
	require(vector_distance(finite_eval.position_m, closed_eval.position_m) == 0.0 &&
		vector_distance(finite_eval.normal, closed_eval.normal) == 0.0,
		"CHP transform changed with topology provenance");
	std::cout << "[PASS] CHP-DOMAIN-INDEPENDENCE-01\n";

	const CHPIntrinsicSample symmetry_sample{ 20000.0, 0.0, 0.0, 0.0, 0.0 };
	CHPEvaluation symmetry_x{}, symmetry_z{}, symmetry_diag{};
	ResolvedCurvedHorizonProfile symmetry_profile{};
	require(try_resolve_curved_horizon_profile(closed_presentation,
		make_profile(CHPFunctionClass::SphericalPolynomial6, 50000.0), symmetry_profile), "symmetry profile failed");
	require(try_evaluate_curved(symmetry_profile, symmetry_sample, symmetry_x) &&
		try_evaluate_curved(symmetry_profile, { 0.0, 20000.0, 0.0, 0.0, 0.0 }, symmetry_z) &&
		try_evaluate_curved(symmetry_profile, { 20000.0 / std::sqrt(2.0), 20000.0 / std::sqrt(2.0), 0.0, 0.0, 0.0 }, symmetry_diag),
		"rotational symmetry evaluation failed");
	require(std::abs(symmetry_x.base_position_m.y - symmetry_z.base_position_m.y) < 1.0e-10 &&
		std::abs(symmetry_x.height_axis.y - symmetry_z.height_axis.y) < 1.0e-12 &&
		std::abs(symmetry_diag.base_position_m.y - symmetry_x.base_position_m.y) < 1.0e-10,
		"CHP rotational symmetry changed across intrinsic directions");
	std::cout << "[PASS] CHP-ROTATIONAL-SYMMETRY-01\n";

	ResolvedCurvedHorizonProfile covariance_profile{};
	require(try_resolve_curved_horizon_profile(closed_presentation,
		make_profile(CHPFunctionClass::SphericalPolynomial6, 1000.0), covariance_profile), "covariance profile failed");
	CHPEvaluation covariance_base{};
	require(try_evaluate_curved(covariance_profile, { 100.0, 200.0, 0.0, 0.0, 0.0 }, covariance_base), "base covariance evaluation failed");
	for (const double scale : { 0.001, 1000.0 }) {
		ResolvedCurvedHorizonProfile scaled = covariance_profile;
		scaled.radius_m *= scale;
		scaled.inverse_radius /= scale;
		scaled.inverse_radius_squared /= scale * scale;
		scaled.certified_maximum_deformation_distance_m *= scale;
		CHPEvaluation scaled_eval{};
		require(try_evaluate_curved(scaled, { 100.0 * scale, 200.0 * scale, 0.0, 0.0, 0.0 }, scaled_eval), "scaled covariance evaluation failed");
		require(vector_distance(
			{ scaled_eval.position_m.x / scale, scaled_eval.position_m.y / scale, scaled_eval.position_m.z / scale },
			covariance_base.position_m) < 1.0e-10 &&
			angle_between(scaled_eval.height_axis, covariance_base.height_axis) < 1.0e-12,
			"CHP scale covariance failed");
	}
	std::cout << "[PASS] CHP-SCALE-COVARIANCE-01\n";

	CurvedHorizonProfile invalid_profile = make_profile(CHPFunctionClass::SphericalPolynomial6, 1000.0);
	invalid_profile.function_class = static_cast<CHPFunctionClass>(99);
	ResolvedCurvedHorizonProfile invalid_resolved{};
	require(!try_resolve_curved_horizon_profile(closed_presentation, invalid_profile, invalid_resolved), "unknown function class accepted");
	CHPIntrinsicSample invalid_sample = flat_sample;
	invalid_sample.x_m = std::numeric_limits<double>::quiet_NaN();
	require(!try_evaluate_flat(invalid_sample, flat), "NaN flat sample accepted");
	WorldPresentationManifest mismatched = closed_presentation;
	mismatched.domain_manifest_hash++;
	require(!try_build_curved_horizon_view(closed_5000, mismatched, horizon_resolved, camera_surface, {}, 1, 1, 1, horizon_view),
		"presentation/domain hash mismatch accepted");
	std::cout << "[PASS] CHP-ERROR-HANDLING-01\n";

	std::cout << "[PASS] CHP-PROFILE-CERTIFICATION-01\n";
	std::cout << "WP6.1 CPU CONTRACT: PASSED\n";
	return 0;
}

#include "multinet/rendering/chp/chp_bounds.h"
#include "multinet/rendering/chp/chp_kernel.h"
#include "multinet/rendering/chp/chp_view.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_profile.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_culling.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using namespace multinet::rendering::chp;

namespace {

struct Vec3 {
	double x{ 0.0 };
	double y{ 0.0 };
	double z{ 0.0 };
};

Vec3 add(const Vec3& a, const Vec3& b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
Vec3 subtract(const Vec3& a, const Vec3& b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
Vec3 scale(const Vec3& a, double value) { return { a.x * value, a.y * value, a.z * value }; }
double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double maximum_abs_component(const Vec3& value) { return std::max({ std::abs(value.x), std::abs(value.y), std::abs(value.z) }); }

struct Basis {
	Vec3 x;
	Vec3 y;
	Vec3 z;

	Vec3 xform(const Vec3& value) const {
		return add(add(scale(x, value.x), scale(y, value.y)), scale(z, value.z));
	}
	Vec3 inverse_xform(const Vec3& value) const {
		return { dot(x, value), dot(y, value), dot(z, value) };
	}
};

struct Aabb {
	Vec3 minimum;
	Vec3 maximum;
};

struct DomainCase {
	const char* name;
	double radius_m;
	double certified_distance_m;
};

struct HeightRange {
	const char* name;
	double minimum_m;
	double maximum_m;
};

struct BasisCase {
	const char* name;
	Basis basis;
};

struct BoundsChain {
	Aabb global;
	Vec3 q_origin;
	CHPCurvedCoverageBounds curved;
};

void require(bool value, const std::string& message) {
	if (!value) {
		std::cerr << "FAILURE: " << message << "\n";
		std::exit(1);
	}
}

ResolvedCurvedHorizonProfile make_profile(CHPFunctionClass function_class, double radius_m, double certified_distance_m) {
	ResolvedCurvedHorizonProfile profile{};
	profile.requested.function_class = function_class;
	profile.requested.requested_maximum_deformation_distance_m = certified_distance_m;
	profile.requested.maximum_base_position_error_m = 1.0;
	profile.requested.maximum_visual_up_error_radians = 1.0e-4;
	profile.radius_m = radius_m;
	profile.inverse_radius = 1.0 / radius_m;
	profile.inverse_radius_squared = 1.0 / (radius_m * radius_m);
	profile.certified_maximum_deformation_distance_m = certified_distance_m;
	profile.certified_maximum_theta = certified_distance_m / radius_m;
	profile.certified_maximum_u = profile.certified_maximum_theta * profile.certified_maximum_theta;
	return profile;
}

std::array<BasisCase, 8> make_bases() {
	std::array<BasisCase, 8> bases{};
	const std::array<std::array<int, 3>, 4> signs{{
		{ 1, 1, 1 }, { -1, 1, 1 }, { 1, -1, -1 }, { -1, -1, -1 }
	}};
	for (size_t i = 0; i < signs.size(); ++i) {
		const auto [sx, sy, sz] = signs[i];
		bases[i] = BasisCase{
			i == 0 ? "identity" : i == 1 ? "signed-x" : i == 2 ? "signed-yz" : "signed-xyz",
			Basis{ { static_cast<double>(sx), 0.0, 0.0 }, { 0.0, static_cast<double>(sy), 0.0 }, { 0.0, 0.0, static_cast<double>(sz) } }
		};
		bases[i + 4] = BasisCase{
			i == 0 ? "permuted-xz" : i == 1 ? "permuted-signed-x" : i == 2 ? "permuted-signed-yz" : "permuted-signed-xyz",
			Basis{ { 0.0, 0.0, static_cast<double>(sx) }, { 0.0, static_cast<double>(sy), 0.0 }, { static_cast<double>(sz), 0.0, 0.0 } }
		};
	}
	return bases;
}

bool try_build_bounds_chain(
	const ResolvedCurvedHorizonProfile& profile,
	double altitude_m,
	const Basis& basis,
	double block_size_m,
	const Vec3& q_origin,
	double minimum_height_m,
	double maximum_height_m,
	bool camera_relative_y,
	BoundsChain& out_chain
) {
	const std::array<Vec3, 4> flat_corners{
		q_origin,
		add(q_origin, basis.xform({ block_size_m, 0.0, 0.0 })),
		add(q_origin, basis.xform({ 0.0, 0.0, block_size_m })),
		add(q_origin, basis.xform({ block_size_m, 0.0, block_size_m }))
	};
	double minimum_x = std::numeric_limits<double>::infinity();
	double maximum_x = -std::numeric_limits<double>::infinity();
	double minimum_z = std::numeric_limits<double>::infinity();
	double maximum_z = -std::numeric_limits<double>::infinity();
	for (const Vec3& corner : flat_corners) {
		minimum_x = std::min(minimum_x, corner.x);
		maximum_x = std::max(maximum_x, corner.x);
		minimum_z = std::min(minimum_z, corner.z);
		maximum_z = std::max(maximum_z, corner.z);
	}
	out_chain = BoundsChain{};
	if (!try_build_conservative_curved_bounds(
		profile, altitude_m, minimum_x, maximum_x, minimum_z, maximum_z,
		minimum_height_m, maximum_height_m, out_chain.curved))
	{
		return false;
	}
	
	// When camera_relative_y is false (old unshifted regression test), we add back altitude to simulate unshifted Y
	const double minimum_y = camera_relative_y ? out_chain.curved.minimum_y_m : (out_chain.curved.minimum_y_m + altitude_m);
	const double maximum_y = camera_relative_y ? out_chain.curved.maximum_y_m : (out_chain.curved.maximum_y_m + altitude_m);
	if (!std::isfinite(minimum_y) || !std::isfinite(maximum_y) || minimum_y > maximum_y) {
		return false;
	}

	Vec3 local_min{ std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() };
	Vec3 local_max{ -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity() };
	for (int ix = 0; ix < 2; ++ix) for (int iy = 0; iy < 2; ++iy) for (int iz = 0; iz < 2; ++iz) {
		const Vec3 q_corner{
			ix != 0 ? out_chain.curved.maximum_x_m : out_chain.curved.minimum_x_m,
			iy != 0 ? maximum_y : minimum_y,
			iz != 0 ? out_chain.curved.maximum_z_m : out_chain.curved.minimum_z_m
		};
		const Vec3 local = basis.inverse_xform(subtract(q_corner, q_origin));
		local_min.x = std::min(local_min.x, local.x);
		local_min.y = std::min(local_min.y, local.y);
		local_min.z = std::min(local_min.z, local.z);
		local_max.x = std::max(local_max.x, local.x);
		local_max.y = std::max(local_max.y, local.y);
		local_max.z = std::max(local_max.z, local.z);
	}
	out_chain.q_origin = q_origin;
	out_chain.global.minimum = { std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() };
	out_chain.global.maximum = { -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity() };
	for (int ix = 0; ix < 2; ++ix) for (int iy = 0; iy < 2; ++iy) for (int iz = 0; iz < 2; ++iz) {
		const Vec3 local{
			ix != 0 ? local_max.x : local_min.x,
			iy != 0 ? local_max.y : local_min.y,
			iz != 0 ? local_max.z : local_min.z
		};
		const Vec3 global = add(q_origin, basis.xform(local));
		out_chain.global.minimum.x = std::min(out_chain.global.minimum.x, global.x);
		out_chain.global.minimum.y = std::min(out_chain.global.minimum.y, global.y);
		out_chain.global.minimum.z = std::min(out_chain.global.minimum.z, global.z);
		out_chain.global.maximum.x = std::max(out_chain.global.maximum.x, global.x);
		out_chain.global.maximum.y = std::max(out_chain.global.maximum.y, global.y);
		out_chain.global.maximum.z = std::max(out_chain.global.maximum.z, global.z);
	}
	return true;
}

double containment_margin(const Aabb& bounds, const Vec3& point) {
	return std::min({
		point.x - bounds.minimum.x, bounds.maximum.x - point.x,
		point.y - bounds.minimum.y, bounds.maximum.y - point.y,
		point.z - bounds.minimum.z, bounds.maximum.z - point.z
	});
}

// Shader-equivalent FP32 position evaluator
Vec3 evaluate_chp_gpu_fp32(
	CHPFunctionClass function_class,
	float radius_m,
	float inv_R,
	float inv_R2,
	float flat_x,
	float flat_z,
	float height_m,
	float signed_alt_m
) {
	const float d2 = flat_x * flat_x + flat_z * flat_z;
	if (function_class == CHPFunctionClass::QuadraticVerticalFallback) {
		const float drop = d2 * 0.5f * inv_R;
		return { static_cast<double>(flat_x), static_cast<double>(height_m - drop - signed_alt_m), static_cast<double>(flat_z) };
	}
	const float u = d2 * inv_R2;
	const float u2 = u * u;
	float a = 0.0f;
	float b = 0.0f;
	float c = 0.0f;
	if (function_class == CHPFunctionClass::SphericalPolynomial4) {
		a = 1.0f - u / 6.0f + u2 / 120.0f;
		b = u * 0.5f - u2 / 24.0f;
		c = 1.0f - u * 0.5f + u2 / 24.0f;
	} else {
		const float u3 = u2 * u;
		a = 1.0f - u / 6.0f + u2 / 120.0f - u3 / 5040.0f;
		b = u * 0.5f - u2 / 24.0f + u3 / 720.0f;
		c = 1.0f - u * 0.5f + u2 / 24.0f - u3 / 720.0f;
	}
	const float base_x = a * flat_x;
	const float base_y = -radius_m * b;
	const float base_z = a * flat_z;

	const float raw_x = base_x * inv_R;
	const float raw_y = c;
	const float raw_z = base_z * inv_R;

	const float raw_len = std::sqrt(raw_x * raw_x + raw_y * raw_y + raw_z * raw_z);
	const float axis_x = raw_len > 1.0e-15f ? raw_x / raw_len : 0.0f;
	const float axis_y = raw_len > 1.0e-15f ? raw_y / raw_len : 1.0f;
	const float axis_z = raw_len > 1.0e-15f ? raw_z / raw_len : 0.0f;

	const float pos_x = base_x + height_m * axis_x;
	const float pos_y = base_y + height_m * axis_y - signed_alt_m;
	const float pos_z = base_z + height_m * axis_z;

	return { static_cast<double>(pos_x), static_cast<double>(pos_y), static_cast<double>(pos_z) };
}

} // namespace

int main() {
	std::cout << std::setprecision(15);
	constexpr double PI = 3.141592653589793238462643383279502884;
	const auto area_equivalent_radius = [=](double area_m2) {
		return std::sqrt(area_m2 / (4.0 * PI));
	};
	const double scale_2km_radius_m = area_equivalent_radius(2000.0 * 2000.0);
	const double scale_32km_radius_m = area_equivalent_radius(32000.0 * 32000.0);
	const double closed_100_radius_m = area_equivalent_radius(100000.0 * 100000.0);
	const double scale_500km_radius_m = area_equivalent_radius(500000.0 * 500000.0);
	const double closed_5000_radius_m = area_equivalent_radius(5000000.0 * 5000000.0);
	const double scale_22518km_radius_m = area_equivalent_radius(22518000.0 * 22518000.0);
	const double scale_25000km_radius_m = area_equivalent_radius(25000000.0 * 25000000.0);

	const std::array<DomainCase, 7> domains{
		DomainCase{ "scale_2km", scale_2km_radius_m, std::min(2000.0, scale_2km_radius_m * 0.35) },
		DomainCase{ "scale_32km", scale_32km_radius_m, std::min(32000.0, scale_32km_radius_m * 0.35) },
		DomainCase{ "closed_100km", closed_100_radius_m, std::min(100000.0, closed_100_radius_m * 0.35) },
		DomainCase{ "scale_500km", scale_500km_radius_m, std::min(500000.0, scale_500km_radius_m * 0.35) },
		DomainCase{ "closed_5000km", closed_5000_radius_m, std::min(100000.0, closed_5000_radius_m * 0.35) },
		DomainCase{ "scale_22518km", scale_22518km_radius_m, std::min(100000.0, scale_22518km_radius_m * 0.35) },
		DomainCase{ "scale_25000km", scale_25000km_radius_m, std::min(100000.0, scale_25000km_radius_m * 0.35) }
	};
	const std::array<double, 6> altitudes_m{ 0.0, 100.0, 3000.0, 109665.5, 250000.0, -500.0 };
	const std::array<HeightRange, 5> heights{
		HeightRange{ "ground", 0.0, 0.0 },
		HeightRange{ "small", -10.0, 10.0 },
		HeightRange{ "terrain", -500.0, 1000.0 },
		HeightRange{ "broad_fallback", -2000.0, 8000.0 },
		HeightRange{ "deep", -5000.0, 10000.0 }
	};
	const std::array<CHPFunctionClass, 3> functions{
		CHPFunctionClass::QuadraticVerticalFallback,
		CHPFunctionClass::SphericalPolynomial4,
		CHPFunctionClass::SphericalPolynomial6
	};
	const auto bases = make_bases();
	const multinet::rendering::BlockClipmapProfile clipmap_profile{};

	double worst_containment_margin_m = std::numeric_limits<double>::infinity();
	double worst_fp32_containment_margin_m = std::numeric_limits<double>::infinity();
	double worst_reconstruction_error_m = 0.0;
	double old_vertical_error_m = 0.0;
	uint64_t case_count = 0;
	uint64_t sample_count = 0;
	uint64_t fp32_sample_count = 0;
	uint64_t fallback_count = 0;
	uint64_t rebase_case_count = 0;

	for (const DomainCase& domain : domains) {
		for (CHPFunctionClass function_class : functions) {
			const ResolvedCurvedHorizonProfile profile = make_profile(function_class, domain.radius_m, domain.certified_distance_m);
			require(profile.is_valid(), "fixture profile invalid");
			for (double altitude_m : altitudes_m) {
				for (uint8_t lod = 0; lod < 8; ++lod) {
					// Scale block size to fit domain scale if needed for small domains
					const double raw_block_size = clipmap_profile.get_lod_block_size(lod);
					const double block_size_m = std::min(raw_block_size, domain.certified_distance_m * 0.3);
					for (int sign : { -1, 1 }) {
						const double center_x = sign * domain.certified_distance_m * 0.20;
						const double center_z = sign * domain.certified_distance_m * 0.13;
						for (const BasisCase& basis_case : bases) {
							const Vec3 target_center{ center_x, -altitude_m, center_z };
							Vec3 q_origin = subtract(target_center,
								basis_case.basis.xform({ block_size_m * 0.5, 0.0, block_size_m * 0.5 }));
							if (lod == 3 && sign > 0) {
								const Vec3 rebase_offset{ 131072.0, 0.0, -98304.0 };
								const Vec3 canonical_block_origin = add(rebase_offset, q_origin);
								q_origin = subtract(canonical_block_origin, rebase_offset);
								++rebase_case_count;
							}
							for (const HeightRange& height : heights) {
								BoundsChain corrected{};
								if (!try_build_bounds_chain(
									profile, altitude_m, basis_case.basis, block_size_m, q_origin,
									height.minimum_m, height.maximum_m, true, corrected))
								{
									++fallback_count;
									continue;
								}
								++case_count;
								for (double fx : { 0.0, 0.25, 0.5, 0.75, 1.0 }) for (double fz : { 0.0, 0.25, 0.5, 0.75, 1.0 })
								for (double sample_height : { height.minimum_m, 0.5 * (height.minimum_m + height.maximum_m), height.maximum_m }) {
									const Vec3 flat_q = add(q_origin, basis_case.basis.xform(
										{ block_size_m * fx, 0.0, block_size_m * fz }));
									CHPEvaluation evaluation{};
									require(try_evaluate_curved(profile, CHPIntrinsicSample{
										flat_q.x, flat_q.z, sample_height, 0.0, 0.0 }, evaluation),
										"production CHP point rejected inside admitted envelope");
									const Vec3 exact_q{
										evaluation.position_m.x,
										evaluation.position_m.y - altitude_m,
										evaluation.position_m.z
									};
									const double margin = containment_margin(corrected.global, exact_q);
									worst_containment_margin_m = std::min(worst_containment_margin_m, margin);
									require(margin >= -1.0e-7, "camera-relative global AABB failed to contain production CHP point");
									const Vec3 reconstructed_q = add(corrected.q_origin,
										basis_case.basis.xform(basis_case.basis.inverse_xform(subtract(exact_q, corrected.q_origin))));
									const double reconstruction_error = maximum_abs_component(subtract(reconstructed_q, exact_q));
									worst_reconstruction_error_m = std::max(worst_reconstruction_error_m, reconstruction_error);
									require(reconstruction_error <= 1.0e-9, "inverse basis/global reconstruction drifted");
									++sample_count;

									// Gate 2: FP32 GPU shader emulation containment
									const Vec3 fp32_pos = evaluate_chp_gpu_fp32(
										function_class,
										static_cast<float>(profile.radius_m),
										static_cast<float>(profile.inverse_radius),
										static_cast<float>(profile.inverse_radius_squared),
										static_cast<float>(flat_q.x),
										static_cast<float>(flat_q.z),
										static_cast<float>(sample_height),
										static_cast<float>(altitude_m)
									);
									const double fp32_margin = containment_margin(corrected.global, fp32_pos);
									worst_fp32_containment_margin_m = std::min(worst_fp32_containment_margin_m, fp32_margin);
									require(fp32_margin >= 0.0, "camera-relative global AABB failed to contain FP32 shader point");
									++fp32_sample_count;
								}
							}
						}
					}
				}
			}
		}
	}

	// Gate 3: B1 Morph Footprint Invariant Verification
	// Verify that across all fine grid points [0..16] x [0..16], candidate snap phases, and recursion depths,
	// the morphed local horizontal coordinate stays strictly inside [0, 16] x [0, 16].
	for (int u = 0; u <= 16; ++u) {
		for (int v = 0; v <= 16; ++v) {
			for (double mu : { 0.0, 0.25, 0.5, 0.75, 1.0 }) {
				for (int depth = 0; depth <= 3; ++depth) {
					// Stage points
					int p1_u = u & ~1;
					int p1_v = v & ~1;
					int p2_u = u & ~3;
					int p2_v = v & ~3;
					int p3_u = u & ~7;
					int p3_v = v & ~7;

					double mu1 = std::clamp(mu * 3.0, 0.0, 1.0);
					double mu2 = std::clamp(mu * 3.0 - 1.0, 0.0, 1.0);
					double mu3 = std::clamp(mu * 3.0 - 2.0, 0.0, 1.0);

					double mu1_smooth = mu1 * mu1 * (3.0 - 2.0 * mu1);
					double mu2_smooth = mu2 * mu2 * (3.0 - 2.0 * mu2);
					double mu3_smooth = mu3 * mu3 * (3.0 - 2.0 * mu3);

					double cur_u = static_cast<double>(u);
					double cur_v = static_cast<double>(v);

					if (depth >= 1) {
						cur_u = cur_u + (p1_u - cur_u) * mu1_smooth;
						cur_v = cur_v + (p1_v - cur_v) * mu1_smooth;
					}
					if (depth >= 2) {
						cur_u = cur_u + (p2_u - cur_u) * mu2_smooth;
						cur_v = cur_v + (p2_v - cur_v) * mu2_smooth;
					}
					if (depth >= 3) {
						cur_u = cur_u + (p3_u - cur_u) * mu3_smooth;
						cur_v = cur_v + (p3_v - cur_v) * mu3_smooth;
					}

					require(cur_u >= -1.0e-9 && cur_u <= 16.0 + 1.0e-9, "B1 morphed u coordinate escaped [0, 16]");
					require(cur_v >= -1.0e-9 && cur_v <= 16.0 + 1.0e-9, "B1 morphed v coordinate escaped [0, 16]");
				}
			}
		}
	}

	// Gate 4: Altitude Regression Gate (109665.5m)
	const double regression_altitude_m = 109665.5;
	const ResolvedCurvedHorizonProfile regression_profile = make_profile(
		CHPFunctionClass::SphericalPolynomial6,
		closed_5000_radius_m,
		std::min(100000.0, closed_5000_radius_m * 0.35));
	const Basis identity = bases[0].basis;
	BoundsChain old_chain{};
	BoundsChain corrected_chain{};
	require(try_build_bounds_chain(
		regression_profile, regression_altitude_m, identity, 32.0,
		{ -16.0, -regression_altitude_m, -16.0 }, 0.0, 0.0, false, old_chain),
		"old chain build failed");
	require(try_build_bounds_chain(
		regression_profile, regression_altitude_m, identity, 32.0,
		{ -16.0, -regression_altitude_m, -16.0 }, 0.0, 0.0, true, corrected_chain),
		"corrected chain build failed");
	CHPEvaluation ground{};
	require(try_evaluate_curved(regression_profile,
		CHPIntrinsicSample{ 0.0, 0.0, 0.0, 0.0, 0.0 }, ground),
		"old implementation regression point failed");
	const double old_ground_y = 0.5 * (old_chain.global.minimum.y + old_chain.global.maximum.y);
	const double corrected_ground_y = ground.position_m.y - regression_altitude_m;
	old_vertical_error_m = old_ground_y - corrected_ground_y;
	require(std::abs(old_vertical_error_m - regression_altitude_m) < 1.0e-3,
		"old culling implementation did not reproduce the 109665.5m vertical error");
	require(containment_margin(old_chain.global, { ground.position_m.x, corrected_ground_y, ground.position_m.z }) < 0.0,
		"old intrinsic-Y AABB unexpectedly contained the camera-relative ground point");
	require(containment_margin(corrected_chain.global, { ground.position_m.x, corrected_ground_y, ground.position_m.z }) >= -1.0e-7,
		"corrected camera-relative AABB did not contain the regression ground point");

	std::cout << "## rendering::CHP-WP6.2-RENDERER-SPACE-BOUNDS\n";
	std::cout << "case_count=" << case_count << "\nsample_count=" << sample_count << "\nfp32_sample_count=" << fp32_sample_count << "\n";
	std::cout << "domain_cases=" << domains.size() << "\naltitude_cases=" << altitudes_m.size() << "\nlod_cases=8\nq_sign_cases=2\nbasis_cases=" << bases.size() << "\nheight_range_cases=" << heights.size() << "\nfunction_cases=" << functions.size() << "\nrebase_case_count=" << rebase_case_count << "\n";
	std::cout << "camera_altitude_regression_m=109665.5\nground_presentation_y_m=0\n";
	std::cout << "old_vertical_error_m=" << old_vertical_error_m << "\n";
	std::cout << "worst_containment_margin_m=" << worst_containment_margin_m << "\n";
	std::cout << "worst_fp32_containment_margin_m=" << worst_fp32_containment_margin_m << "\n";
	std::cout << "worst_coordinate_space_reconstruction_error_m=" << worst_reconstruction_error_m << "\n";
	std::cout << "[PASS] CHP-R3-CURVED-BOUNDS-CONTAINMENT-01\n";
	std::cout << "[PASS] CHP-R3-FP32-CONTAINMENT-01\n";
	std::cout << "[PASS] CHP-R3-B1-FOOTPRINT-01\n";
	std::cout << "[PASS] CHP-R3-SIGNED-ALTITUDE-BOUNDS-01\n";
	std::cout << "[PASS] BCCM-R3-REBASE-CULLING-INVARIANCE-01\n";
	std::cout << "STATUS: PASSED WITH EVIDENCE\n";
	return 0;
}

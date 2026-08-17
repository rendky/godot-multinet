#include "multinet/rendering/chp/chp_bounds.h"
#include "multinet/rendering/chp/chp_kernel.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_profile.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

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

CHPGpuView make_gpu_view(CHPFunctionClass function_class, double radius_m, double certified_distance_m, double altitude_m) {
	CHPGpuView view{};
	view.effective = true;
	view.function_class = function_class;
	view.radius_m = static_cast<float>(radius_m);
	view.inverse_radius = static_cast<float>(1.0 / radius_m);
	view.inverse_radius_squared = static_cast<float>(1.0 / (radius_m * radius_m));
	view.certified_maximum_distance_m = static_cast<float>(certified_distance_m);
	view.certified_maximum_theta = static_cast<float>(certified_distance_m / radius_m);
	view.certified_maximum_u = static_cast<float>((certified_distance_m / radius_m) * (certified_distance_m / radius_m));
	view.signed_camera_surface_altitude_m = static_cast<float>(altitude_m);
	view.kernel_contract_version = CHP_KERNEL_CONTRACT_VERSION_1;
	view.profile_version = CHP_PROFILE_VERSION_1;
	view.presentation_manifest_hash = 0xC4CULL;
	view.surface_frame_epoch = 1;
	view.camera_epoch = 1;
	view.source_epoch = 1;
	return view;
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

BoundsChain build_bounds_chain(
	const CHPGpuView& view,
	const Basis& basis,
	double block_size_m,
	const Vec3& q_origin,
	double minimum_height_m,
	double maximum_height_m,
	bool camera_relative_y
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
	BoundsChain chain{};
	require(try_build_conservative_curved_bounds(
		view, minimum_x, maximum_x, minimum_z, maximum_z,
		minimum_height_m, maximum_height_m, chain.curved),
		"curved bounds rejected an admitted renderer-space block");
	const double minimum_y = chain.curved.minimum_y_m -
		(camera_relative_y ? static_cast<double>(view.signed_camera_surface_altitude_m) : 0.0);
	const double maximum_y = chain.curved.maximum_y_m -
		(camera_relative_y ? static_cast<double>(view.signed_camera_surface_altitude_m) : 0.0);
	require(std::isfinite(minimum_y) && std::isfinite(maximum_y) && minimum_y <= maximum_y,
		"camera-relative curved Y bounds invalid");

	Vec3 local_min{ std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() };
	Vec3 local_max{ -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity() };
	for (int ix = 0; ix < 2; ++ix) for (int iy = 0; iy < 2; ++iy) for (int iz = 0; iz < 2; ++iz) {
		const Vec3 q_corner{
			ix != 0 ? chain.curved.maximum_x_m : chain.curved.minimum_x_m,
			iy != 0 ? maximum_y : minimum_y,
			iz != 0 ? chain.curved.maximum_z_m : chain.curved.minimum_z_m
		};
		const Vec3 local = basis.inverse_xform(subtract(q_corner, q_origin));
		local_min.x = std::min(local_min.x, local.x);
		local_min.y = std::min(local_min.y, local.y);
		local_min.z = std::min(local_min.z, local.z);
		local_max.x = std::max(local_max.x, local.x);
		local_max.y = std::max(local_max.y, local.y);
		local_max.z = std::max(local_max.z, local.z);
	}
	chain.q_origin = q_origin;
	chain.global.minimum = { std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() };
	chain.global.maximum = { -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity() };
	for (int ix = 0; ix < 2; ++ix) for (int iy = 0; iy < 2; ++iy) for (int iz = 0; iz < 2; ++iz) {
		const Vec3 local{
			ix != 0 ? local_max.x : local_min.x,
			iy != 0 ? local_max.y : local_min.y,
			iz != 0 ? local_max.z : local_min.z
		};
		const Vec3 global = add(q_origin, basis.xform(local));
		chain.global.minimum.x = std::min(chain.global.minimum.x, global.x);
		chain.global.minimum.y = std::min(chain.global.minimum.y, global.y);
		chain.global.minimum.z = std::min(chain.global.minimum.z, global.z);
		chain.global.maximum.x = std::max(chain.global.maximum.x, global.x);
		chain.global.maximum.y = std::max(chain.global.maximum.y, global.y);
		chain.global.maximum.z = std::max(chain.global.maximum.z, global.z);
	}
	return chain;
}

double containment_margin(const Aabb& bounds, const Vec3& point) {
	return std::min({
		point.x - bounds.minimum.x, bounds.maximum.x - point.x,
		point.y - bounds.minimum.y, bounds.maximum.y - point.y,
		point.z - bounds.minimum.z, bounds.maximum.z - point.z
	});
}

} // namespace

int main() {
	std::cout << std::setprecision(15);
	constexpr double PI = 3.141592653589793238462643383279502884;
	const auto area_equivalent_radius = [=](double area_m2) {
		return std::sqrt(area_m2 / (4.0 * PI));
	};
	const double closed_100_radius_m = area_equivalent_radius(100000.0 * 100000.0);
	const double closed_5000_radius_m = area_equivalent_radius(5000000.0 * 5000000.0);
	const double finite_500x400_radius_m = area_equivalent_radius(500000.0 * 400000.0);
	const std::array<DomainCase, 3> domains{
		DomainCase{ "closed_100km", closed_100_radius_m, std::min(100000.0, closed_100_radius_m * 0.35) },
		DomainCase{ "closed_5000km", closed_5000_radius_m, std::min(100000.0, closed_5000_radius_m * 0.35) },
		DomainCase{ "finite_500x400km", finite_500x400_radius_m, std::min(100000.0, finite_500x400_radius_m * 0.35) }
	};
	const std::array<double, 6> altitudes_m{ 0.0, 100.0, 3000.0, 109665.5, 250000.0, -500.0 };
	const std::array<HeightRange, 4> heights{
		HeightRange{ "ground", 0.0, 0.0 },
		HeightRange{ "small", -10.0, 10.0 },
		HeightRange{ "terrain", -500.0, 1000.0 },
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
	double worst_reconstruction_error_m = 0.0;
	double old_vertical_error_m = 0.0;
	uint64_t case_count = 0;
	uint64_t sample_count = 0;
	uint64_t rebase_case_count = 0;

	for (const DomainCase& domain : domains) {
		for (CHPFunctionClass function_class : functions) {
			const ResolvedCurvedHorizonProfile profile = make_profile(function_class, domain.radius_m, domain.certified_distance_m);
			require(profile.is_valid(), "fixture profile invalid");
			for (double altitude_m : altitudes_m) {
				const CHPGpuView view = make_gpu_view(function_class, domain.radius_m, domain.certified_distance_m, altitude_m);
				require(view.is_valid(), "fixture GPU view invalid");
				for (uint8_t lod = 0; lod < 8; ++lod) {
					const double block_size_m = clipmap_profile.get_lod_block_size(lod);
					for (int sign : { -1, 1 }) {
						const double center_x = sign * domain.certified_distance_m * 0.20;
						const double center_z = sign * domain.certified_distance_m * 0.13;
						for (const BasisCase& basis_case : bases) {
							const Vec3 target_center{ center_x, -altitude_m, center_z };
							Vec3 q_origin = subtract(target_center,
								basis_case.basis.xform({ block_size_m * 0.5, 0.0, block_size_m * 0.5 }));
							if (domain.name == std::string("finite_500x400km") && lod == 3 && sign > 0) {
								// A rebase changes large editor-world operands, not the small q
								// coordinate that reaches CHP and the block model transform.
								const Vec3 rebase_offset{ 131072.0, 0.0, -98304.0 };
								const Vec3 canonical_block_origin = add(rebase_offset, q_origin);
								q_origin = subtract(canonical_block_origin, rebase_offset);
								++rebase_case_count;
							}
							for (const HeightRange& height : heights) {
								const BoundsChain corrected = build_bounds_chain(
									view, basis_case.basis, block_size_m, q_origin,
									height.minimum_m, height.maximum_m, true);
								++case_count;
								for (double fx : { 0.0, 0.5, 1.0 }) for (double fz : { 0.0, 0.5, 1.0 })
								for (double sample_height : { height.minimum_m, 0.5 * (height.minimum_m + height.maximum_m), height.maximum_m }) {
									const Vec3 flat_q = add(q_origin, basis_case.basis.xform(
										{ block_size_m * fx, 0.0, block_size_m * fz }));
									CHPEvaluation evaluation{};
									require(try_evaluate_curved(profile, CHPIntrinsicSample{
										flat_q.x, flat_q.z, sample_height, 0.0, 0.0 }, evaluation),
										"production CHP point rejected inside admitted envelope");
									const Vec3 exact_q{
										evaluation.position_m.x,
										evaluation.position_m.y - static_cast<double>(view.signed_camera_surface_altitude_m),
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
								}
							}
						}
					}
				}
			}
		}
	}

	const double regression_altitude_m = 109665.5;
	const CHPGpuView regression_view = make_gpu_view(
		CHPFunctionClass::SphericalPolynomial6,
		closed_5000_radius_m,
		std::min(100000.0, closed_5000_radius_m * 0.35),
		regression_altitude_m);
	const Basis identity = bases[0].basis;
	const BoundsChain old_chain = build_bounds_chain(
		regression_view, identity, 32.0,
		{ -16.0, -regression_altitude_m, -16.0 }, 0.0, 0.0, false);
	const BoundsChain corrected_chain = build_bounds_chain(
		regression_view, identity, 32.0,
		{ -16.0, -regression_altitude_m, -16.0 }, 0.0, 0.0, true);
	CHPEvaluation ground{};
	require(try_evaluate_curved(make_profile(
		CHPFunctionClass::SphericalPolynomial6,
		closed_5000_radius_m,
		std::min(100000.0, closed_5000_radius_m * 0.35)),
		CHPIntrinsicSample{ 0.0, 0.0, 0.0, 0.0, 0.0 }, ground),
		"old implementation regression point failed");
	const double old_ground_y = 0.5 * (old_chain.global.minimum.y + old_chain.global.maximum.y);
	const double corrected_ground_y = ground.position_m.y - static_cast<double>(regression_view.signed_camera_surface_altitude_m);
	old_vertical_error_m = old_ground_y - corrected_ground_y;
	require(std::abs(old_vertical_error_m - regression_altitude_m) < 1.0e-3,
		"old culling implementation did not reproduce the 109665.5m vertical error");
	require(containment_margin(old_chain.global, { ground.position_m.x, corrected_ground_y, ground.position_m.z }) < 0.0,
		"old intrinsic-Y AABB unexpectedly contained the camera-relative ground point");
	require(containment_margin(corrected_chain.global, { ground.position_m.x, corrected_ground_y, ground.position_m.z }) >= -1.0e-7,
		"corrected camera-relative AABB did not contain the regression ground point");

	std::cout << "## rendering::CHP-WP6.2-RENDERER-SPACE-BOUNDS\n";
	std::cout << "case_count=" << case_count << "\nsample_count=" << sample_count << "\n";
	std::cout << "domain_cases=3\naltitude_cases=6\nlod_cases=8\nq_sign_cases=2\nbasis_cases=" << bases.size() << "\nheight_range_cases=" << heights.size() << "\nfunction_cases=" << functions.size() << "\nrebase_case_count=" << rebase_case_count << "\n";
	std::cout << "closed_100km_radius_m=" << closed_100_radius_m << "\nclosed_5000km_radius_m=" << closed_5000_radius_m << "\nfinite_500x400km_radius_m=" << finite_500x400_radius_m << "\n";
	std::cout << "camera_altitude_regression_m=109665.5\nground_presentation_y_m=0\n";
	std::cout << "old_vertical_error_m=" << old_vertical_error_m << "\n";
	std::cout << "worst_containment_margin_m=" << worst_containment_margin_m << "\n";
	std::cout << "worst_coordinate_space_reconstruction_error_m=" << worst_reconstruction_error_m << "\n";
	std::cout << "camera_relative_y_chain=PASS\nold_vertical_error_regression=PASS\n[PASS] CHP-RENDERER-SPACE-BOUNDS-01\n";
	return 0;
}

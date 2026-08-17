#include "multinet/rendering/chp/chp_kernel.h"
#include "multinet/rendering/chp/chp_certification.h"
#include "multinet/rendering/chp/chp_view.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_profile.h"
#include "multinet/core/spatial/world_domain.h"
#include "multinet/core/spatial/world_manifests.h"
#include "multinet/world/terrain/canonical_terrain_signal.h"
#include "multinet/world/terrain/terrain_recipe.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <cassert>
#include <vector>

using namespace multinet::rendering::chp;
using namespace multinet::rendering;

namespace {

void require(bool condition, const char* message) {
	if (!condition) {
		std::cerr << "FAIL: " << message << std::endl;
		std::exit(1);
	}
}

CurvedHorizonProfile make_profile(CHPFunctionClass function_class, double requested_distance_m) {
	CurvedHorizonProfile profile;
	profile.function_class = function_class;
	profile.requested_maximum_deformation_distance_m = requested_distance_m;
	profile.maximum_base_position_error_m = 1.0;
	profile.maximum_visual_up_error_radians = 1.0e-4;
	return profile;
}

// Simulates the exact shader position math in double precision
Multinet::Vec3d eval_shader_chp_position_sim(
	CHPFunctionClass function_class,
	double radius_m,
	double inverse_radius,
	double inverse_radius_squared,
	double q_x,
	double q_z,
	double height_m
) {
	double d2 = q_x * q_x + q_z * q_z;
	if (function_class == CHPFunctionClass::QuadraticVerticalFallback) {
		double drop = d2 * 0.5 * inverse_radius;
		return { q_x, height_m - drop, q_z };
	}
	double u = d2 * inverse_radius_squared;
	double a = 1.0;
	double b = 0.0;
	double c = 1.0;
	double u2 = u * u;
	if (function_class == CHPFunctionClass::SphericalPolynomial4) {
		a = 1.0 - u / 6.0 + u2 / 120.0;
		b = u / 2.0 - u2 / 24.0;
		c = 1.0 - u / 2.0 + u2 / 24.0;
	} else {
		double u3 = u2 * u;
		a = 1.0 - u / 6.0 + u2 / 120.0 - u3 / 5040.0;
		b = u / 2.0 - u2 / 24.0 + u3 / 720.0;
		c = 1.0 - u / 2.0 + u2 / 24.0 - u3 / 720.0;
	}
	Multinet::Vec3d base_pos{ a * q_x, -radius_m * b, a * q_z };
	Multinet::Vec3d raw_axis{ a * q_x * inverse_radius, c, a * q_z * inverse_radius };
	double raw_len = std::sqrt(raw_axis.x * raw_axis.x + raw_axis.y * raw_axis.y + raw_axis.z * raw_axis.z);
	Multinet::Vec3d axis = raw_len > 1e-15
		? Multinet::Vec3d{ raw_axis.x / raw_len, raw_axis.y / raw_len, raw_axis.z / raw_len }
		: Multinet::Vec3d{ 0.0, 1.0, 0.0 };
	return { base_pos.x + height_m * axis.x, base_pos.y + height_m * axis.y, base_pos.z + height_m * axis.z };
}

// 3x3 Matrix helpers
struct Mat3x3 {
	double m[3][3]{}; // row, col

	Multinet::Vec3d col(int c) const {
		return { m[0][c], m[1][c], m[2][c] };
	}

	Multinet::Vec3d mult(const Multinet::Vec3d& v) const {
		return {
			m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
			m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
			m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
		};
	}

	Mat3x3 transpose() const {
		Mat3x3 r;
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				r.m[i][j] = m[j][i];
			}
		}
		return r;
	}
};

// Generates all 48 signed permutation orthogonal bases
std::vector<Mat3x3> generate_48_signed_permutation_bases() {
	std::vector<Mat3x3> bases;
	int perms[6][3] = {
		{ 0, 1, 2 }, { 0, 2, 1 },
		{ 1, 0, 2 }, { 1, 2, 0 },
		{ 2, 0, 1 }, { 2, 1, 0 }
	};
	double signs[8][3] = {
		{  1.0,  1.0,  1.0 },
		{  1.0,  1.0, -1.0 },
		{  1.0, -1.0,  1.0 },
		{  1.0, -1.0, -1.0 },
		{ -1.0,  1.0,  1.0 },
		{ -1.0,  1.0, -1.0 },
		{ -1.0, -1.0,  1.0 },
		{ -1.0, -1.0, -1.0 }
	};
	for (const auto& p : perms) {
		for (const auto& s : signs) {
			Mat3x3 m{};
			m.m[p[0]][0] = s[0];
			m.m[p[1]][1] = s[1];
			m.m[p[2]][2] = s[2];
			bases.push_back(m);
		}
	}
	return bases;
}

double dot_product(const Multinet::Vec3d& a, const Multinet::Vec3d& b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

} // namespace

int main() {
	std::cout << "## rendering::CHP-R1-POSITION-RECONSTRUCTION-FIXTURE" << std::endl;

	BlockClipmapProfile profile;
	double earth_radius_m = 6371000.0;
	double inv_r = 1.0 / earth_radius_m;
	double inv_r2 = inv_r * inv_r;

	ResolvedCurvedHorizonProfile resolved{};
	resolved.requested.function_class = CHPFunctionClass::SphericalPolynomial6;
	resolved.requested.profile_version = CHP_PROFILE_VERSION_1;
	resolved.requested.requested_maximum_deformation_distance_m = 500000.0;
	resolved.radius_m = earth_radius_m;
	resolved.inverse_radius = inv_r;
	resolved.inverse_radius_squared = inv_r2;
	resolved.certified_maximum_deformation_distance_m = 500000.0;
	resolved.certified_maximum_theta = 500000.0 * inv_r;
	resolved.certified_maximum_u = 500000.0 * 500000.0 * inv_r2;

	// Test all 8 LODs (spacing 2m through 256m)
	std::vector<uint8_t> all_lods = { 0, 1, 2, 3, 4, 5, 6, 7 };
	auto all_48_bases = generate_48_signed_permutation_bases();
	require(all_48_bases.size() == 48, "Expected 48 signed permutation bases");

	std::vector<double> test_altitudes = { 0.0, 100.0, 3000.0, 50000.0 };
	std::vector<double> test_heights = { -100.0, 0.0, 1234.5, 8000.0 };
	std::vector<Multinet::Vec3d> test_origins = {
		{ 0.0, -3000.0, 0.0 },
		{ 12345.0, -100.0, -6789.0 },
		{ -50000.0, 0.0, 50000.0 }
	};
	std::vector<Multinet::Vec3d> test_local_coords = {
		{ -8.0, 0.0, -8.0 },
		{  0.0, 0.0,  0.0 },
		{  8.0, 0.0,  8.0 },
		{ 16.0, 0.0, 16.0 }
	};

	double max_correct_reconstruction_error_m = 0.0;
	double max_old_transpose_path_error_m = 0.0;
	double max_parity_error_m = 0.0;
	uint64_t cases_tested = 0;

	struct LODErrorStats {
		double max_correct_err = 0.0;
		double max_old_err = 0.0;
		double min_old_err_on_displaced = 1e30;
	};
	std::vector<LODErrorStats> lod_stats(8);

	for (uint8_t lod : all_lods) {
		double spacing = profile.get_lod_spacing(lod);

		for (const auto& rot : all_48_bases) {
			// Model basis = rot * diag(spacing, 1.0, spacing)
			Mat3x3 model_basis{};
			model_basis.m[0][0] = rot.m[0][0] * spacing; model_basis.m[0][1] = rot.m[0][1]; model_basis.m[0][2] = rot.m[0][2] * spacing;
			model_basis.m[1][0] = rot.m[1][0] * spacing; model_basis.m[1][1] = rot.m[1][1]; model_basis.m[1][2] = rot.m[1][2] * spacing;
			model_basis.m[2][0] = rot.m[2][0] * spacing; model_basis.m[2][1] = rot.m[2][1]; model_basis.m[2][2] = rot.m[2][2] * spacing;

			Multinet::Vec3d bx = model_basis.col(0);
			Multinet::Vec3d by = model_basis.col(1);
			Multinet::Vec3d bz = model_basis.col(2);

			double bx2 = dot_product(bx, bx);
			double by2 = dot_product(by, by);
			double bz2 = dot_product(bz, bz);

			require(bx2 > 1e-12 && by2 > 1e-12 && bz2 > 1e-12, "Degenerate basis columns");

			// Old Forward+ transpose basis: MODEL_NORMAL_MATRIX ≈ model_basis
			// inverse_model_basis = transpose(MODEL_NORMAL_MATRIX)
			Mat3x3 old_inv_model_basis = model_basis.transpose();

			for (const auto& origin : test_origins) {
				for (double alt : test_altitudes) {
					for (double h : test_heights) {
						for (const auto& local_coord : test_local_coords) {
							// 1. Forward flat model position
							Multinet::Vec3d flat_model{ local_coord.x, h, local_coord.z };
							Multinet::Vec3d flat_cam_rel = {
								model_basis.mult(flat_model).x + origin.x,
								model_basis.mult(flat_model).y + origin.y,
								model_basis.mult(flat_model).z + origin.z
							};

							double q_x = flat_cam_rel.x;
							double q_z = flat_cam_rel.z;

							// 2. Evaluate CPU authority
							CHPIntrinsicSample sample{ q_x, q_z, h, 0.0, 0.0 };
							CHPEvaluation cpu_eval{};
							bool cpu_ok = try_evaluate_curved(resolved, sample, cpu_eval);
							require(cpu_ok, "try_evaluate_curved failed within certified range");

							// 3. Evaluate shader simulation
							Multinet::Vec3d p_surface_sim = eval_shader_chp_position_sim(
								CHPFunctionClass::SphericalPolynomial6,
								earth_radius_m, inv_r, inv_r2, q_x, q_z, h
							);

							double parity_err = std::sqrt(
								(cpu_eval.position_m.x - p_surface_sim.x) * (cpu_eval.position_m.x - p_surface_sim.x) +
								(cpu_eval.position_m.y - p_surface_sim.y) * (cpu_eval.position_m.y - p_surface_sim.y) +
								(cpu_eval.position_m.z - p_surface_sim.z) * (cpu_eval.position_m.z - p_surface_sim.z)
							);
							max_parity_error_m = std::max(max_parity_error_m, parity_err);
							require(parity_err < 1e-9, "Shader CHP position simulation diverged from CPU authority");

							// 4. Target curved camera-relative position
							Multinet::Vec3d target_cam_rel = {
								p_surface_sim.x,
								p_surface_sim.y - alt,
								p_surface_sim.z
							};

							Multinet::Vec3d delta = {
								target_cam_rel.x - origin.x,
								target_cam_rel.y - origin.y,
								target_cam_rel.z - origin.z
							};

							// 5A. Correct Explicit Orthogonal-Column Inverse Reconstruction:
							Multinet::Vec3d model_local_correct{
								dot_product(delta, bx) / bx2,
								dot_product(delta, by) / by2,
								dot_product(delta, bz) / bz2
							};

							Multinet::Vec3d final_cam_rel_correct = {
								model_basis.mult(model_local_correct).x + origin.x,
								model_basis.mult(model_local_correct).y + origin.y,
								model_basis.mult(model_local_correct).z + origin.z
							};

							double correct_err = std::sqrt(
								(final_cam_rel_correct.x - target_cam_rel.x) * (final_cam_rel_correct.x - target_cam_rel.x) +
								(final_cam_rel_correct.y - target_cam_rel.y) * (final_cam_rel_correct.y - target_cam_rel.y) +
								(final_cam_rel_correct.z - target_cam_rel.z) * (final_cam_rel_correct.z - target_cam_rel.z)
							);
							max_correct_reconstruction_error_m = std::max(max_correct_reconstruction_error_m, correct_err);
							lod_stats[lod].max_correct_err = std::max(lod_stats[lod].max_correct_err, correct_err);
							require(correct_err < 1e-9, "Explicit orthogonal-column inverse reconstruction failed round-trip");

							// 5B. Old Transpose Failure Path Reconstruction:
							Multinet::Vec3d model_local_old = old_inv_model_basis.mult(delta);
							Multinet::Vec3d final_cam_rel_old = {
								model_basis.mult(model_local_old).x + origin.x,
								model_basis.mult(model_local_old).y + origin.y,
								model_basis.mult(model_local_old).z + origin.z
							};

							double old_err = std::sqrt(
								(final_cam_rel_old.x - target_cam_rel.x) * (final_cam_rel_old.x - target_cam_rel.x) +
								(final_cam_rel_old.y - target_cam_rel.y) * (final_cam_rel_old.y - target_cam_rel.y) +
								(final_cam_rel_old.z - target_cam_rel.z) * (final_cam_rel_old.z - target_cam_rel.z)
							);
							max_old_transpose_path_error_m = std::max(max_old_transpose_path_error_m, old_err);
							lod_stats[lod].max_old_err = std::max(lod_stats[lod].max_old_err, old_err);

							// 6. Identity reconstruction test: when target == flat_cam_rel
							Multinet::Vec3d flat_delta = {
								flat_cam_rel.x - origin.x,
								flat_cam_rel.y - origin.y,
								flat_cam_rel.z - origin.z
							};
							Multinet::Vec3d model_local_identity{
								dot_product(flat_delta, bx) / bx2,
								dot_product(flat_delta, by) / by2,
								dot_product(flat_delta, bz) / bz2
							};
							double id_err = std::sqrt(
								(model_local_identity.x - flat_model.x) * (model_local_identity.x - flat_model.x) +
								(model_local_identity.y - flat_model.y) * (model_local_identity.y - flat_model.y) +
								(model_local_identity.z - flat_model.z) * (model_local_identity.z - flat_model.z)
							);
							require(id_err < 1e-9, "Identity reconstruction failed to match flat model vertex");

							// 7. Verify q=0 altitude invariance: at q=0, visible Y = h - alt
							if (std::abs(q_x) < 1e-12 && std::abs(q_z) < 1e-12) {
								require(std::abs(target_cam_rel.y - (h - alt)) < 1e-9, "Altitude invariance violated at q=0");
							}

							++cases_tested;
						}
					}
				}
			}
		}
	}

	std::cout << "[REGRESSION TABLE] Old Transpose Failure vs Correct Explicit Column Inverse:" << std::endl;
	for (uint8_t lod = 0; lod < 8; ++lod) {
		double spacing = profile.get_lod_spacing(lod);
		std::cout << "  LOD" << int(lod)
				  << " (spacing=" << std::setw(3) << spacing << "m, s^2=" << std::setw(6) << spacing * spacing << "):"
				  << " max_old_error=" << std::setw(12) << lod_stats[lod].max_old_err << " m"
				  << " | max_correct_error=" << lod_stats[lod].max_correct_err << " m"
				  << std::endl;
	}

	std::cout << "[PASS] cases_tested=" << cases_tested << std::endl;
	std::cout << "[PASS] max_parity_error_m=" << max_parity_error_m << std::endl;
	std::cout << "[PASS] max_correct_reconstruction_error_m=" << max_correct_reconstruction_error_m << std::endl;
	std::cout << "[PASS] max_old_transpose_path_error_m=" << max_old_transpose_path_error_m << std::endl;
	require(max_correct_reconstruction_error_m < 1e-9, "Correct reconstruction failed");
	require(max_old_transpose_path_error_m > 1e6, "Old path failed to reproduce massive scale-amplification error");

	// =========================================================================
	// R1.1 TESTS: Signed Altitude, Transitions, Negative Terrain & q=0 Geometry
	// =========================================================================

	// 1. Signed Altitude & Nonnegative Horizon Clamp Tests
	{
		std::cout << "\n[R1.1 TEST 1] Signed CHP Altitude & Nonnegative Horizon Clamp Tests..." << std::endl;
		Multinet::WorldDomainInput domain_input{};
		domain_input.topology = Multinet::WorldDomainTopology::ClosedSurfaceSixFace;
		domain_input.closed_surface.area_equivalent_side_m = 5000000;
		Multinet::WorldDomainManifest domain = Multinet::build_world_domain_manifest(domain_input);
		require(domain.is_valid(), "Domain manifest invalid");

		Multinet::WorldPresentationInput pres_input{};
		pres_input.chp_enabled = true;
		pres_input.chp_radius_policy = Multinet::CHPRadiusPolicy::CanonicalClosedSurface;
		Multinet::WorldPresentationManifest pres = Multinet::build_world_presentation_manifest(domain, pres_input);
		require(pres.is_valid(), "Presentation manifest invalid");

		CurvedHorizonProfile req = make_profile(CHPFunctionClass::SphericalPolynomial6, 1410474.0);
		ResolvedCurvedHorizonProfile rprofile{};
		require(try_resolve_curved_horizon_profile(pres, req, rprofile), "Profile resolution failed");

		const std::vector<double> test_altitudes = { 3000.0, 1.0, 0.0, -1.0, -200.0, -5000.0 };
		for (double alt : test_altitudes) {
			Multinet::SurfacePosition64 cam_surface{};
			cam_surface.face = Multinet::SurfaceFace::PositiveX;
			cam_surface.u_m = 0.0;
			cam_surface.v_m = 0.0;
			cam_surface.altitude_m = alt;
			cam_surface.topology_version = domain.topology_version;
			cam_surface.projection_version = domain.projection_version;

			Multinet::FramePosition64 cam_frame{ 0.0, alt, 0.0 };
			CurvedHorizonView view{};
			bool ok = try_build_curved_horizon_view(
				domain, pres, rprofile, cam_surface, cam_frame, 1, 1, 1, view
			);
			require(ok && view.is_valid(), "Failed to build valid CurvedHorizonView for altitude");
			require(view.signed_camera_surface_altitude_m == alt, "signed_camera_surface_altitude_m mismatch");
			require(view.camera_surface_height_m == alt, "camera_surface_height_m mismatch");
			require(view.horizon_observer_height_m == std::max(0.0, alt), "horizon_observer_height_m mismatch");

			if (alt <= 0.0) {
				require(view.horizon_line_of_sight_m == 0.0, "LOS horizon must be 0 for altitude <= 0");
				require(view.horizon_surface_arc_m == 0.0, "Arc horizon must be 0 for altitude <= 0");
			} else {
				require(view.horizon_line_of_sight_m > 0.0, "LOS horizon must be > 0 for altitude > 0");
				require(view.horizon_surface_arc_m > 0.0, "Arc horizon must be > 0 for altitude > 0");
			}
			std::cout << "  alt=" << std::setw(7) << alt << " m -> signed=" << std::setw(7) << view.signed_camera_surface_altitude_m
					  << " m | horizon_h=" << std::setw(7) << view.horizon_observer_height_m << " m [PASS]" << std::endl;
		}
		std::cout << "[PASS] R1.1-SIGNED-ALTITUDE-01" << std::endl;
	}

	// 2. Sequential Observer Altitude Transitions
	{
		std::cout << "\n[R1.1 TEST 2] Sequential Runtime Observer Altitude Transitions..." << std::endl;
		Multinet::WorldDomainInput domain_input{};
		domain_input.topology = Multinet::WorldDomainTopology::ClosedSurfaceSixFace;
		domain_input.closed_surface.area_equivalent_side_m = 5000000;
		Multinet::WorldDomainManifest domain = Multinet::build_world_domain_manifest(domain_input);

		Multinet::WorldPresentationInput pres_input{};
		pres_input.chp_enabled = true;
		pres_input.chp_radius_policy = Multinet::CHPRadiusPolicy::CanonicalClosedSurface;
		Multinet::WorldPresentationManifest pres = Multinet::build_world_presentation_manifest(domain, pres_input);

		CurvedHorizonProfile req = make_profile(CHPFunctionClass::SphericalPolynomial6, 1410474.0);
		ResolvedCurvedHorizonProfile rprofile{};
		require(try_resolve_curved_horizon_profile(pres, req, rprofile), "Profile resolution failed");

		const std::vector<double> sequence = { 3000.0, 100.0, 0.0, -100.0, -1000.0, 500.0 };
		for (size_t i = 0; i < sequence.size(); ++i) {
			double canonical_alt = sequence[i];
			Multinet::SurfacePosition64 cam_surface{};
			cam_surface.face = Multinet::SurfaceFace::PositiveX;
			cam_surface.u_m = 0.0;
			cam_surface.v_m = 0.0;
			cam_surface.altitude_m = canonical_alt;
			cam_surface.topology_version = domain.topology_version;
			cam_surface.projection_version = domain.projection_version;

			Multinet::FramePosition64 cam_frame{ 0.0, canonical_alt, 0.0 };
			CurvedHorizonView view{};
			bool ok = try_build_curved_horizon_view(
				domain, pres, rprofile, cam_surface, cam_frame, i + 1, static_cast<uint32_t>(i + 1), 1, view
			);
			require(ok && view.is_valid(), "View build failed in sequential transition");
			require(view.signed_camera_surface_altitude_m == canonical_alt, "Sequential transition signed altitude mismatch");
			require(view.camera_surface_height_m == canonical_alt, "Sequential transition camera_surface_height_m mismatch");
			std::cout << "  Step " << i << ": canonical_alt=" << std::setw(7) << canonical_alt << " m == CHP signed_alt="
					  << std::setw(7) << view.signed_camera_surface_altitude_m << " m [PASS]" << std::endl;
		}
		std::cout << "[PASS] R1.1-SEQUENTIAL-TRANSITIONS-01" << std::endl;
	}

	// 3. Deterministic Negative Terrain Evaluation & Parity
	{
		std::cout << "\n[R1.1 TEST 3] Deterministic Negative Terrain Evaluation & Parity..." << std::endl;
		Multinet::WorldDomainInput domain_input{};
		domain_input.topology = Multinet::WorldDomainTopology::ClosedSurfaceSixFace;
		domain_input.closed_surface.area_equivalent_side_m = 5000000;
		Multinet::WorldDomainManifest domain = Multinet::build_world_domain_manifest(domain_input);

		Multinet::TerrainRecipe recipe{};
		recipe.identity.world_seed = 1337;
		recipe.legacy_signals.continental_frequency = 0.0001f;
		recipe.legacy_signals.min_elevation_m = -200.0f;
		recipe.legacy_signals.max_elevation_m = 8000.0f;
		recipe.legacy_signals.octave_count = 4;
		recipe.legacy_signals.persistence = 0.5f;
		recipe.legacy_signals.lacunarity = 2.0f;
		require(Multinet::finalize_terrain_recipe(recipe, domain.closed_surface), "Recipe finalization failed");

		Multinet::CanonicalTerrainSignalV1 signal(recipe, domain.closed_surface);
		Multinet::SurfacePosition64 sample_pos{
			Multinet::SurfaceFace::PositiveX,
			0.0,
			6080.0,
			0.0,
			domain.topology_version,
			domain.projection_version
		};

		double canonical_h = signal.evaluate_height(sample_pos);
		std::cout << "  Deterministic query at Face=+X, u=0.0m, v=6080.0m:" << std::endl;
		std::cout << "  Canonical Terrain Height = " << std::fixed << std::setprecision(6) << canonical_h << " m" << std::endl;
		require(canonical_h < 0.0, "Deterministic terrain height must be strictly negative");
		require(std::abs(canonical_h - (-113.083)) < 0.1, "Deterministic terrain height does not match expected ~ -113.083 m");
		std::cout << "[PASS] R1.1-NEGATIVE-TERRAIN-PARITY-01" << std::endl;
	}

	// 4. Geometry Law at q=(0,0) (P_surface.y == final_y)
	{
		std::cout << "\n[R1.1 TEST 4] Geometry Law at q=(0,0) (P_surface.y == final_y)..." << std::endl;
		const double canonical_h = -113.083;
		const double radius_m = 1410474.0;
		const double inverse_radius = 1.0 / radius_m;
		const double inverse_radius_sq = inverse_radius * inverse_radius;

		Multinet::Vec3d p_surf = eval_shader_chp_position_sim(
			CHPFunctionClass::SphericalPolynomial6,
			radius_m, inverse_radius, inverse_radius_sq,
			0.0, 0.0, canonical_h
		);
		require(std::abs(p_surf.x) < 1e-12 && std::abs(p_surf.z) < 1e-12, "p_surf x/z must be zero at q=(0,0)");
		require(std::abs(p_surf.y - canonical_h) < 1e-12, "P_surface.y must exactly equal final_y at q=(0,0)");

		const std::vector<std::pair<double, double>> cases = {
			{ 0.0, -113.083 },
			{ 100.0, -213.083 },
			{ -100.0, -13.083 }
		};
		for (const auto& [alt, expected_cam_rel_y] : cases) {
			double cam_rel_y = p_surf.y - alt;
			require(std::abs(cam_rel_y - expected_cam_rel_y) < 1e-6, "Camera relative Y mismatch at q=(0,0)");
			std::cout << "  Camera Alt=" << std::setw(6) << alt << " m -> rendered camera-relative Y="
					  << std::setw(9) << cam_rel_y << " m (expected=" << std::setw(9) << expected_cam_rel_y << " m) [PASS]" << std::endl;
		}
		std::cout << "[PASS] R1.1-Q0-GEOMETRY-LAW-01" << std::endl;
	}

	std::cout << "\nSTATUS: ALL R1 & R1.1 FIXTURES PASSED WITH EVIDENCE" << std::endl;
	return 0;
}

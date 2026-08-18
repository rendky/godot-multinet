#include "block_clipmap_morph.h"

namespace multinet::rendering {

MorphBandProfile get_morph_band_profile(
	uint8_t lod,
	const BlockClipmapProfile& profile,
	MorphStrategy strategy
) noexcept {
	const double block_size = profile.get_lod_block_size(lod);
	MorphBandProfile band{};

	switch (strategy) {
		case MorphStrategy::StrategyA_Legal_1B_1B_2B:
			band.fine_zone_radius_m = 1.0 * block_size;
			band.transition_band_width_m = 1.0 * block_size;
			band.parent_lock_radius_m = 2.0 * block_size;
			break;

		case MorphStrategy::StrategyA_Legal_1_25B_0_75B_2B:
			band.fine_zone_radius_m = 1.25 * block_size;
			band.transition_band_width_m = 0.75 * block_size;
			band.parent_lock_radius_m = 2.0 * block_size;
			break;

		case MorphStrategy::StrategyA_Legal_1_5B_0_5B_2B:
			band.fine_zone_radius_m = 1.5 * block_size;
			band.transition_band_width_m = 0.5 * block_size;
			band.parent_lock_radius_m = 2.0 * block_size;
			break;

		case MorphStrategy::StrategyA_SubmittedDefective:
			band.fine_zone_radius_m = 1.5 * block_size;
			band.transition_band_width_m = 1.5 * block_size;
			band.parent_lock_radius_m = 3.0 * block_size;
			break;

		case MorphStrategy::StrategyB_Defective_4_5B:
			band.fine_zone_radius_m = 2.5 * block_size;
			band.transition_band_width_m = 2.0 * block_size;
			band.parent_lock_radius_m = 4.5 * block_size;
			break;

		case MorphStrategy::StrategyB_Legal_3B:
			band.fine_zone_radius_m = 1.5 * block_size;
			band.transition_band_width_m = 1.5 * block_size;
			band.parent_lock_radius_m = 3.0 * block_size;
			break;

		case MorphStrategy::ProfileDerived: {
			const double r = static_cast<double>(profile.candidate_grid_radius);
			const double fine_r = std::max(1.0, r - 3.0);
			band.fine_zone_radius_m = fine_r * block_size;
			band.transition_band_width_m = 1.0 * block_size;
			band.parent_lock_radius_m = (fine_r + 1.0) * block_size;
			break;
		}
	}

	return band;
}

VertexMorphResult evaluate_vertex_morph(
	int64_t block_bx,
	int64_t block_bv,
	int32_t quad_ix,
	int32_t quad_iz,
	uint8_t lod,
	double observer_u_m,
	double observer_v_m,
	const BlockClipmapProfile& profile,
	MorphStrategy strategy,
	MorphMetricLaw metric_law
) noexcept {
	const double fine_spacing = profile.get_lod_spacing(lod);
	const double fine_block_size = profile.get_lod_block_size(lod);

	const double fine_u = static_cast<double>(block_bx) * fine_block_size + static_cast<double>(quad_ix) * fine_spacing;
	const double fine_v = static_cast<double>(block_bv) * fine_block_size + static_cast<double>(quad_iz) * fine_spacing;

	const bool has_parent = has_parent_lod(lod, profile);

	VertexMorphResult res{};
	res.fine_u_m = fine_u;
	res.fine_v_m = fine_v;
	res.parent_target_u_m = fine_u;
	res.parent_target_v_m = fine_v;
	res.morphed_u_m = fine_u;
	res.morphed_v_m = fine_v;
	res.is_odd_u = ((quad_ix % 2) != 0);
	res.is_odd_v = ((quad_iz % 2) != 0);
	res.has_parent = has_parent;

	if (!has_parent) {
		return res;
	}

	const double parent_target_u = compute_parent_lattice_target_local(block_bx, quad_ix, fine_block_size, fine_spacing);
	const double parent_target_v = compute_parent_lattice_target_local(block_bv, quad_iz, fine_block_size, fine_spacing);
	res.parent_target_u_m = parent_target_u;
	res.parent_target_v_m = parent_target_v;

	const MorphBandProfile band = get_morph_band_profile(lod, profile, strategy);

	const double du = std::abs(fine_u - observer_u_m);
	const double dv = std::abs(fine_v - observer_v_m);

	const double mu_u = compute_1d_morph_factor(fine_u, observer_u_m, band);
	const double mu_v = compute_1d_morph_factor(fine_v, observer_v_m, band);
	res.morph_factor_u = mu_u;
	res.morph_factor_v = mu_v;

	double morphed_u = fine_u;
	double morphed_v = fine_v;
	double combined_mu = 0.0;

	switch (metric_law) {
		case MorphMetricLaw::ChebyshevScalar: {
			const double d_inf = std::max(du, dv);
			if (d_inf <= band.fine_zone_radius_m) {
				combined_mu = 0.0;
			} else if (d_inf >= band.parent_lock_radius_m || band.transition_band_width_m <= 1e-9) {
				combined_mu = 1.0;
			} else {
				combined_mu = (d_inf - band.fine_zone_radius_m) / band.transition_band_width_m;
			}
			morphed_u = fine_u + (parent_target_u - fine_u) * combined_mu;
			morphed_v = fine_v + (parent_target_v - fine_v) * combined_mu;
			break;
		}
		case MorphMetricLaw::IndependentAxes: {
			morphed_u = fine_u + (parent_target_u - fine_u) * mu_u;
			morphed_v = fine_v + (parent_target_v - fine_v) * mu_v;
			combined_mu = std::max(mu_u, mu_v);
			break;
		}
		case MorphMetricLaw::AlgebraicSum: {
			combined_mu = 1.0 - (1.0 - mu_u) * (1.0 - mu_v);
			morphed_u = fine_u + (parent_target_u - fine_u) * combined_mu;
			morphed_v = fine_v + (parent_target_v - fine_v) * combined_mu;
			break;
		}
	}

	const double disp_u = morphed_u - fine_u;
	const double disp_v = morphed_v - fine_v;

	res.combined_morph_factor = combined_mu;
	res.morphed_u_m = morphed_u;
	res.morphed_v_m = morphed_v;
	res.horizontal_displacement_m = std::sqrt(disp_u * disp_u + disp_v * disp_v);
	res.is_fully_parent_locked = (combined_mu >= 1.0 - 1e-9);
	res.active_recursion_depth = (combined_mu > 0.0) ? 1 : 0;

	return res;
}

VertexMorphResult evaluate_vertex_morph_world(
	double world_u_m,
	double world_v_m,
	uint8_t lod,
	double observer_u_m,
	double observer_v_m,
	const BlockClipmapProfile& profile,
	MorphStrategy strategy,
	MorphMetricLaw metric_law
) noexcept {
	const double fine_spacing = profile.get_lod_spacing(lod);
	const double parent_spacing = fine_spacing * 2.0;

	const bool has_parent = has_parent_lod(lod, profile);

	VertexMorphResult res{};
	res.fine_u_m = world_u_m;
	res.fine_v_m = world_v_m;
	res.parent_target_u_m = world_u_m;
	res.parent_target_v_m = world_v_m;
	res.morphed_u_m = world_u_m;
	res.morphed_v_m = world_v_m;
	res.is_odd_u = (std::abs(std::round(world_u_m / fine_spacing) - (std::floor(world_u_m / parent_spacing) * 2.0)) > 0.5);
	res.is_odd_v = (std::abs(std::round(world_v_m / fine_spacing) - (std::floor(world_v_m / parent_spacing) * 2.0)) > 0.5);
	res.has_parent = has_parent;

	if (!has_parent) {
		return res;
	}

	const double parent_target_u = compute_parent_lattice_target(world_u_m, parent_spacing);
	const double parent_target_v = compute_parent_lattice_target(world_v_m, parent_spacing);
	res.parent_target_u_m = parent_target_u;
	res.parent_target_v_m = parent_target_v;

	const MorphBandProfile band = get_morph_band_profile(lod, profile, strategy);

	const double du = std::abs(world_u_m - observer_u_m);
	const double dv = std::abs(world_v_m - observer_v_m);

	const double mu_u = compute_1d_morph_factor(world_u_m, observer_u_m, band);
	const double mu_v = compute_1d_morph_factor(world_v_m, observer_v_m, band);
	res.morph_factor_u = mu_u;
	res.morph_factor_v = mu_v;

	double morphed_u = world_u_m;
	double morphed_v = world_v_m;
	double combined_mu = 0.0;

	switch (metric_law) {
		case MorphMetricLaw::ChebyshevScalar: {
			const double d_inf = std::max(du, dv);
			if (d_inf <= band.fine_zone_radius_m) {
				combined_mu = 0.0;
			} else if (d_inf >= band.parent_lock_radius_m || band.transition_band_width_m <= 1e-9) {
				combined_mu = 1.0;
			} else {
				combined_mu = (d_inf - band.fine_zone_radius_m) / band.transition_band_width_m;
			}
			morphed_u = world_u_m + (parent_target_u - world_u_m) * combined_mu;
			morphed_v = world_v_m + (parent_target_v - world_v_m) * combined_mu;
			break;
		}
		case MorphMetricLaw::IndependentAxes: {
			morphed_u = world_u_m + (parent_target_u - world_u_m) * mu_u;
			morphed_v = world_v_m + (parent_target_v - world_v_m) * mu_v;
			combined_mu = std::max(mu_u, mu_v);
			break;
		}
		case MorphMetricLaw::AlgebraicSum: {
			combined_mu = 1.0 - (1.0 - mu_u) * (1.0 - mu_v);
			morphed_u = world_u_m + (parent_target_u - world_u_m) * combined_mu;
			morphed_v = world_v_m + (parent_target_v - world_v_m) * combined_mu;
			break;
		}
	}

	const double disp_u = morphed_u - world_u_m;
	const double disp_v = morphed_v - world_v_m;

	res.combined_morph_factor = combined_mu;
	res.morphed_u_m = morphed_u;
	res.morphed_v_m = morphed_v;
	res.horizontal_displacement_m = std::sqrt(disp_u * disp_u + disp_v * disp_v);
	res.is_fully_parent_locked = (combined_mu >= 1.0 - 1e-9);
	res.active_recursion_depth = (combined_mu > 0.0) ? 1 : 0;

	return res;
}

VertexMorphResult evaluate_vertex_morph_recursive_world(
	double world_u_m,
	double world_v_m,
	uint8_t lod,
	double observer_u_m,
	double observer_v_m,
	const BlockClipmapProfile& profile,
	MorphStrategy strategy,
	MorphMetricLaw metric_law
) noexcept {
	const double fine_spacing = profile.get_lod_spacing(lod);
	const double parent_spacing = fine_spacing * 2.0;

	const bool has_parent = has_parent_lod(lod, profile);

	VertexMorphResult res{};
	res.fine_u_m = world_u_m;
	res.fine_v_m = world_v_m;
	res.parent_target_u_m = world_u_m;
	res.parent_target_v_m = world_v_m;
	res.morphed_u_m = world_u_m;
	res.morphed_v_m = world_v_m;
	res.is_odd_u = (std::abs(std::round(world_u_m / fine_spacing) - (std::floor(world_u_m / parent_spacing) * 2.0)) > 0.5);
	res.is_odd_v = (std::abs(std::round(world_v_m / fine_spacing) - (std::floor(world_v_m / parent_spacing) * 2.0)) > 0.5);
	res.has_parent = has_parent;

	if (!has_parent) {
		return res;
	}

	const double parent_target_u = compute_parent_lattice_target(world_u_m, parent_spacing);
	const double parent_target_v = compute_parent_lattice_target(world_v_m, parent_spacing);
	res.parent_target_u_m = parent_target_u;
	res.parent_target_v_m = parent_target_v;

	const MorphBandProfile band = get_morph_band_profile(lod, profile, strategy);

	const double du = std::abs(world_u_m - observer_u_m);
	const double dv = std::abs(world_v_m - observer_v_m);

	const double mu_u = compute_1d_morph_factor(world_u_m, observer_u_m, band);
	const double mu_v = compute_1d_morph_factor(world_v_m, observer_v_m, band);
	res.morph_factor_u = mu_u;
	res.morph_factor_v = mu_v;

	double combined_mu = 0.0;
	switch (metric_law) {
		case MorphMetricLaw::ChebyshevScalar: {
			const double d_inf = std::max(du, dv);
			if (d_inf <= band.fine_zone_radius_m) {
				combined_mu = 0.0;
			} else if (d_inf >= band.parent_lock_radius_m || band.transition_band_width_m <= 1e-9) {
				combined_mu = 1.0;
			} else {
				combined_mu = (d_inf - band.fine_zone_radius_m) / band.transition_band_width_m;
			}
			break;
		}
		case MorphMetricLaw::IndependentAxes:
			combined_mu = std::max(mu_u, mu_v);
			break;
		case MorphMetricLaw::AlgebraicSum:
			combined_mu = 1.0 - (1.0 - mu_u) * (1.0 - mu_v);
			break;
	}

	res.combined_morph_factor = combined_mu;
	res.is_fully_parent_locked = (combined_mu >= 1.0 - 1e-9);

	if (combined_mu <= 0.0) {
		res.morphed_u_m = world_u_m;
		res.morphed_v_m = world_v_m;
		res.horizontal_displacement_m = 0.0;
		res.active_recursion_depth = 0;
		return res;
	}

	// Recursive live-parent evaluation: F_{L+1}(P_L(x), observer)
	const VertexMorphResult parent_live = evaluate_vertex_morph_recursive_world(
		parent_target_u, parent_target_v,
		lod + 1, observer_u_m, observer_v_m, profile, strategy, metric_law
	);

	res.parent_target_u_m = parent_live.morphed_u_m;
	res.parent_target_v_m = parent_live.morphed_v_m;

	const double morphed_u = world_u_m + (parent_live.morphed_u_m - world_u_m) * combined_mu;
	const double morphed_v = world_v_m + (parent_live.morphed_v_m - world_v_m) * combined_mu;

	const double disp_u = morphed_u - world_u_m;
	const double disp_v = morphed_v - world_v_m;

	res.morphed_u_m = morphed_u;
	res.morphed_v_m = morphed_v;
	res.horizontal_displacement_m = std::sqrt(disp_u * disp_u + disp_v * disp_v);
	res.active_recursion_depth = parent_live.active_recursion_depth + 1;

	return res;
}

VertexMorphResult evaluate_vertex_morph_recursive(
	int64_t block_bx,
	int64_t block_bv,
	int32_t quad_ix,
	int32_t quad_iz,
	uint8_t lod,
	double observer_u_m,
	double observer_v_m,
	const BlockClipmapProfile& profile,
	MorphStrategy strategy,
	MorphMetricLaw metric_law
) noexcept {
	const double fine_spacing = profile.get_lod_spacing(lod);
	const double fine_block_size = profile.get_lod_block_size(lod);

	const double fine_u = static_cast<double>(block_bx) * fine_block_size + static_cast<double>(quad_ix) * fine_spacing;
	const double fine_v = static_cast<double>(block_bv) * fine_block_size + static_cast<double>(quad_iz) * fine_spacing;

	return evaluate_vertex_morph_recursive_world(
		fine_u, fine_v, lod, observer_u_m, observer_v_m, profile, strategy, metric_law
	);
}

VertexMorphResult evaluate_presentation_vertex_morph_recursive(
	int64_t pres_bx,
	int64_t pres_bv,
	int32_t local_ix,
	int32_t local_iz,
	uint8_t lod,
	double active_cam_u_m,
	double active_cam_v_m,
	const BlockClipmapProfile& profile,
	MorphStrategy strategy,
	MorphMetricLaw metric_law
) noexcept {
	const double fine_spacing = profile.get_lod_spacing(lod);
	const double fine_block_size = profile.get_lod_block_size(lod);

	const double fine_u = static_cast<double>(pres_bx) * fine_block_size + static_cast<double>(local_ix) * fine_spacing;
	const double fine_v = static_cast<double>(pres_bv) * fine_block_size + static_cast<double>(local_iz) * fine_spacing;

	return evaluate_vertex_morph_recursive_world(
		fine_u, fine_v, lod, active_cam_u_m, active_cam_v_m, profile, strategy, metric_law
	);
}

VertexMorphResult evaluate_vertex_morph_finite_clamped(
	double world_u_m,
	double world_v_m,
	uint8_t lod,
	double observer_u_m,
	double observer_v_m,
	double finite_min_u_m,
	double finite_max_u_m,
	double finite_min_v_m,
	double finite_max_v_m,
	const BlockClipmapProfile& profile,
	MorphStrategy strategy,
	MorphMetricLaw metric_law,
	bool use_recursive
) noexcept {
	VertexMorphResult res = use_recursive
		? evaluate_vertex_morph_recursive_world(world_u_m, world_v_m, lod, observer_u_m, observer_v_m, profile, strategy, metric_law)
		: evaluate_vertex_morph_world(world_u_m, world_v_m, lod, observer_u_m, observer_v_m, profile, strategy, metric_law);

	const bool on_min_u = (std::abs(world_u_m - finite_min_u_m) < 1e-9);
	const bool on_max_u = (std::abs(world_u_m - finite_max_u_m) < 1e-9);
	const bool on_min_v = (std::abs(world_v_m - finite_min_v_m) < 1e-9);
	const bool on_max_v = (std::abs(world_v_m - finite_max_v_m) < 1e-9);

	if (on_min_u) {
		res.morphed_u_m = finite_min_u_m;
		res.parent_target_u_m = finite_min_u_m;
	} else if (on_max_u) {
		res.morphed_u_m = finite_max_u_m;
		res.parent_target_u_m = finite_max_u_m;
	} else {
		res.morphed_u_m = std::clamp(res.morphed_u_m, finite_min_u_m, finite_max_u_m);
	}

	if (on_min_v) {
		res.morphed_v_m = finite_min_v_m;
		res.parent_target_v_m = finite_min_v_m;
	} else if (on_max_v) {
		res.morphed_v_m = finite_max_v_m;
		res.parent_target_v_m = finite_max_v_m;
	} else {
		res.morphed_v_m = std::clamp(res.morphed_v_m, finite_min_v_m, finite_max_v_m);
	}

	const double disp_u = res.morphed_u_m - world_u_m;
	const double disp_v = res.morphed_v_m - world_v_m;
	res.horizontal_displacement_m = std::sqrt(disp_u * disp_u + disp_v * disp_v);

	return res;
}

LODMorphFootprintMetrics compute_morph_footprint_metrics(
	uint8_t lod,
	const BlockClipmapProfile& profile,
	MorphStrategy strategy
) noexcept {
	const double block_size = profile.get_lod_block_size(lod);
	const double snap_period = (lod + 1 < profile.level_count) ? profile.get_lod_block_size(lod + 1) : block_size;
	const double fine_spacing = profile.get_lod_spacing(lod);
	const double parent_spacing = fine_spacing * 2.0;

	const bool has_parent = has_parent_lod(lod, profile);
	const MorphBandProfile band = get_morph_band_profile(lod, profile, strategy);

	LODMorphFootprintMetrics m{};
	m.lod = lod;
	m.block_size_m = block_size;
	m.snap_period_m = snap_period;
	m.fine_spacing_m = fine_spacing;
	m.parent_spacing_m = parent_spacing;
	m.has_parent = has_parent;

	if (strategy == MorphStrategy::StrategyB_Defective_4_5B || strategy == MorphStrategy::StrategyB_Legal_3B) {
		m.candidate_half_extent_m = 5.0 * block_size;
		m.candidate_count = (lod == 0) ? 100 : 84;
	} else {
		m.candidate_half_extent_m = static_cast<double>(profile.candidate_grid_radius) * block_size;
		const uint32_t total_cands = static_cast<uint32_t>(4 * profile.candidate_grid_radius * profile.candidate_grid_radius);
		const uint32_t hole_cands = (lod == 0) ? 0 : static_cast<uint32_t>(4 * profile.inner_hole_radius * profile.inner_hole_radius);
		m.candidate_count = total_cands - hole_cands;
	}

	m.stable_fine_half_extent_m = band.fine_zone_radius_m;
	m.transition_band_width_m = band.transition_band_width_m;
	m.parent_locked_margin_m = m.candidate_half_extent_m - band.parent_lock_radius_m;

	const double total_cand_area = 4.0 * m.candidate_half_extent_m * m.candidate_half_extent_m;
	const double fine_detail_area = 4.0 * m.stable_fine_half_extent_m * m.stable_fine_half_extent_m;
	m.fine_detail_area_pct = (total_cand_area > 0.0) ? (fine_detail_area / total_cand_area * 100.0) : 0.0;

	return m;
}

bool get_quad_diagonal_v00_v11(
	TriangulationPattern pattern,
	int32_t ix,
	int32_t iz,
	uint8_t lod
) noexcept {
	switch (pattern) {
		case TriangulationPattern::ProductionLegacyUniform:
			return false;

		case TriangulationPattern::ProductionDiamond:
			return (((ix + iz) & 1) != 0);

		case TriangulationPattern::Reference_T0:
			if (lod == 0) {
				return (((ix + iz) & 1) != 0);
			}
			return false;

		case TriangulationPattern::ConstraintDerived_T1:
		case TriangulationPattern::ConstraintDerived_T2: {
			if (lod <= 3) {
				if ((ix % 2 == 0) || (iz % 2 == 0)) {
					return (((ix + iz) & 1) != 0);
				}
				return false;
			}
			return false;
		}
	}
	return false;
}

void generate_block_indices(
	TriangulationPattern pattern,
	uint8_t lod,
	std::vector<uint32_t>& out_indices
) {
	out_indices.clear();
	out_indices.reserve(16 * 16 * 6);

	for (int32_t iz = 0; iz < 16; ++iz) {
		for (int32_t ix = 0; ix < 16; ++ix) {
			const uint32_t v00 = static_cast<uint32_t>(iz * 17 + ix);
			const uint32_t v10 = static_cast<uint32_t>(iz * 17 + (ix + 1));
			const uint32_t v01 = static_cast<uint32_t>((iz + 1) * 17 + ix);
			const uint32_t v11 = static_cast<uint32_t>((iz + 1) * 17 + (ix + 1));

			const bool use_v00_v11 = get_quad_diagonal_v00_v11(pattern, ix, iz, lod);

			if (use_v00_v11) {
				out_indices.push_back(v00);
				out_indices.push_back(v10);
				out_indices.push_back(v11);

				out_indices.push_back(v00);
				out_indices.push_back(v11);
				out_indices.push_back(v01);
			} else {
				out_indices.push_back(v00);
				out_indices.push_back(v10);
				out_indices.push_back(v01);

				out_indices.push_back(v10);
				out_indices.push_back(v11);
				out_indices.push_back(v01);
			}
		}
	}
}

} // namespace multinet::rendering

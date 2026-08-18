#ifndef MULTINET_BLOCK_CLIPMAP_MORPH_H
#define MULTINET_BLOCK_CLIPMAP_MORPH_H

#include "block_clipmap_profile.h"
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <vector>
#include <array>

namespace multinet::rendering {

/// Maximum possible active recursive morph stages for certified baseline radius 4
inline constexpr uint8_t MAX_RECURSION_DEPTH_BASELINE = 3;

/// Returns the profile-aware maximum active recursion depth bound
[[nodiscard]] inline constexpr uint8_t get_profile_recursion_bound(int32_t candidate_grid_radius) noexcept {
	if (candidate_grid_radius <= 2) return 2;
	if (candidate_grid_radius <= 6) return 3;
	return 4;
}

/// Morph Strategy definition for Phase-B2 candidate evaluation
enum class MorphStrategy : uint8_t {
	StrategyA_Legal_1B_1B_2B = 0,      // Legal 8x8 ring (fine=1.0B, trans=1.0B, parent=2.0B)
	StrategyA_Legal_1_25B_0_75B_2B = 1,// Legal 8x8 ring (fine=1.25B, trans=0.75B, parent=2.0B)
	StrategyA_Legal_1_5B_0_5B_2B = 2,  // Legal 8x8 ring (fine=1.5B, trans=0.5B, parent=2.0B)
	StrategyA_SubmittedDefective = 3,  // Defective R1.2B0 submission (fine=1.5B, trans=1.5B, parent=3.0B)
	StrategyB_Defective_4_5B = 4,      // Defective Strategy B (fine=2.5B, trans=2.0B, parent=4.5B)
	StrategyB_Legal_3B = 5,            // Legal Strategy B (fine=1.5B, trans=1.5B, parent=3.0B)
	ProfileDerived = 6                 // Profile-derived (fine=(r-3)B, trans=1.0B, parent=(r-2)B)
};

/// Distance Metric Law for Spatial Morphing
enum class MorphMetricLaw : uint8_t {
	ChebyshevScalar = 0,  // Scalar mu = f(max(|du|, |dv|)), morphs both axes together (recommended)
	IndependentAxes = 1,  // u morphs by mu_u(du), v morphs by mu_v(dv) (axis decoupled, defective)
	AlgebraicSum = 2      // mu = 1 - (1 - mu_u)(1 - mu_v)
};

/// Triangulation patterns for fine and coarse block quad grids (Production-Exact)
enum class TriangulationPattern : uint8_t {
	ProductionLegacyUniform = 0,       // Production Diamond OFF: every quad -> v10-v01 (use_v00_v11 = false)
	ProductionDiamond = 1,             // Production Diamond ON: odd (x+z) -> v00-v11, even (x+z) -> v10-v01
	Reference_T0 = 2,                  // Production-exact T0: LOD 0 ProductionDiamond, LOD 1..7 ProductionLegacyUniform (0 mismatches)
	ConstraintDerived_T1 = 3,          // Unweighted DSU optimization over D[lod][x][z]
	ConstraintDerived_T2 = 4           // Fine-LOD-weighted DSU optimization over D[lod][x][z]
};

/// Parameters for spatial parent-grid morphing
struct MorphBandProfile {
	double fine_zone_radius_m{ 0.0 };     // Distance within which morph factor == 0.0
	double transition_band_width_m{ 0.0 };// Width of region where 0.0 < morph factor < 1.0
	double parent_lock_radius_m{ 0.0 };   // Distance beyond which morph factor == 1.0
};

/// Evaluation result for a single fine lattice vertex
struct VertexMorphResult {
	double fine_u_m{ 0.0 };
	double fine_v_m{ 0.0 };
	double parent_target_u_m{ 0.0 };
	double parent_target_v_m{ 0.0 };
	double morph_factor_u{ 0.0 };
	double morph_factor_v{ 0.0 };
	double combined_morph_factor{ 0.0 };
	double morphed_u_m{ 0.0 };
	double morphed_v_m{ 0.0 };
	double horizontal_displacement_m{ 0.0 };
	uint8_t active_recursion_depth{ 0 };
	bool is_odd_u{ false };
	bool is_odd_v{ false };
	bool is_fully_parent_locked{ false };
	bool has_parent{ true };
};

/// Footprint & coverage metric analysis for a given LOD and Strategy
struct LODMorphFootprintMetrics {
	uint8_t lod{ 0 };
	double block_size_m{ 0.0 };
	double snap_period_m{ 0.0 };
	double fine_spacing_m{ 0.0 };
	double parent_spacing_m{ 0.0 };
	double candidate_half_extent_m{ 0.0 };
	double stable_fine_half_extent_m{ 0.0 };
	double transition_band_width_m{ 0.0 };
	double parent_locked_margin_m{ 0.0 };
	double fine_detail_area_pct{ 0.0 };
	uint32_t candidate_count{ 0 };
	bool has_parent{ true };
};

// ===========================================================================
// Core Engine-Neutral Morphing & Signed Math Functions
// ===========================================================================

/// Signed lattice floor division by 2 with zero overflow (arithmetic shift)
[[nodiscard]] inline constexpr int64_t signed_floor_div2(int64_t a) noexcept {
	return (a >> 1);
}

/// Signed lattice floor division by arbitrary positive integer b with zero overflow
[[nodiscard]] inline constexpr int64_t signed_floor_div(int64_t a, int64_t b) noexcept {
	int64_t res = a / b;
	int64_t rem = a % b;
	if (rem != 0 && a < 0) {
		res--;
	}
	return res;
}

/// Checks whether an ordinary coarse parent LOD exists for the given level.
[[nodiscard]] inline bool has_parent_lod(
	uint8_t lod,
	const BlockClipmapProfile& profile
) noexcept {
	return (lod + 1 < profile.level_count);
}

/// Computes the exact parent-lattice target coordinate for a single axis in global coordinates.
[[nodiscard]] inline double compute_parent_lattice_target(
	double fine_coord_m,
	double parent_spacing_m
) noexcept {
	return std::floor(fine_coord_m / parent_spacing_m) * parent_spacing_m;
}

/// Computes the exact parent-lattice target using block index and local quad index.
[[nodiscard]] inline double compute_parent_lattice_target_local(
	int64_t block_index,
	int32_t local_quad_index,
	double fine_block_size_m,
	double fine_spacing_m
) noexcept {
	const int32_t collapsed_local = local_quad_index & ~1;
	return static_cast<double>(block_index) * fine_block_size_m +
	       static_cast<double>(collapsed_local) * fine_spacing_m;
}

/// Derives the spatial morph band configuration for a given LOD and strategy.
[[nodiscard]] MorphBandProfile get_morph_band_profile(
	uint8_t lod,
	const BlockClipmapProfile& profile,
	MorphStrategy strategy = MorphStrategy::StrategyA_Legal_1B_1B_2B
) noexcept;

/// Computes the continuous 1D morph factor along a single axis given observer distance.
[[nodiscard]] inline double compute_1d_morph_factor(
	double vertex_axis_pos_m,
	double observer_axis_pos_m,
	const MorphBandProfile& band
) noexcept {
	const double dist = std::abs(vertex_axis_pos_m - observer_axis_pos_m);
	if (dist <= band.fine_zone_radius_m) {
		return 0.0;
	}
	if (dist >= band.parent_lock_radius_m || band.transition_band_width_m <= 1e-9) {
		return 1.0;
	}
	return (dist - band.fine_zone_radius_m) / band.transition_band_width_m;
}

/// Evaluates complete vertex morphing for a fine vertex (one-level baseline).
[[nodiscard]] VertexMorphResult evaluate_vertex_morph(
	int64_t block_bx,
	int64_t block_bv,
	int32_t quad_ix,
	int32_t quad_iz,
	uint8_t lod,
	double observer_u_m,
	double observer_v_m,
	const BlockClipmapProfile& profile,
	MorphStrategy strategy = MorphStrategy::StrategyA_Legal_1B_1B_2B,
	MorphMetricLaw metric_law = MorphMetricLaw::ChebyshevScalar
) noexcept;

/// Evaluates vertex morphing from pure continuous world coordinates (one-level baseline).
[[nodiscard]] VertexMorphResult evaluate_vertex_morph_world(
	double world_u_m,
	double world_v_m,
	uint8_t lod,
	double observer_u_m,
	double observer_v_m,
	const BlockClipmapProfile& profile,
	MorphStrategy strategy = MorphStrategy::StrategyA_Legal_1B_1B_2B,
	MorphMetricLaw metric_law = MorphMetricLaw::ChebyshevScalar
) noexcept;

/// Evaluates Recursive Live-Parent Morphing from pure continuous world coordinates.
[[nodiscard]] VertexMorphResult evaluate_vertex_morph_recursive_world(
	double world_u_m,
	double world_v_m,
	uint8_t lod,
	double observer_u_m,
	double observer_v_m,
	const BlockClipmapProfile& profile,
	MorphStrategy strategy = MorphStrategy::StrategyA_Legal_1B_1B_2B,
	MorphMetricLaw metric_law = MorphMetricLaw::ChebyshevScalar
) noexcept;

/// Evaluates Recursive Live-Parent Morphing for a fine vertex given block placement.
[[nodiscard]] VertexMorphResult evaluate_vertex_morph_recursive(
	int64_t block_bx,
	int64_t block_bv,
	int32_t quad_ix,
	int32_t quad_iz,
	uint8_t lod,
	double observer_u_m,
	double observer_v_m,
	const BlockClipmapProfile& profile,
	MorphStrategy strategy = MorphStrategy::StrategyA_Legal_1B_1B_2B,
	MorphMetricLaw metric_law = MorphMetricLaw::ChebyshevScalar
) noexcept;

/// Evaluates presentation-lattice-rooted recursive morphing in camera-relative flat active frame.
[[nodiscard]] VertexMorphResult evaluate_presentation_vertex_morph_recursive(
	int64_t pres_bx,
	int64_t pres_bv,
	int32_t local_ix,
	int32_t local_iz,
	uint8_t lod,
	double active_cam_u_m,
	double active_cam_v_m,
	const BlockClipmapProfile& profile,
	MorphStrategy strategy = MorphStrategy::StrategyA_Legal_1B_1B_2B,
	MorphMetricLaw metric_law = MorphMetricLaw::ChebyshevScalar
) noexcept;

/// Evaluates vertex morphing with finite rectangular domain clamping.
[[nodiscard]] VertexMorphResult evaluate_vertex_morph_finite_clamped(
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
	MorphStrategy strategy = MorphStrategy::StrategyA_Legal_1B_1B_2B,
	MorphMetricLaw metric_law = MorphMetricLaw::ChebyshevScalar,
	bool use_recursive = true
) noexcept;

/// Computes quantitative footprint metrics for a given LOD and Strategy.
[[nodiscard]] LODMorphFootprintMetrics compute_morph_footprint_metrics(
	uint8_t lod,
	const BlockClipmapProfile& profile,
	MorphStrategy strategy
) noexcept;

/// Determines diagonal direction (true: v00-v11, false: v10-v01) for quad (ix, iz) matching production
[[nodiscard]] bool get_quad_diagonal_v00_v11(
	TriangulationPattern pattern,
	int32_t ix,
	int32_t iz,
	uint8_t lod
) noexcept;

/// Builds shared index buffer for a 16x16 quad block (17x17 vertices) given a triangulation pattern.
void generate_block_indices(
	TriangulationPattern pattern,
	uint8_t lod,
	std::vector<uint32_t>& out_indices
);

} // namespace multinet::rendering

#endif // MULTINET_BLOCK_CLIPMAP_MORPH_H

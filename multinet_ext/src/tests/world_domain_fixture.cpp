#include "multinet/core/spatial/world_domain.h"
#include "multinet/core/spatial/world_manifests.h"
#include "multinet/core/spatial/surface_projection.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_renderer.h"
#include "multinet/rendering/terrain/block_clipmap/terrain_sample_patch.h"
#include "multinet/world/terrain/terrain_recipe.h"
#include "multinet/world/terrain/outputs/rendering/concrete_terrain_render_source.h"

#include <algorithm>
#include <cmath>
#include <array>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>

using namespace Multinet;
using namespace multinet::rendering;

static void require(bool condition, const char* message) {
	if (!condition) {
		std::cerr << "FAILURE: " << message << "\n";
		std::exit(1);
	}
}

int main() {
	std::cout << "## foundation::WORLD-DOMAIN-AREA-01\n";

	WorldDomainInput finite_input;
	finite_input.topology = WorldDomainTopology::FiniteRectangle;
	finite_input.finite.extent_x_m = 500000;
	finite_input.finite.extent_z_m = 400000;
	WorldDomainManifest finite = build_world_domain_manifest(finite_input);
	require(finite.is_valid(), "finite 500x400 km manifest invalid");
	require(finite.canonical_area_m2 == 200000000000ULL, "finite exact area mismatch");
	require(finite.finite.regions_x > 0 && finite.finite.regions_z > 0, "finite regions missing");
	require(finite.finite.actual_region_extent_x_m != finite.finite.actual_region_extent_z_m, "rectangular partition collapsed to square");
	WorldDomainInput finite_square_input = finite_input;
	finite_square_input.finite.extent_x_m = 500000;
	finite_square_input.finite.extent_z_m = 500000;
	WorldDomainManifest finite_square = build_world_domain_manifest(finite_square_input);
	require(finite_square.is_valid() && finite_square.canonical_area_m2 == 250000000000ULL, "finite 500x500 area mismatch");

	WorldDomainInput inactive_a = finite_input;
	WorldDomainInput inactive_b = finite_input;
	inactive_a.closed_surface.area_equivalent_side_m = 5000000;
	inactive_b.closed_surface.area_equivalent_side_m = 32;
	require(build_world_domain_manifest(inactive_a).domain_manifest_hash == build_world_domain_manifest(inactive_b).domain_manifest_hash,
		"inactive closed payload changed finite hash");

	WorldDomainInput closed_input;
	closed_input.closed_surface.area_equivalent_side_m = 5000000;
	WorldDomainManifest closed = build_world_domain_manifest(closed_input);
	require(closed.is_valid(), "closed default manifest invalid");
	require(closed.canonical_area_m2 == 25000000000000ULL, "closed 5000 km area mismatch");
	WorldDomainInput closed_inactive_a = closed_input;
	WorldDomainInput closed_inactive_b = closed_input;
	closed_inactive_a.finite.extent_x_m = 1;
	closed_inactive_a.finite.extent_z_m = 2;
	closed_inactive_b.finite.extent_x_m = 999999;
	closed_inactive_b.finite.extent_z_m = 888888;
	require(build_world_domain_manifest(closed_inactive_a).domain_manifest_hash == build_world_domain_manifest(closed_inactive_b).domain_manifest_hash,
		"inactive finite payload changed closed hash");
	WorldDomainInput closed_500_input = closed_input;
	closed_500_input.closed_surface.area_equivalent_side_m = 500000;
	WorldDomainManifest closed_500 = build_world_domain_manifest(closed_500_input);
	require(closed_500.is_valid() && closed_500.canonical_area_m2 == finite_square.canonical_area_m2,
		"finite 500x500 and closed S=500 area mismatch");
	WorldDomainInput finite_dimension_change = finite_square_input;
	finite_dimension_change.finite.extent_z_m = 501000;
	require(build_world_domain_manifest(finite_dimension_change).domain_manifest_hash != finite_square.domain_manifest_hash,
		"finite dimension change did not change domain hash");
	WorldDomainInput closed_dimension_change = closed_500_input;
	closed_dimension_change.closed_surface.area_equivalent_side_m = 501000;
	require(build_world_domain_manifest(closed_dimension_change).domain_manifest_hash != closed_500.domain_manifest_hash,
		"closed dimension change did not change domain hash");

	// A small closed world must keep the ordinary 2m/32m BCCM geometry, but
	// must not admit outer levels whose block footprint exceeds its chart span.
	const uint64_t required_scales_m[] = { 2000, 32000, 100000, 500000, 5000000, 25000000 };
	for (const uint64_t side_m : required_scales_m) {
		WorldDomainInput closed_scale_input;
		closed_scale_input.closed_surface.area_equivalent_side_m = side_m;
		const WorldDomainManifest closed_scale = build_world_domain_manifest(closed_scale_input);
		require(closed_scale.is_valid(), "required closed scale manifest invalid");
		const uint8_t expected_levels = side_m == 2000 ? 5 : 8;
		require(derive_domain_compatible_bccm_level_count(closed_scale) == expected_levels,
			"closed effective BCCM prefix mismatch");
		const uint8_t expected_flat_levels = side_m == 2000 ? 1 : side_m == 32000 ? 5 : side_m == 100000 ? 6 : 8;
		require(derive_flat_presentation_bccm_level_count(closed_scale) == expected_flat_levels,
			"closed flat-chart BCCM prefix exceeded the non-folding radius");
		TerrainRenderBlockKey key{};
		require(!make_domain_block_key(static_cast<SurfaceFace>(255), 0, 0, 0, closed_scale, key),
			"invalid canonical face admitted for closed domain");
		for (uint8_t lod = 0; lod < BlockClipmapProfile::MAX_LEVELS / 2; ++lod) {
			const bool admitted = make_domain_block_key(SurfaceFace::PositiveX, 0, 0, lod, closed_scale, key);
			require(admitted == (lod < expected_levels), "closed level admission ignored effective prefix");
			if (admitted) require(key.is_valid(), "admitted closed key was invalid");
		}

		WorldDomainInput finite_square_scale_input;
		finite_square_scale_input.topology = WorldDomainTopology::FiniteRectangle;
		finite_square_scale_input.finite.extent_x_m = side_m;
		finite_square_scale_input.finite.extent_z_m = side_m;
		const WorldDomainManifest finite_scale = build_world_domain_manifest(finite_square_scale_input);
		require(finite_scale.is_valid(), "required finite square scale manifest invalid");
		const uint8_t expected_finite_levels = side_m == 2000 ? 6 : 8;
		require(derive_domain_compatible_bccm_level_count(finite_scale) == expected_finite_levels,
			"finite square effective BCCM prefix mismatch");
	}
	for (const uint64_t side_m : { uint64_t{ 10000 }, uint64_t{ 20000 } }) {
		WorldDomainInput tiny_closed_input;
		tiny_closed_input.closed_surface.area_equivalent_side_m = side_m;
		const WorldDomainManifest tiny_closed = build_world_domain_manifest(tiny_closed_input);
		const uint8_t expected_flat_levels = side_m == 10000 ? 3 : 4;
		require(tiny_closed.is_valid() &&
			derive_flat_presentation_bccm_level_count(tiny_closed) == expected_flat_levels,
			"10/20 km flat-chart coverage prefix mismatch");
	}

	// The flat presentation grid must remain edge-adjacent even though a
	// 100 km face is not divisible by any ordinary BCCM block size.
	WorldDomainInput closed_100_input;
	closed_100_input.closed_surface.area_equivalent_side_m = 100000;
	const WorldDomainManifest closed_100 = build_world_domain_manifest(closed_100_input);
	require(closed_100.is_valid(), "100 km wrapping fixture manifest invalid");
	// The closed flat-view coverage control is deliberately bounded by the
	// preallocated 256-instance finest-level buffer. These are coverage choices,
	// not a claim that one planar chart can represent the whole closed surface.
	for (const int32_t radius : { 4, 6, 8 }) {
		auto coverage_renderer = std::make_unique<BlockClipmapRenderer>();
		require(coverage_renderer->set_candidate_grid_radius(radius),
			"supported closed flat coverage radius rejected");
		const uint8_t levels = derive_flat_presentation_bccm_level_count(closed_100, 32.0, radius, 8);
		require(levels == 6, "100 km coverage radius changed the guarded flat LOD prefix");
		coverage_renderer->test_set_profile_levels(levels);
		const double expected_extent_m = static_cast<double>(radius) * 2.0 * 1024.0;
		require(std::abs(coverage_renderer->get_effective_coverage_extent_m() - expected_extent_m) < 1e-9,
			"closed flat coverage extent mismatch");
		const double expected_corner_m = (static_cast<double>(radius) + 1.0) * 1024.0 * 1.41421356237309504880;
		require(std::abs(coverage_renderer->get_effective_coverage_corner_radius_m() - expected_corner_m) < 1e-9,
			"closed flat coverage corner radius mismatch");
	}
	auto rejected_coverage_renderer = std::make_unique<BlockClipmapRenderer>();
	require(!rejected_coverage_renderer->set_candidate_grid_radius(9),
		"coverage radius exceeded the fixed 256-instance capacity");
	const double h100 = static_cast<double>(closed_100.closed_surface.chart_half_extent_mm) * 0.001;
	SurfaceFrame observer_100{};
	require(try_make_flat_surface_frame_for_face(SurfaceFace::PositiveX, observer_100),
		"100 km presentation observer frame unavailable");
	observer_100.origin.face = SurfaceFace::PositiveX;
	observer_100.origin.u_m = h100 - 8.0;
	observer_100.origin.v_m = 0.0;
	observer_100.origin.altitude_m = 3000.0;
	observer_100.origin.topology_version = closed_100.topology_version;
	observer_100.origin.projection_version = closed_100.projection_version;
	observer_100.topology_version = closed_100.topology_version;
	observer_100.projection_version = closed_100.projection_version;
	observer_100.frame_epoch = 1;

	TerrainSamplePatchKey patch_a{};
	TerrainSamplePatchKey patch_b{};
	uint32_t patch_b_crossings = 0;
	require(try_make_sample_patch(observer_100, 0.0, 0.0, 0, ORDINARY_BCCM_V1_PROFILE, closed_100, patch_a),
		"100 km source-side sample patch failed");
	require(try_make_sample_patch(observer_100, 32.0, 0.0, 0, ORDINARY_BCCM_V1_PROFILE, closed_100, patch_b, &patch_b_crossings),
		"100 km destination-side sample patch failed");
	require(patch_b_crossings > 0 && patch_a.anchor_face != patch_b.anchor_face,
		"100 km adjacent patch did not cross the canonical edge");
	SurfacePosition64 shared_from_a{};
	SurfacePosition64 shared_from_b{};
	require(try_sample_patch_position(patch_a, 16.0, 0.0, closed_100, shared_from_a),
		"100 km source shared edge sample failed");
	require(try_sample_patch_position(patch_b, -16.0, 0.0, closed_100, shared_from_b),
		"100 km destination shared edge sample failed");
	require(shared_from_a.face == shared_from_b.face &&
		std::abs(shared_from_a.u_m - shared_from_b.u_m) < 1e-9 &&
		std::abs(shared_from_a.v_m - shared_from_b.v_m) < 1e-9,
		"100 km adjacent presentation blocks disagree at their shared edge");

	TerrainPresentationBlockKey presentation_a{ 0, 0, 1, 0, ORDINARY_BCCM_V1_PROFILE };
	TerrainPresentationBlockKey presentation_b{ 1, 0, 1, 0, ORDINARY_BCCM_V1_PROFILE };
	TerrainRenderBlockKey owner_a{ patch_a.anchor_face, 0, 0, 0, ORDINARY_BCCM_V1_PROFILE, 0 };
	TerrainRenderBlockKey owner_b{ patch_b.anchor_face, 0, 0, 0, ORDINARY_BCCM_V1_PROFILE, 0 };
	TerrainFallbackBounds placement_bounds{ -100.0f, 2000.0f, 1.0f, 0.0f };
	auto placement_renderer = std::make_unique<BlockClipmapRenderer>();
	const BlockPlacement placement_a = placement_renderer->build_presentation_block_placement(
		presentation_a, owner_a, 16.0, 0.0, placement_bounds);
	const BlockPlacement placement_b = placement_renderer->build_presentation_block_placement(
		presentation_b, owner_b, 16.0, 0.0, placement_bounds);
	require(placement_a.valid && placement_b.valid,
		"100 km presentation placement returned an invalid transform");
	require(std::abs((placement_a.local_origin.x + 32.0) - placement_b.local_origin.x) < 1e-6 &&
		std::abs(placement_a.local_origin.z - placement_b.local_origin.z) < 1e-6,
		"100 km presentation cells are not exactly edge-adjacent");

	// Exercise every directed topology edge at every ordinary LOD. A shared
	// presentation edge must resolve to the same canonical samples even when
	// the two block centres live on different faces and orientations.
	uint32_t qualified_patch_edges = 0;
	uint32_t qualified_patch_samples = 0;
	for (uint8_t source_face_index = 0; source_face_index < 6; ++source_face_index) {
		for (uint8_t edge_index = 0; edge_index < 4; ++edge_index) {
			const SurfaceFace source_face = static_cast<SurfaceFace>(source_face_index);
			const SurfaceEdge edge = static_cast<SurfaceEdge>(edge_index);
			for (uint8_t lod = 0; lod < 8; ++lod) {
				const double block_m = BlockClipmapProfile{}.get_lod_block_size(lod);
				const double spacing_m = BlockClipmapProfile{}.get_lod_spacing(lod);
				const double half_block_m = block_m * 0.5;
				SurfaceFrame edge_observer{};
				require(try_make_flat_surface_frame_for_face(source_face, edge_observer),
					"exhaustive patch seam observer frame unavailable");
				edge_observer.origin.face = source_face;
				edge_observer.origin.topology_version = closed_100.topology_version;
				edge_observer.origin.projection_version = closed_100.projection_version;
				edge_observer.topology_version = closed_100.topology_version;
				edge_observer.projection_version = closed_100.projection_version;
				edge_observer.frame_epoch = 1;

				double outward_x = 0.0;
				double outward_z = 0.0;
				double along_x = 0.0;
				double along_z = 0.0;
				if (edge == SurfaceEdge::NegativeU || edge == SurfaceEdge::PositiveU) {
					const double sign = edge == SurfaceEdge::PositiveU ? 1.0 : -1.0;
					edge_observer.origin.u_m = sign * (h100 - block_m * 0.25);
					outward_x = sign * edge_observer.tangent_basis.u_axis.x;
					outward_z = sign * edge_observer.tangent_basis.u_axis.z;
					along_x = edge_observer.tangent_basis.v_axis.x;
					along_z = edge_observer.tangent_basis.v_axis.z;
				} else {
					const double sign = edge == SurfaceEdge::PositiveV ? 1.0 : -1.0;
					edge_observer.origin.v_m = sign * (h100 - block_m * 0.25);
					outward_x = sign * edge_observer.tangent_basis.v_axis.x;
					outward_z = sign * edge_observer.tangent_basis.v_axis.z;
					along_x = edge_observer.tangent_basis.u_axis.x;
					along_z = edge_observer.tangent_basis.u_axis.z;
				}

				TerrainSamplePatchKey inside_patch{};
				TerrainSamplePatchKey outside_patch{};
				require(try_make_sample_patch(
					edge_observer, 0.0, 0.0, lod, ORDINARY_BCCM_V1_PROFILE,
					closed_100, inside_patch),
					"inside exhaustive patch seam mapping failed");
				require(try_make_sample_patch(
					edge_observer, outward_x * block_m, outward_z * block_m,
					lod, ORDINARY_BCCM_V1_PROFILE, closed_100, outside_patch),
					"outside exhaustive patch seam mapping failed");
				require(inside_patch.anchor_face != outside_patch.anchor_face,
					"exhaustive patch pair did not cross its directed topology edge");

				for (uint32_t sample_index = 0; sample_index < BlockClipmapProfile::VERTS_PER_EDGE; ++sample_index) {
					const double along_m = -half_block_m + static_cast<double>(sample_index) * spacing_m;
					SurfacePosition64 from_inside{};
					SurfacePosition64 from_outside{};
					require(try_sample_patch_position(
						inside_patch,
						outward_x * half_block_m + along_x * along_m,
						outward_z * half_block_m + along_z * along_m,
						closed_100, from_inside),
						"inside shared edge sample mapping failed");
					require(try_sample_patch_position(
						outside_patch,
						-outward_x * half_block_m + along_x * along_m,
						-outward_z * half_block_m + along_z * along_m,
						closed_100, from_outside),
						"outside shared edge sample mapping failed");
					require(from_inside.face == from_outside.face &&
						std::abs(from_inside.u_m - from_outside.u_m) < 1e-6 &&
						std::abs(from_inside.v_m - from_outside.v_m) < 1e-6,
						"adjacent closed patches disagree along a rendered shared edge");
					++qualified_patch_samples;
				}
				++qualified_patch_edges;
			}
		}
	}
	require(qualified_patch_edges == 24u * 8u &&
		qualified_patch_samples == 24u * 8u * BlockClipmapProfile::VERTS_PER_EDGE,
		"exhaustive patch seam qualification count mismatch");

	// Reproduce the saved 100 km editor position that exposed the remaining
	// disconnect. Unlike the same-LOD test above, this compares independently
	// anchored fine/coarse patches along every clipmap ring boundary near a
	// canonical corner. The cube unfolding has cuts, so this is the dangerous
	// case for path-dependent mappings.
	struct RingPatch {
		int64_t block_u{ 0 };
		int64_t block_v{ 0 };
		TerrainSamplePatchKey patch{};
	};
	std::array<std::array<RingPatch, BlockClipmapProfile::MAX_CANDIDATES>, 8> ring_patches{};
	std::array<uint32_t, 8> ring_patch_counts{};
	const double saved_presentation_x_m = -23967.78;
	const double saved_presentation_z_m = -18589.145;
	SurfaceFrame saved_origin{};
	require(try_make_flat_surface_frame_for_face(SurfaceFace::PositiveX, saved_origin),
		"saved editor origin frame unavailable");
	saved_origin.origin.face = SurfaceFace::PositiveX;
	saved_origin.origin.topology_version = closed_100.topology_version;
	saved_origin.origin.projection_version = closed_100.projection_version;
	saved_origin.topology_version = closed_100.topology_version;
	saved_origin.projection_version = closed_100.projection_version;
	saved_origin.frame_epoch = 1;
	SurfaceFrame saved_observer{};
	SurfacePosition64 saved_observer_position{};
	uint32_t saved_observer_crossings = 0;
	require(try_advance_domain_surface_frame(
		FramePosition64{ saved_presentation_x_m, 2727.7258, saved_presentation_z_m },
		closed_100, saved_origin, saved_observer, saved_observer_position,
		saved_observer_crossings),
		"saved editor position could not be folded to canonical authority");
	saved_observer.origin = saved_observer_position;

	for (uint8_t lod = 0; lod < 8; ++lod) {
		const double block_m = BlockClipmapProfile{}.get_lod_block_size(lod);
		const double snap_m = lod + 1 < 8
			? BlockClipmapProfile{}.get_lod_block_size(lod + 1) : block_m;
		const int64_t center_u = static_cast<int64_t>(std::floor(
			(std::floor(saved_presentation_x_m / snap_m) * snap_m) / block_m));
		const int64_t center_v = static_cast<int64_t>(std::floor(
			(std::floor(saved_presentation_z_m / snap_m) * snap_m) / block_m));
		int32_t hole_u = 0;
		int32_t hole_v = 0;
		if (lod > 0) {
			hole_u = static_cast<int32_t>(std::floor(saved_presentation_x_m / block_m) - center_u);
			hole_v = static_cast<int32_t>(std::floor(saved_presentation_z_m / block_m) - center_v);
		}
		for (int32_t dv = -4; dv < 4; ++dv) {
			for (int32_t du = -4; du < 4; ++du) {
				if (lod > 0) {
					const int32_t hu = du - hole_u;
					const int32_t hv = dv - hole_v;
					if (hu >= -2 && hu < 2 && hv >= -2 && hv < 2) continue;
				}
				RingPatch& candidate = ring_patches[lod][ring_patch_counts[lod]++];
				candidate.block_u = center_u + du;
				candidate.block_v = center_v + dv;
				const double centre_x = (static_cast<double>(candidate.block_u) + 0.5) * block_m;
				const double centre_z = (static_cast<double>(candidate.block_v) + 0.5) * block_m;
				require(try_make_coherent_sample_patch(
					saved_origin,
					0.0,
					0.0,
					centre_x,
					centre_z,
					1,
					lod, ORDINARY_BCCM_V1_PROFILE, closed_100, candidate.patch),
					"saved editor ring patch mapping failed");
			}
		}
	}

	auto physical_gap = [h100](const SurfacePosition64& a, const SurfacePosition64& b) {
		const FramePosition64 pa = ProjectionCOBE::map_forward(
			static_cast<int>(a.face), a.u_m / h100, a.v_m / h100);
		const FramePosition64 pb = ProjectionCOBE::map_forward(
			static_cast<int>(b.face), b.u_m / h100, b.v_m / h100);
		const double dx = pa.x - pb.x;
		const double dy = pa.y - pb.y;
		const double dz = pa.z - pb.z;
		return std::sqrt(dx * dx + dy * dy + dz * dz);
	};
	uint32_t cross_lod_segments = 0;
	uint32_t cross_lod_samples = 0;
	double maximum_cross_lod_physical_gap = 0.0;
	uint8_t worst_fine_lod = 0;
	int64_t worst_fine_u = 0;
	int64_t worst_fine_v = 0;
	int64_t worst_coarse_u = 0;
	int64_t worst_coarse_v = 0;
	double worst_presentation_x = 0.0;
	double worst_presentation_z = 0.0;
	SurfacePosition64 worst_fine_position{};
	SurfacePosition64 worst_coarse_position{};
	for (uint8_t fine_lod = 0; fine_lod < 7; ++fine_lod) {
		const double fine_block_m = BlockClipmapProfile{}.get_lod_block_size(fine_lod);
		const double shared_step_m = BlockClipmapProfile{}.get_lod_spacing(fine_lod + 1);
		for (uint32_t fine_index = 0; fine_index < ring_patch_counts[fine_lod]; ++fine_index) {
			const RingPatch& fine = ring_patches[fine_lod][fine_index];
			for (uint32_t coarse_index = 0; coarse_index < ring_patch_counts[fine_lod + 1]; ++coarse_index) {
				const RingPatch& coarse = ring_patches[fine_lod + 1][coarse_index];
				const int64_t fine_min_u = fine.block_u;
				const int64_t fine_max_u = fine.block_u + 1;
				const int64_t fine_min_v = fine.block_v;
				const int64_t fine_max_v = fine.block_v + 1;
				const int64_t coarse_min_u = coarse.block_u * 2;
				const int64_t coarse_max_u = coarse_min_u + 2;
				const int64_t coarse_min_v = coarse.block_v * 2;
				const int64_t coarse_max_v = coarse_min_v + 2;
				const bool shared_u_edge = fine_max_u == coarse_min_u || fine_min_u == coarse_max_u;
				const bool shared_v_edge = fine_max_v == coarse_min_v || fine_min_v == coarse_max_v;
				const int64_t overlap_u_min = std::max(fine_min_u, coarse_min_u);
				const int64_t overlap_u_max = std::min(fine_max_u, coarse_max_u);
				const int64_t overlap_v_min = std::max(fine_min_v, coarse_min_v);
				const int64_t overlap_v_max = std::min(fine_max_v, coarse_max_v);
				if (!(shared_u_edge && overlap_v_max > overlap_v_min) &&
					!(shared_v_edge && overlap_u_max > overlap_u_min)) continue;

				const double fine_centre_x = (static_cast<double>(fine.block_u) + 0.5) * fine_block_m;
				const double fine_centre_z = (static_cast<double>(fine.block_v) + 0.5) * fine_block_m;
				const double coarse_block_m = fine_block_m * 2.0;
				const double coarse_centre_x = (static_cast<double>(coarse.block_u) + 0.5) * coarse_block_m;
				const double coarse_centre_z = (static_cast<double>(coarse.block_v) + 0.5) * coarse_block_m;
				for (uint32_t point_index = 0; point_index <= 8; ++point_index) {
					double presentation_x = 0.0;
					double presentation_z = 0.0;
					if (shared_u_edge) {
						presentation_x = static_cast<double>(fine_max_u == coarse_min_u ? fine_max_u : fine_min_u) * fine_block_m;
						presentation_z = static_cast<double>(overlap_v_min) * fine_block_m +
							static_cast<double>(point_index) * shared_step_m;
					} else {
						presentation_x = static_cast<double>(overlap_u_min) * fine_block_m +
							static_cast<double>(point_index) * shared_step_m;
						presentation_z = static_cast<double>(fine_max_v == coarse_min_v ? fine_max_v : fine_min_v) * fine_block_m;
					}
					SurfacePosition64 fine_position{};
					SurfacePosition64 coarse_position{};
					require(try_sample_patch_position(
						fine.patch, presentation_x - fine_centre_x, presentation_z - fine_centre_z,
						closed_100, fine_position),
						"saved editor fine ring sample failed");
					require(try_sample_patch_position(
						coarse.patch, presentation_x - coarse_centre_x, presentation_z - coarse_centre_z,
						closed_100, coarse_position),
						"saved editor coarse ring sample failed");
					const double gap = physical_gap(fine_position, coarse_position);
					if (gap > maximum_cross_lod_physical_gap) {
						maximum_cross_lod_physical_gap = gap;
						worst_fine_lod = fine_lod;
						worst_fine_u = fine.block_u;
						worst_fine_v = fine.block_v;
						worst_coarse_u = coarse.block_u;
						worst_coarse_v = coarse.block_v;
						worst_presentation_x = presentation_x;
						worst_presentation_z = presentation_z;
						worst_fine_position = fine_position;
						worst_coarse_position = coarse_position;
					}
					++cross_lod_samples;
				}
				++cross_lod_segments;
			}
		}
	}
	require(cross_lod_segments > 0 && cross_lod_samples == cross_lod_segments * 9,
		"saved editor cross-LOD seam coverage was not exercised");
	std::cout << "Saved-camera cross-LOD worst gap: "
		<< maximum_cross_lod_physical_gap * closed_100.closed_surface.logical_area_radius_m << " m"
		<< " | fine LOD " << static_cast<int>(worst_fine_lod)
		<< " block " << worst_fine_u << "," << worst_fine_v
		<< " | coarse block " << worst_coarse_u << "," << worst_coarse_v
		<< " | presentation " << worst_presentation_x << "," << worst_presentation_z
		<< " | fine face/uv " << static_cast<int>(worst_fine_position.face) << "/"
		<< worst_fine_position.u_m << "," << worst_fine_position.v_m
		<< " | coarse face/uv " << static_cast<int>(worst_coarse_position.face) << "/"
		<< worst_coarse_position.u_m << "," << worst_coarse_position.v_m
		<< std::endl;
	require(maximum_cross_lod_physical_gap * closed_100.closed_surface.logical_area_radius_m < 0.005,
		"fine and coarse wrapping patches disagree on their shared physical surface");

	// A welded edge is not enough: a path branch may still map neighbouring
	// presentation samples to distant canonical terrain and create a vertical
	// wall. Compare the fixed global root with a root rebased at the screenshot
	// camera over an 8 km diagnostic footprint.
	auto maximum_adjacent_gap_m = [&](const SurfaceFrame& root, double root_x, double root_z,
		double center_x, double center_z, uint64_t generation) {
		constexpr int grid_radius = 256;
		constexpr double step_m = 64.0;
		constexpr size_t grid_width = grid_radius * 2 + 1;
		TerrainSamplePatchKey diagnostic_patch{};
		require(try_make_coherent_sample_patch(
			root, root_x, root_z, center_x, center_z, generation, 0,
			ORDINARY_BCCM_V1_PROFILE, closed_100, diagnostic_patch),
			"branch continuity diagnostic patch failed");
		std::array<SurfacePosition64, grid_width> previous_row{};
		double max_gap_m = 0.0;
		for (int iz = -grid_radius; iz <= grid_radius; ++iz) {
			SurfacePosition64 previous_in_row{};
			for (int ix = -grid_radius; ix <= grid_radius; ++ix) {
				SurfacePosition64 position{};
				require(try_sample_patch_position(
					diagnostic_patch, static_cast<double>(ix) * step_m,
					static_cast<double>(iz) * step_m, closed_100, position),
					"branch continuity diagnostic sample failed");
				const size_t column = static_cast<size_t>(ix + grid_radius);
				if (ix > -grid_radius) {
					max_gap_m = std::max(max_gap_m,
						physical_gap(previous_in_row, position) * closed_100.closed_surface.logical_area_radius_m);
				}
				if (iz > -grid_radius) {
					max_gap_m = std::max(max_gap_m,
						physical_gap(previous_row[column], position) * closed_100.closed_surface.logical_area_radius_m);
				}
				previous_in_row = position;
				previous_row[column] = position;
			}
		}
		return max_gap_m;
	};
	const double cliff_camera_x = 63065.3;
	const double cliff_camera_z = -107275.9;
	SurfaceFrame cliff_local_root{};
	SurfacePosition64 cliff_local_position{};
	uint32_t cliff_root_crossings = 0;
	require(try_advance_domain_surface_frame(
		FramePosition64{ cliff_camera_x, 0.0, cliff_camera_z }, closed_100,
		saved_origin, cliff_local_root, cliff_local_position, cliff_root_crossings),
		"cliff screenshot camera could not be folded to canonical authority");
	cliff_local_root.origin = cliff_local_position;
	const double global_root_adjacent_gap_m = maximum_adjacent_gap_m(
		saved_origin, 0.0, 0.0, cliff_camera_x, cliff_camera_z, 2);
	const double local_root_adjacent_gap_m = maximum_adjacent_gap_m(
		cliff_local_root, cliff_camera_x, cliff_camera_z, cliff_camera_x, cliff_camera_z, 3);
	std::cout << "Cliff-camera 64 m neighbour gap over 32 km: global_root="
		<< global_root_adjacent_gap_m << " m | local_root="
		<< local_root_adjacent_gap_m << " m" << std::endl;

	// A cube net cannot cover a neighbourhood around a cube corner without a
	// cut or overlap. Test a continuous logical-sphere lookup chart before it
	// is allowed anywhere near the runtime renderer. The BCCM vertices remain
	// flat; only their canonical sample addresses follow the closed surface.
	TerrainSamplePatchKey logical_chart_patch{};
	require(try_make_logical_sample_patch(
		cliff_local_root,
		cliff_camera_x,
		cliff_camera_z,
		cliff_camera_x,
		cliff_camera_z,
		4,
		0,
		ORDINARY_BCCM_V1_PROFILE,
		closed_100,
		logical_chart_patch),
		"logical chart patch construction failed");
	require(logical_chart_patch.mapping_version == TERRAIN_SAMPLE_PATCH_MAPPING_LOCAL_EXP_CHART_V5,
		"runtime logical chart did not use the local exponential V5 identity");
	LogicalSampleChart logical_chart{};
	require(try_build_logical_sample_chart(cliff_local_root, closed_100, logical_chart),
		"logical chart basis construction failed");
	const FramePosition64& logical_root_x = logical_chart.presentation_x_angular_tangent;
	const FramePosition64& logical_root_z = logical_chart.presentation_z_angular_tangent;
	auto logical_chart_sample = [&](double presentation_dx_m, double presentation_dz_m,
		SurfacePosition64& out) {
		return try_sample_patch_position(
			logical_chart_patch, presentation_dx_m, presentation_dz_m, closed_100, out);
	};
	const double safe_local_radius_m = closed_flat_chart_max_radius_m(closed_100);
	require(std::abs(safe_local_radius_m -
		closed_100.closed_surface.logical_area_radius_m * 0.5) < 1e-9,
		"local exponential chart metric radius drifted");

	constexpr int logical_grid_radius = 256;
	constexpr double logical_step_m = 64.0;
	constexpr size_t logical_grid_width = logical_grid_radius * 2 + 1;
	std::array<SurfacePosition64, logical_grid_width> logical_previous_row{};
	double logical_chart_adjacent_gap_m = 0.0;
	for (int iz = -logical_grid_radius; iz <= logical_grid_radius; ++iz) {
		SurfacePosition64 previous_in_row{};
		for (int ix = -logical_grid_radius; ix <= logical_grid_radius; ++ix) {
			SurfacePosition64 position{};
			require(logical_chart_sample(
				static_cast<double>(ix) * logical_step_m,
				static_cast<double>(iz) * logical_step_m, position),
				"logical chart continuity sample failed");
			const size_t column = static_cast<size_t>(ix + logical_grid_radius);
			if (ix > -logical_grid_radius) {
				logical_chart_adjacent_gap_m = std::max(logical_chart_adjacent_gap_m,
					physical_gap(previous_in_row, position) * closed_100.closed_surface.logical_area_radius_m);
			}
			if (iz > -logical_grid_radius) {
				logical_chart_adjacent_gap_m = std::max(logical_chart_adjacent_gap_m,
					physical_gap(logical_previous_row[column], position) * closed_100.closed_surface.logical_area_radius_m);
			}
			previous_in_row = position;
			logical_previous_row[column] = position;
		}
	}
	const double tangent_x_sq = logical_root_x.x * logical_root_x.x +
		logical_root_x.y * logical_root_x.y + logical_root_x.z * logical_root_x.z;
	const double tangent_z_sq = logical_root_z.x * logical_root_z.x +
		logical_root_z.y * logical_root_z.y + logical_root_z.z * logical_root_z.z;
	const double tangent_cross = logical_root_x.x * logical_root_z.x +
		logical_root_x.y * logical_root_z.y + logical_root_x.z * logical_root_z.z;
	const double tangent_discriminant = std::sqrt(
		(tangent_x_sq - tangent_z_sq) * (tangent_x_sq - tangent_z_sq) +
		4.0 * tangent_cross * tangent_cross);
	const double maximum_local_scale = std::sqrt(
		0.5 * (tangent_x_sq + tangent_z_sq + tangent_discriminant));
	const double logical_chart_step_bound_m =
		closed_100.closed_surface.logical_area_radius_m * maximum_local_scale * logical_step_m;
	std::cout << "Cliff-camera logical-sphere chart 64 m neighbour gap over 32 km: "
		<< logical_chart_adjacent_gap_m << " m | Jacobian bound "
		<< logical_chart_step_bound_m << " m" << std::endl;
	require(logical_chart_adjacent_gap_m <= logical_chart_step_bound_m + 0.1,
		"logical-sphere chart contains a discontinuous canonical sample jump");

	// A diagonal step across a cube vertex changes two face aliases at once.
	// The chart axes must follow the physical direction by a tiny rotation, not
	// inherit the discrete signed-permutation selected by edge canonicalization.
	const double corner_extent_m = static_cast<double>(closed_100.closed_surface.chart_half_extent_mm) * 0.001;
	SurfaceFrame corner_before{};
	require(try_make_flat_surface_frame_for_face(SurfaceFace::PositiveX, corner_before),
		"corner chart source frame unavailable");
	corner_before.origin.face = SurfaceFace::PositiveX;
	corner_before.origin.u_m = corner_extent_m - 0.5;
	corner_before.origin.v_m = corner_extent_m - 0.5;
	corner_before.origin.altitude_m = 0.0;
	corner_before.origin.topology_version = closed_100.topology_version;
	corner_before.origin.projection_version = closed_100.projection_version;
	corner_before.topology_version = closed_100.topology_version;
	corner_before.projection_version = closed_100.projection_version;
	LogicalSampleChart corner_chart_before{};
	require(try_build_logical_sample_chart(corner_before, closed_100, corner_chart_before),
		"corner chart source construction failed");
	// A held presentation-plane direction must advance the observer along the
	// same V5 direction that the visible mesh samples. This is intentionally not
	// the transported cube-frame delta used by the old control path.
	FramePosition64 v5_corner_delta{};
	require(try_map_logical_chart_delta_to_face_delta(
		corner_chart_before, corner_before.origin, closed_100, 1.0, 1.0, v5_corner_delta),
		"corner V5 presentation delta could not map to face motion");
	SurfaceFrame v5_corner_after{};
	SurfacePosition64 v5_corner_position{};
	uint32_t v5_corner_crossings = 0;
	require(try_advance_domain_surface_frame(
		v5_corner_delta, closed_100, corner_before,
		v5_corner_after, v5_corner_position, v5_corner_crossings),
		"corner V5 mapped advance failed");
	v5_corner_after.origin = v5_corner_position;
	SurfacePosition64 v5_expected_first{};
	require(try_sample_logical_chart(corner_chart_before, 1.0, 1.0, closed_100, v5_expected_first),
		"corner V5 first expected sample failed");
	const FramePosition64 v5_actual_first = ProjectionCOBE::map_forward(
		static_cast<int>(v5_corner_position.face),
		v5_corner_position.u_m / corner_extent_m,
		v5_corner_position.v_m / corner_extent_m);
	require(v5_actual_first.x * ProjectionCOBE::map_forward(static_cast<int>(v5_expected_first.face),
		v5_expected_first.u_m / corner_extent_m, v5_expected_first.v_m / corner_extent_m).x +
		v5_actual_first.y * ProjectionCOBE::map_forward(static_cast<int>(v5_expected_first.face),
		v5_expected_first.u_m / corner_extent_m, v5_expected_first.v_m / corner_extent_m).y +
		v5_actual_first.z * ProjectionCOBE::map_forward(static_cast<int>(v5_expected_first.face),
		v5_expected_first.u_m / corner_extent_m, v5_expected_first.v_m / corner_extent_m).z > 1.0 - 1e-8,
		"corner V5 observer move diverged from the rendered presentation direction");
	LogicalSampleChart v5_corner_chart_after{};
	require(try_transport_logical_sample_chart(corner_chart_before, v5_corner_after, closed_100, v5_corner_chart_after),
		"corner V5 chart could not transport for held input");
	FramePosition64 v5_second_delta{};
	require(try_map_logical_chart_delta_to_face_delta(
		v5_corner_chart_after, v5_corner_position, closed_100, 1.0, 1.0, v5_second_delta),
		"corner V5 held delta could not map to face motion");
	SurfaceFrame v5_second_after{};
	SurfacePosition64 v5_second_position{};
	uint32_t v5_second_crossings = 0;
	require(try_advance_domain_surface_frame(
		v5_second_delta, closed_100, v5_corner_after,
		v5_second_after, v5_second_position, v5_second_crossings),
		"corner V5 held mapped advance failed");
	SurfacePosition64 v5_expected_second{};
	require(try_sample_logical_chart(v5_corner_chart_after, 1.0, 1.0, closed_100, v5_expected_second),
		"corner V5 held expected sample failed");
	const FramePosition64 v5_actual_second = ProjectionCOBE::map_forward(
		static_cast<int>(v5_second_position.face),
		v5_second_position.u_m / corner_extent_m,
		v5_second_position.v_m / corner_extent_m);
	const FramePosition64 v5_expected_second_direction = ProjectionCOBE::map_forward(
		static_cast<int>(v5_expected_second.face),
		v5_expected_second.u_m / corner_extent_m,
		v5_expected_second.v_m / corner_extent_m);
	require(v5_actual_second.x * v5_expected_second_direction.x +
		v5_actual_second.y * v5_expected_second_direction.y +
		v5_actual_second.z * v5_expected_second_direction.z > 1.0 - 1e-8,
		"held corner V5 input diverged from the rendered presentation direction");
	std::cout << "corner_v5_motion_alignment=1\n";

	// The editor consumes V5 deltas one viewport tick at a time. Walk well past
	// a three-face corner so a rejected mapping cannot strand presentation and
	// canonical state on opposite sides of the crossing.
	auto walk_v5_corner_input = [&](const WorldDomainManifest& domain, const SurfaceFrame& start,
		double presentation_dx_m, double presentation_dz_m, int steps) {
		LogicalSampleChart chart{};
		require(try_build_logical_sample_chart(start, domain, chart),
			"three-face V5 walk could not build its starting chart");
		SurfaceFrame frame = start;
		SurfacePosition64 position = start.origin;
		for (int step = 0; step < steps; ++step) {
			FramePosition64 delta{};
			require(try_map_logical_chart_delta_to_face_delta(
				chart, position, domain, presentation_dx_m, presentation_dz_m, delta),
				"three-face V5 walk rejected a presentation delta");
			SurfaceFrame next_frame{};
			SurfacePosition64 next_position{};
			uint32_t crossings = 0;
			require(try_advance_domain_surface_frame(
				delta, domain, frame, next_frame, next_position, crossings),
				"three-face V5 walk rejected a canonical advance");
			next_frame.origin = next_position;
			LogicalSampleChart next_chart{};
			require(try_transport_logical_sample_chart(chart, next_frame, domain, next_chart),
				"three-face V5 walk rejected chart transport");
			chart = next_chart;
			frame = next_frame;
			position = next_position;
		}
	};
	walk_v5_corner_input(closed_100, corner_before, 1.0, 1.0, 4096);
	SurfaceFrame large_corner_before{};
	require(try_make_flat_surface_frame_for_face(SurfaceFace::PositiveX, large_corner_before),
		"large-world corner source frame unavailable");
	const double large_corner_extent_m = static_cast<double>(closed.closed_surface.chart_half_extent_mm) * 0.001;
	large_corner_before.origin.face = SurfaceFace::PositiveX;
	large_corner_before.origin.u_m = large_corner_extent_m - 0.5;
	large_corner_before.origin.v_m = large_corner_extent_m - 0.5;
	large_corner_before.origin.topology_version = closed.topology_version;
	large_corner_before.origin.projection_version = closed.projection_version;
	large_corner_before.topology_version = closed.topology_version;
	large_corner_before.projection_version = closed.projection_version;
	walk_v5_corner_input(closed, large_corner_before, 1.0, 1.0, 4096);
	std::cout << "corner_v5_continuous_editor_walk=1\n";

	SurfaceFrame corner_after{};
	SurfacePosition64 corner_after_position{};
	uint32_t corner_transitions = 0;
	require(try_advance_domain_surface_frame(
		FramePosition64{ 1.0, 0.0, 1.0 }, closed_100, corner_before,
		corner_after, corner_after_position, corner_transitions),
		"diagonal corner transport failed");
	require(corner_transitions == 2, "diagonal corner did not cross exactly two face edges");
	corner_after.origin = corner_after_position;

	// The endpoint may be beyond two edges while the travel path meets V first.
	// The old endpoint canonicalizer always resolved U first, so one large move
	// diverged from the same move split at its real first crossing.
	SurfaceFrame ordered_start{};
	require(try_make_flat_surface_frame_for_face(SurfaceFace::PositiveX, ordered_start),
		"ordered corner source frame unavailable");
	ordered_start.origin.face = SurfaceFace::PositiveX;
	ordered_start.origin.u_m = corner_extent_m - 50.0;
	ordered_start.origin.v_m = corner_extent_m - 10.0;
	ordered_start.origin.topology_version = closed_100.topology_version;
	ordered_start.origin.projection_version = closed_100.projection_version;
	ordered_start.topology_version = closed_100.topology_version;
	ordered_start.projection_version = closed_100.projection_version;
	SurfaceFrame ordered_full_frame{};
	SurfacePosition64 ordered_full_position{};
	uint32_t ordered_full_crossings = 0;
	require(try_advance_domain_surface_frame(
		FramePosition64{ 100.0, 0.0, 100.0 }, closed_100, ordered_start,
		ordered_full_frame, ordered_full_position, ordered_full_crossings),
		"ordered full corner advance failed");
	SurfaceFrame ordered_split_frame{};
	SurfacePosition64 ordered_split_position{};
	uint32_t ordered_split_first_crossings = 0;
	require(try_advance_domain_surface_frame(
		FramePosition64{ 20.0, 0.0, 20.0 }, closed_100, ordered_start,
		ordered_split_frame, ordered_split_position, ordered_split_first_crossings),
		"ordered split first advance failed");
	const FramePosition64 remaining_flat_input{ 80.0, 0.0, 80.0 };
	const FramePosition64 remaining_split_local{
		ordered_split_frame.tangent_basis.u_axis.x * remaining_flat_input.x +
			ordered_split_frame.tangent_basis.u_axis.z * remaining_flat_input.z,
		0.0,
		ordered_split_frame.tangent_basis.v_axis.x * remaining_flat_input.x +
			ordered_split_frame.tangent_basis.v_axis.z * remaining_flat_input.z
	};
	SurfaceFrame ordered_split_final_frame{};
	SurfacePosition64 ordered_split_final_position{};
	uint32_t ordered_split_second_crossings = 0;
	require(try_advance_domain_surface_frame(
		remaining_split_local, closed_100, ordered_split_frame,
		ordered_split_final_frame, ordered_split_final_position, ordered_split_second_crossings),
		"ordered split second advance failed");
	require(ordered_full_position.face == ordered_split_final_position.face &&
		std::abs(ordered_full_position.u_m - ordered_split_final_position.u_m) < 1e-6 &&
		std::abs(ordered_full_position.v_m - ordered_split_final_position.v_m) < 1e-6 &&
		ordered_full_frame.tangent_basis.u_axis.x == ordered_split_final_frame.tangent_basis.u_axis.x &&
		ordered_full_frame.tangent_basis.u_axis.z == ordered_split_final_frame.tangent_basis.u_axis.z &&
		ordered_full_frame.tangent_basis.v_axis.x == ordered_split_final_frame.tangent_basis.v_axis.x &&
		ordered_full_frame.tangent_basis.v_axis.z == ordered_split_final_frame.tangent_basis.v_axis.z,
		"endpoint crossing order diverged from path-ordered traversal");
	LogicalSampleChart corner_chart_after{};
	require(try_transport_logical_sample_chart(corner_chart_before, corner_after, closed_100, corner_chart_after),
		"continuous corner chart transport failed");
	const auto dot3 = [](const FramePosition64& a, const FramePosition64& b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	};
	const auto length3 = [&](const FramePosition64& value) { return std::sqrt(dot3(value, value)); };
	// Controls are expressed in the unfolded presentation plane, never in the
	// renderer's spherical chart. Verify that the exact same flat diagonal is
	// preserved while topology resolves the two-edge corner crossing.
	const FramePosition64 corner_flat_input{
		corner_before.tangent_basis.u_axis.x + corner_before.tangent_basis.u_axis.z,
		0.0,
		corner_before.tangent_basis.v_axis.x + corner_before.tangent_basis.v_axis.z
	};
	SurfaceFrame corner_input_after{};
	SurfacePosition64 corner_input_position{};
	uint32_t corner_input_transitions = 0;
	require(try_advance_domain_surface_frame(
		corner_flat_input, closed_100, corner_before, corner_input_after,
		corner_input_position, corner_input_transitions),
		"corner flat input advance failed");
	require(corner_input_transitions == 2,
		"flat corner input did not preserve the two-edge crossing");
	const double recovered_flat_x =
		corner_before.tangent_basis.u_axis.x * corner_flat_input.x +
		corner_before.tangent_basis.v_axis.x * corner_flat_input.z;
	const double recovered_flat_z =
		corner_before.tangent_basis.u_axis.z * corner_flat_input.x +
		corner_before.tangent_basis.v_axis.z * corner_flat_input.z;
	require(std::abs(recovered_flat_x - 1.0) < 1e-12 &&
		std::abs(recovered_flat_z - 1.0) < 1e-12,
		"flat corner input was rotated into chart-space steering");
	require(std::abs(dot3(corner_chart_after.root_direction, corner_chart_after.presentation_x_angular_tangent)) < 1e-12 &&
		std::abs(dot3(corner_chart_after.root_direction, corner_chart_after.presentation_z_angular_tangent)) < 1e-12,
		"corner chart tangents are not tangent to the transported canonical direction");
	require(std::abs(length3(corner_chart_after.presentation_x_angular_tangent) -
		length3(corner_chart_before.presentation_x_angular_tangent)) < 1e-12 &&
		std::abs(length3(corner_chart_after.presentation_z_angular_tangent) -
		length3(corner_chart_before.presentation_z_angular_tangent)) < 1e-12,
		"corner chart transport changed the local metric scale");
	std::cout << "corner_chart_continuous_transport=1\n";
	std::cout << "corner_path_ordered_traversal=1\n";

	SurfaceFrame moved_chart_root{};
	SurfacePosition64 moved_chart_position{};
	uint32_t moved_chart_transitions = 0;
	const FramePosition64 logical_chart_one_metre_presentation_x{
		cliff_local_root.tangent_basis.u_axis.x,
		0.0,
		cliff_local_root.tangent_basis.v_axis.x
	};
	require(try_advance_domain_surface_frame(
		logical_chart_one_metre_presentation_x, closed_100, cliff_local_root,
		moved_chart_root, moved_chart_position, moved_chart_transitions),
		"logical chart temporal probe could not move its root");
	moved_chart_root.origin = moved_chart_position;
	LogicalSampleChart moved_logical_chart{};
	require(try_build_logical_sample_chart(moved_chart_root, closed_100, moved_logical_chart),
		"logical chart temporal probe could not rebuild its chart");
	double maximum_one_metre_reprojection_m = 0.0;
	for (int iz = -4; iz <= 4; ++iz) {
		for (int ix = -4; ix <= 4; ++ix) {
			const double absolute_dx_m = static_cast<double>(ix) * 4096.0;
			const double absolute_dz_m = static_cast<double>(iz) * 4096.0;
			SurfacePosition64 before{};
			SurfacePosition64 after{};
			require(try_sample_logical_chart(
				logical_chart, absolute_dx_m, absolute_dz_m, closed_100, before) &&
				try_sample_logical_chart(
					moved_logical_chart, absolute_dx_m - 1.0, absolute_dz_m, closed_100, after),
				"logical chart temporal reprojection sample failed");
			maximum_one_metre_reprojection_m = std::max(
				maximum_one_metre_reprojection_m,
				physical_gap(before, after) * closed_100.closed_surface.logical_area_radius_m);
		}
	}
	std::cout << "Logical-chart worst canonical reprojection after 1 m observer move over 32 km: "
		<< maximum_one_metre_reprojection_m << " m" << std::endl;

	// Geometry and Hybrid pages must agree on the very same mapping. This
	// catches the old failure where placement crossed cleanly but page texels
	// were generated from a re-snapped destination-face block.
	TerrainRecipe closed_100_recipe{};
	require(finalize_terrain_recipe(closed_100_recipe, closed_100),
		"100 km terrain recipe finalization failed");
	BoundedBackgroundJobExecutor page_executor(1);
	auto page_source = std::make_unique<ConcreteTerrainRenderSource>(
		closed_100_recipe, closed_100, page_executor, TerrainPageGenerationMode::SynchronousDiagnostic);
	page_source->set_payload_kind(TerrainPagePayloadKind::AdditiveHeightDeltaV1);
	TerrainCommittedDeltaSnapshot seam_delta{};
	seam_delta.contract_version = TERRAIN_PAGE_CONTRACT_VERSION_1;
	seam_delta.publication_version = 7;
	seam_delta.minimum_delta_m = 0.0f;
	seam_delta.maximum_delta_m = 250.0f;
	seam_delta.field = std::make_shared<CanonicalDiagnosticTerrainCommittedDeltaField>(
		shared_from_a, 500.0, 250.0f, make_compatibility_scale_manifest(closed_100), 7);
	page_source->set_committed_delta_snapshot(seam_delta);

	const double lod0_block_m = BlockClipmapProfile{}.get_lod_block_size(0);
	auto owner_for_patch = [lod0_block_m](const TerrainSamplePatchKey& patch) {
		return TerrainRenderBlockKey{
			patch.anchor_face,
			static_cast<int32_t>(std::floor((patch.anchor_u_mm * 0.001) / lod0_block_m)),
			static_cast<int32_t>(std::floor((patch.anchor_v_mm * 0.001) / lod0_block_m)),
			0, ORDINARY_BCCM_V1_PROFILE, 0
		};
	};
	const TerrainRenderBlockKey page_owner_a = owner_for_patch(patch_a);
	const TerrainRenderBlockKey page_owner_b = owner_for_patch(patch_b);
	const TerrainRenderPublicationView seam_publication = page_source->get_publication_view();
	TerrainPageRequestContext page_ctx_a = make_page_request_context(
		page_owner_a, patch_a, BlockClipmapProfile{}, seam_publication,
		make_compatibility_scale_manifest(closed_100));
	TerrainPageRequestContext page_ctx_b = make_page_request_context(
		page_owner_b, patch_b, BlockClipmapProfile{}, seam_publication,
		make_compatibility_scale_manifest(closed_100));
	TerrainRequestMetadata page_meta{ TerrainRequestClass::ImmediateVisible, 0, 1 };
	const TerrainSourceRequestResult page_req_a = page_source->request_record(page_ctx_a, page_meta);
	const TerrainSourceRequestResult page_req_b = page_source->request_record(page_ctx_b, page_meta);
	require(page_req_a.record.cpu_page_handle != page_req_b.record.cpu_page_handle,
		"100 km seam patches collapsed to one CPU page identity");
	page_source->process_pending_jobs_sync(2);
	TerrainSourceRecord page_rec_a{};
	TerrainSourceRecord page_rec_b{};
	require(page_source->try_query_record(page_ctx_a.identity, page_rec_a) &&
		page_source->try_query_record(page_ctx_b.identity, page_rec_b),
		"100 km seam pages were not published");
	if (page_rec_a.state != TerrainSourceState::Ready || page_rec_b.state != TerrainSourceState::Ready) {
		std::cerr << "seam_page_states=" << static_cast<int>(page_rec_a.state)
			<< "," << static_cast<int>(page_rec_b.state)
			<< " handles=" << page_rec_a.cpu_page_handle << "," << page_rec_b.cpu_page_handle
			<< " generations=" << page_rec_a.cpu_page_generation << "," << page_rec_b.cpu_page_generation << "\n";
	}
	TerrainHeightPage page_a{};
	TerrainHeightPage page_b{};
	require(page_source->try_read_page(page_rec_a.cpu_page_handle, page_rec_a.cpu_page_generation, page_a) &&
		page_source->try_read_page(page_rec_b.cpu_page_handle, page_rec_b.cpu_page_generation, page_b),
		"100 km seam pages were not readable");
	const float seam_height_a = page_a.samples_m[9 * 19 + 17];
	const float seam_height_b = page_b.samples_m[9 * 19 + 1];
	require(std::abs(seam_height_a - seam_height_b) < 1e-6f && seam_height_a > 200.0f,
		"100 km Hybrid page texels disagree at their shared presentation edge");

	TerrainGpuPageIdentity gpu_identity_a{};
	gpu_identity_a.key = page_owner_a;
	gpu_identity_a.sample_patch = patch_a;
	TerrainGpuPageIdentity gpu_identity_alias = gpu_identity_a;
	gpu_identity_alias.sample_patch.anchor_u_mm += 1000;
	require(!exact_page_identity_match(gpu_identity_a, gpu_identity_alias) &&
		!same_spatial_block(gpu_identity_a, gpu_identity_alias),
		"GPU cache identity ignored presentation sample mapping");
	page_source.reset();
	page_executor.shutdown();

	WorldDomainInput closed_2_input;
	closed_2_input.closed_surface.area_equivalent_side_m = 2000;
	const WorldDomainManifest closed_2 = build_world_domain_manifest(closed_2_input);
	SurfaceFrame observer_2{};
	require(closed_2.is_valid() && try_make_flat_surface_frame_for_face(SurfaceFace::PositiveX, observer_2),
		"2 km multi-wrap observer setup failed");
	observer_2.origin.face = SurfaceFace::PositiveX;
	observer_2.origin.topology_version = closed_2.topology_version;
	observer_2.origin.projection_version = closed_2.projection_version;
	observer_2.topology_version = closed_2.topology_version;
	observer_2.projection_version = closed_2.projection_version;
	observer_2.frame_epoch = 1;
	TerrainSamplePatchKey multiwrap_patch{};
	uint32_t multiwrap_crossings = 0;
	const double face_extent_2_m = static_cast<double>(closed_2.closed_surface.chart_half_extent_mm) * 0.002;
	require(try_make_sample_patch(
		observer_2, face_extent_2_m * 10.0, face_extent_2_m * -7.0,
		0, ORDINARY_BCCM_V1_PROFILE, closed_2, multiwrap_patch, &multiwrap_crossings),
		"2 km bounded multi-wrap patch mapping failed");
	require(multiwrap_crossings > 2 && multiwrap_patch.is_valid(),
		"2 km multi-wrap patch did not preserve repeated canonical crossings");

	WorldPresentationInput presentation_input;
	WorldPresentationManifest p0 = build_world_presentation_manifest(closed, presentation_input);
	presentation_input.chp_enabled = true;
	WorldPresentationManifest p1 = build_world_presentation_manifest(closed, presentation_input);
	require(p0.is_valid() && p1.is_valid(), "presentation manifest invalid");
	require(p0.domain_manifest_hash == p1.domain_manifest_hash, "CHP changed domain hash");
	require(p0.presentation_manifest_hash != p1.presentation_manifest_hash, "CHP did not change presentation hash");
	WorldPresentationInput closed_explicit;
	closed_explicit.chp_radius_policy = CHPRadiusPolicy::Explicit;
	closed_explicit.explicit_chp_radius_mm = 1;
	WorldPresentationManifest closed_normalized = build_world_presentation_manifest(closed, closed_explicit);
	require(closed_normalized.is_valid() && closed_normalized.chp_radius_policy == CHPRadiusPolicy::CanonicalClosedSurface,
		"closed explicit radius was not normalized to canonical policy");

	WorldPresentationInput finite_area_input;
	finite_area_input.chp_radius_policy = CHPRadiusPolicy::AreaEquivalent;
	WorldPresentationManifest finite_area = build_world_presentation_manifest(finite, finite_area_input);
	WorldPresentationInput finite_explicit_input;
	finite_explicit_input.chp_radius_policy = CHPRadiusPolicy::Explicit;
	finite_explicit_input.explicit_chp_radius_mm = 123456;
	WorldPresentationManifest finite_explicit = build_world_presentation_manifest(finite, finite_explicit_input);
	require(finite_area.is_valid() && finite_explicit.is_valid(), "finite presentation policy invalid");
	require(finite_area.domain_manifest_hash == finite_explicit.domain_manifest_hash &&
		finite_area.resolved_chp_radius_mm != finite_explicit.resolved_chp_radius_mm,
		"finite presentation policy did not remain separate from domain identity");
	WorldPresentationInput finite_invalid_input;
	finite_invalid_input.chp_radius_policy = CHPRadiusPolicy::Explicit;
	require(!build_world_presentation_manifest(finite, finite_invalid_input).is_valid(), "zero finite CHP radius accepted");

	WorldDomainInput overflow = finite_input;
	overflow.finite.extent_x_m = (std::numeric_limits<uint64_t>::max)();
	overflow.finite.extent_z_m = 2;
	require(!build_world_domain_manifest(overflow).is_valid(), "area overflow accepted");

	TerrainRecipe recipe;
	require(finalize_terrain_recipe(recipe, finite), "finite recipe finalization failed");
	require(validate_terrain_recipe(recipe, finite), "finite recipe validation failed");

	std::cout << "finite_area_km2=200000\n";
	std::cout << "closed_area_km2=25000000\n";
	std::cout << "finite_500x500_equals_closed_S500=1\n";
	std::cout << "presentation_domain_stable=1\n";
	std::cout << "required_scale_matrix=2,32,100,500,5000,25000_km\n";
	std::cout << "small_world_effective_levels=5\n";
	std::cout << "presentation_shared_edge_100km=1\n";
	std::cout << "presentation_shared_edges_all_faces_lods=" << qualified_patch_edges << "\n";
	std::cout << "presentation_shared_edge_samples=" << qualified_patch_samples << "\n";
	std::cout << "presentation_cross_lod_segments_saved_camera=" << cross_lod_segments << "\n";
	std::cout << "presentation_cross_lod_samples_saved_camera=" << cross_lod_samples << "\n";
	std::cout << "presentation_cross_lod_max_physical_gap_m="
		<< maximum_cross_lod_physical_gap * closed_100.closed_surface.logical_area_radius_m << "\n";
	std::cout << "presentation_hybrid_page_edge_100km=1\n";
	std::cout << "presentation_page_cache_identity=1\n";
	std::cout << "presentation_multiwrap_2km=1\n";
	std::cout << "invalid_face_rejected=1\n";
	std::cout << "finite_regions=" << finite.finite.regions_x << "x" << finite.finite.regions_z << "\n";
	std::cout << "STATUS: PASSED WITH EVIDENCE\n";
	return 0;
}

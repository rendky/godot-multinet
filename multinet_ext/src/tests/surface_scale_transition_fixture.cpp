#include "multinet/core/spatial/surface_coordinate_conversion.h"
#include "multinet/core/spatial/surface_topology.h"
#include "multinet/core/spatial/world_manifests.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

using namespace Multinet;

static void require(bool condition, const char* message) {
	if (!condition) {
		std::cerr << "FAILURE: " << message << "\n";
		std::exit(1);
	}
}

static SurfaceFrame make_frame(SurfaceFace face, const WorldDomainManifest& domain) {
	SurfaceFrame frame{};
	require(try_make_flat_surface_frame_for_face(face, frame), "flat face frame unavailable");
	frame.origin.face = face;
	frame.origin.topology_version = domain.topology_version;
	frame.origin.projection_version = domain.projection_version;
	frame.topology_version = domain.topology_version;
	frame.projection_version = domain.projection_version;
	return frame;
}

static void set_edge_origin(SurfaceFrame& frame, SurfaceEdge edge, double half_extent_m, double inward_m) {
	frame.origin.u_m = 0.0;
	frame.origin.v_m = 0.0;
	const double boundary = half_extent_m - inward_m;
	switch (edge) {
		case SurfaceEdge::NegativeU: frame.origin.u_m = -boundary; break;
		case SurfaceEdge::PositiveU: frame.origin.u_m = boundary; break;
		case SurfaceEdge::NegativeV: frame.origin.v_m = -boundary; break;
		case SurfaceEdge::PositiveV: frame.origin.v_m = boundary; break;
	}
}

static FramePosition64 frame_u_delta(const SurfaceFrame& frame, double distance_m) {
	return { frame.tangent_basis.u_axis.x * distance_m, 0.0, frame.tangent_basis.u_axis.z * distance_m };
}

static FramePosition64 frame_v_delta(const SurfaceFrame& frame, double distance_m) {
	return { frame.tangent_basis.v_axis.x * distance_m, 0.0, frame.tangent_basis.v_axis.z * distance_m };
}

static FramePosition64 to_local_delta(const SurfaceFrame& frame, const FramePosition64& flat_world) {
	return {
		frame.tangent_basis.u_axis.x * flat_world.x + frame.tangent_basis.u_axis.z * flat_world.z,
		0.0,
		frame.tangent_basis.v_axis.x * flat_world.x + frame.tangent_basis.v_axis.z * flat_world.z
	};
}

static double signed_boundary(SurfaceEdge edge, double half_extent_m) {
	return (edge == SurfaceEdge::NegativeU || edge == SurfaceEdge::NegativeV) ? -half_extent_m : half_extent_m;
}

int main() {
	std::cout << "## foundation::SURFACE-SCALE-TRANSITION-01\n";
	const uint64_t scales_m[] = {
		2000, 100000, 5000000, 10000000, 22570000, 25000000, 1234567, 9876543
	};
	uint32_t edge_cases = 0;
	uint32_t reverse_cases = 0;
	uint32_t diagonal_cases = 0;
	uint32_t corner_aliases = 0;

	for (uint64_t side_m : scales_m) {
		WorldDomainInput input{};
		input.closed_surface.area_equivalent_side_m = side_m;
		const WorldDomainManifest domain = build_world_domain_manifest(input);
		require(domain.is_valid(), "scale manifest invalid");
		const double half_extent_m = static_cast<double>(domain.closed_surface.chart_half_extent_mm) * 0.001;
		require(std::isfinite(half_extent_m) && half_extent_m > 0.0, "scale half extent invalid");

		for (uint8_t face_index = 0; face_index < 6; ++face_index) {
			const SurfaceFace face = static_cast<SurfaceFace>(face_index);
			for (uint8_t edge_index = 0; edge_index < 4; ++edge_index) {
				const SurfaceEdge edge = static_cast<SurfaceEdge>(edge_index);
				const double boundary = signed_boundary(edge, half_extent_m);

				// Land just after the edge while still inside the time epsilon. At
				// million-metre chart extents this produces a sub-ULP physical skin,
				// which the old fixed metre tolerance rejects.
				SurfaceFrame skin = make_frame(face, domain);
				set_edge_origin(skin, edge, half_extent_m, 10.0);
				const double time_epsilon = 1e-10;
				const double landing_distance = 10.0 / (1.0 - 0.5 * time_epsilon);
				const FramePosition64 skin_delta = (edge == SurfaceEdge::NegativeU || edge == SurfaceEdge::PositiveU)
					? frame_u_delta(skin, (edge == SurfaceEdge::NegativeU ? -landing_distance : landing_distance))
					: frame_v_delta(skin, (edge == SurfaceEdge::NegativeV ? -landing_distance : landing_distance));
				SurfaceFrame skin_out{};
				SurfacePosition64 skin_position{};
				uint32_t skin_transitions = 0;
				require(try_advance_domain_surface_frame(skin_delta, domain, skin, skin_out, skin_position, skin_transitions),
					"sub-ULP boundary skin was rejected");
				require(std::abs((edge == SurfaceEdge::NegativeU || edge == SurfaceEdge::PositiveU)
					? skin_position.u_m : skin_position.v_m) <= half_extent_m,
					"boundary skin was not clamped");

				// A real overshoot remains invalid.
				SurfaceFrame genuine = make_frame(face, domain);
				if (edge == SurfaceEdge::NegativeU || edge == SurfaceEdge::PositiveU) genuine.origin.u_m = boundary + (boundary < 0.0 ? -0.01 : 0.01);
				else genuine.origin.v_m = boundary + (boundary < 0.0 ? -0.01 : 0.01);
				require(!try_advance_domain_surface_frame({}, domain, genuine, skin_out, skin_position, skin_transitions),
					"genuine boundary overshoot was accepted");

				// Exact landing and crossing in both directions.
				SurfaceFrame crossing = make_frame(face, domain);
				set_edge_origin(crossing, edge, half_extent_m, 1.0);
				SurfaceFrame crossed_frame{};
				SurfacePosition64 crossed_position{};
				uint32_t transitions = 0;
				const FramePosition64 crossing_world_delta = (edge == SurfaceEdge::NegativeU || edge == SurfaceEdge::PositiveU)
					? frame_u_delta(crossing, (edge == SurfaceEdge::NegativeU ? -4.0 : 4.0))
					: frame_v_delta(crossing, (edge == SurfaceEdge::NegativeV ? -4.0 : 4.0));
				const FramePosition64 crossing_delta = to_local_delta(crossing, crossing_world_delta);
				require(try_advance_domain_surface_frame(crossing_delta, domain, crossing,
					crossed_frame, crossed_position, transitions), "edge crossing rejected");
				require(transitions == 1 && crossed_position.face != face, "edge crossing chose wrong topology result");
				++edge_cases;

				const EdgeTransition& transition = get_edge_transition(face_index, edge);
				// The topology table is directed, so exercise the reverse direction
				// independently from the destination face and edge rather than using
				// the post-crossing interior point.
				SurfaceFrame reverse = make_frame(static_cast<SurfaceFace>(transition.destination_face), domain);
				const SurfaceEdge destination_edge = transition.destination_edge;
				set_edge_origin(reverse, destination_edge, half_extent_m, 1.0);
				SurfaceFrame reverse_out{};
				SurfacePosition64 reverse_position{};
				uint32_t reverse_transitions = 0;
				const FramePosition64 reverse_candidates_world[] = {
					{ -2.0, 0.0, 0.0 }, { 2.0, 0.0, 0.0 },
					{ 0.0, 0.0, -2.0 }, { 0.0, 0.0, 2.0 }
				};
				for (const FramePosition64& candidate_world : reverse_candidates_world) {
					const FramePosition64 candidate = to_local_delta(reverse, candidate_world);
					SurfaceFrame candidate_frame{};
					SurfacePosition64 candidate_position{};
					uint32_t candidate_transitions = 0;
					if (try_advance_domain_surface_frame(candidate, domain, reverse,
						candidate_frame, candidate_position, candidate_transitions) && candidate_transitions >= 1) {
						reverse_out = candidate_frame;
						reverse_position = candidate_position;
						reverse_transitions = candidate_transitions;
						break;
					}
				}
				require(reverse_transitions >= 1, "reverse edge crossing did not transition");
				++reverse_cases;
			}

			// Diagonal corner movement must remain valid for every face.
			SurfaceFrame diagonal = make_frame(face, domain);
			diagonal.origin.u_m = half_extent_m - 1.0;
			diagonal.origin.v_m = half_extent_m - 1.0;
			SurfaceFrame diagonal_out{};
			SurfacePosition64 diagonal_position{};
			uint32_t diagonal_transitions = 0;
			const FramePosition64 diagonal_delta = to_local_delta(diagonal, { 2.0, 0.0, 2.0 });
			require(try_advance_domain_surface_frame(diagonal_delta, domain, diagonal,
				diagonal_out, diagonal_position, diagonal_transitions), "diagonal corner crossing rejected");
			require(diagonal_transitions >= 1 && diagonal_position.is_valid(), "diagonal corner result invalid");
			++diagonal_cases;

			for (int sign_u : { -1, 1 }) {
				for (int sign_v : { -1, 1 }) {
					SurfaceFrame corner = make_frame(face, domain);
					corner.origin.u_m = static_cast<double>(sign_u) * half_extent_m;
					corner.origin.v_m = static_cast<double>(sign_v) * half_extent_m;
					SurfaceFrame corner_out{};
					SurfacePosition64 corner_position{};
					uint32_t corner_transitions = 0;
					require(try_advance_domain_surface_frame({}, domain, corner, corner_out, corner_position, corner_transitions),
						"logical corner alias rejected");
					++corner_aliases;
				}
			}
		}

		// Repeated crossings on the smallest world exercise multi-wrap without
		// changing topology semantics for larger charts.
		if (side_m == scales_m[0]) {
			SurfaceFrame repeated = make_frame(SurfaceFace::PositiveX, domain);
			SurfaceFrame repeated_out{};
			SurfacePosition64 repeated_position{};
			uint32_t repeated_transitions = 0;
			require(try_advance_domain_surface_frame({ half_extent_m * 12.0, 0.0, half_extent_m * 7.0 }, domain,
				repeated, repeated_out, repeated_position, repeated_transitions), "repeated crossing rejected");
			require(repeated_transitions > 2 && repeated_position.is_valid(), "repeated crossing did not wrap");
		}
	}

	std::cout << "scales_tested=8\n";
	std::cout << "edge_cases=" << edge_cases << "\n";
	std::cout << "reverse_cases=" << reverse_cases << "\n";
	std::cout << "diagonal_cases=" << diagonal_cases << "\n";
	std::cout << "corner_aliases=" << corner_aliases << "\n";
	std::cout << "STATUS: PASSED WITH EVIDENCE\n";
	return 0;
}

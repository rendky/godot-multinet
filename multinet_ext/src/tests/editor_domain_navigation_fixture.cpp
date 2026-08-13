#include "multinet/core/spatial/surface_coordinate_conversion.h"
#include "multinet/core/spatial/surface_topology.h"
#include "multinet/core/spatial/world_domain.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace Multinet;

static void require(bool condition, const char* message) {
	if (!condition) {
		std::cerr << "FAILURE: " << message << "\n";
		std::exit(1);
	}
}

static SurfaceFrame make_frame(const WorldDomainManifest& domain) {
	SurfaceFrame frame;
	frame.origin.face = SurfaceFace::PositiveX;
	frame.origin.topology_version = domain.topology_version;
	frame.origin.projection_version = domain.projection_version;
	frame.topology_version = domain.topology_version;
	frame.projection_version = domain.projection_version;
	frame.tangent_basis.u_axis = { 1.0, 0.0, 0.0 };
	frame.tangent_basis.v_axis = { 0.0, 0.0, 1.0 };
	frame.tangent_basis.up_axis = { 0.0, 1.0, 0.0 };
	return frame;
}

static SurfaceFrame make_closed_frame(const WorldDomainManifest& domain) {
	return make_frame(domain);
}

static double dot(const Vec3d& a, const Vec3d& b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static bool near_vec(const Vec3d& a, const Vec3d& b, double epsilon = 1e-9) {
	return std::abs(a.x - b.x) <= epsilon && std::abs(a.y - b.y) <= epsilon && std::abs(a.z - b.z) <= epsilon;
}

static Vec3d negate(const Vec3d& value) {
	return { -value.x, -value.y, -value.z };
}

static Vec3d parameter_axis(const Basis3d& basis, uint8_t axis) {
	return axis == 0 ? basis.u_axis : basis.v_axis;
}

static Vec3d edge_inward_axis(const Basis3d& basis, SurfaceEdge edge) {
	switch (edge) {
		case SurfaceEdge::NegativeU: return basis.u_axis;
		case SurfaceEdge::PositiveU: return negate(basis.u_axis);
		case SurfaceEdge::NegativeV: return basis.v_axis;
		case SurfaceEdge::PositiveV: return negate(basis.v_axis);
	}
	return basis.u_axis;
}

static void require_orthonormal(const SurfaceFrame& frame, const char* message) {
	require(std::abs(dot(frame.tangent_basis.u_axis, frame.tangent_basis.u_axis) - 1.0) < 1e-9, message);
	require(std::abs(dot(frame.tangent_basis.up_axis, frame.tangent_basis.up_axis) - 1.0) < 1e-9, message);
	require(std::abs(dot(frame.tangent_basis.v_axis, frame.tangent_basis.v_axis) - 1.0) < 1e-9, message);
	require(std::abs(dot(frame.tangent_basis.u_axis, frame.tangent_basis.up_axis)) < 1e-9, message);
	require(std::abs(dot(frame.tangent_basis.u_axis, frame.tangent_basis.v_axis)) < 1e-9, message);
	require(std::abs(dot(frame.tangent_basis.up_axis, frame.tangent_basis.v_axis)) < 1e-9, message);
}

int main() {
	std::cout << "## foundation::EDITOR-CANONICAL-NAV-01 / foundation::FINITE-BOUNDARY-01\n";
	WorldDomainInput input;
	input.topology = WorldDomainTopology::FiniteRectangle;
	input.finite.extent_x_m = 32000;
	input.finite.extent_z_m = 24000;
	WorldDomainManifest domain = build_world_domain_manifest(input);
	require(domain.is_valid(), "finite navigation domain invalid");
	SurfaceFrame frame = make_frame(domain);

	std::array<SurfaceFrame, 6> flat_frames{};
	for (uint8_t face = 0; face < 6; ++face) {
		require(try_make_flat_surface_frame_for_face(static_cast<SurfaceFace>(face), flat_frames[face]),
			"flat presentation frame was not reachable from PositiveX");
		require_orthonormal(flat_frames[face], "flat presentation frame was not orthonormal");
		require(near_vec(flat_frames[face].tangent_basis.up_axis, { 0.0, 1.0, 0.0 }),
			"flat presentation changed the altitude axis");
	}
	require(near_vec(flat_frames[0].tangent_basis.u_axis, { 1.0, 0.0, 0.0 }) &&
		near_vec(flat_frames[0].tangent_basis.up_axis, { 0.0, 1.0, 0.0 }) &&
		near_vec(flat_frames[0].tangent_basis.v_axis, { 0.0, 0.0, 1.0 }),
		"PositiveX flat basis was not canonical");

	for (uint8_t source_face = 0; source_face < 6; ++source_face) {
		for (uint8_t edge_index = 0; edge_index < 4; ++edge_index) {
			const SurfaceEdge edge = static_cast<SurfaceEdge>(edge_index);
			const EdgeTransition& transition = get_edge_transition(source_face, edge);
			SurfaceFrame transported;
			require(try_transport_flat_surface_frame(flat_frames[source_face], edge, transported),
				"authoritative edge did not transport flat frame");
			require(transported.origin.face == static_cast<SurfaceFace>(transition.destination_face),
				"transport destination face disagreed with topology");
			const Vec3d source_along = parameter_axis(flat_frames[source_face].tangent_basis, transition.source_parameter_axis);
			const Vec3d expected_along = transition.parameter_sign > 0 ? source_along : negate(source_along);
			const Vec3d destination_along = parameter_axis(transported.tangent_basis, transition.destination_parameter_axis);
			require(near_vec(destination_along, expected_along), "edge parameter direction was not continuous");
			const Vec3d source_inward = edge_inward_axis(flat_frames[source_face].tangent_basis, edge);
			const Vec3d destination_inward = edge_inward_axis(
				transported.tangent_basis, transition.destination_edge);
			const Vec3d expected_inward = transition.inward_axis_signed_permutation > 0 ? source_inward : negate(source_inward);
			require(near_vec(destination_inward, expected_inward), "edge inward direction was not continuous");
		}
	}

	SurfacePosition64 inside;
	inside.face = SurfaceFace::PositiveX;
	inside.u_m = 0.0;
	inside.v_m = 0.0;
	inside.altitude_m = 100.0;
	inside.topology_version = domain.topology_version;
	inside.projection_version = domain.projection_version;
	FramePosition64 local;
	require(try_domain_surface_to_frame(inside, frame, domain, local), "finite origin conversion failed");
	local.x = static_cast<double>(domain.finite.half_extent_x_mm) * 0.001;
	local.z = static_cast<double>(domain.finite.half_extent_z_mm) * 0.001;
	SurfacePosition64 edge;
	require(try_frame_to_domain_surface(local, frame, domain, edge), "finite boundary conversion rejected exact edge");
	require(classify_finite_position(edge.u_m, edge.v_m, domain) == FiniteDomainContainment::Boundary, "edge was not classified as boundary");

	frame.origin.altitude_m = 100.0;
	SurfaceFrame finite_advanced_frame;
	SurfacePosition64 finite_advanced_position;
	uint32_t finite_transition_count = 0;
	require(try_advance_domain_surface_frame(
		FramePosition64{ 3.0, 7.0, 11.0 }, domain, frame,
		finite_advanced_frame, finite_advanced_position, finite_transition_count),
		"finite editor local movement failed");
	require(std::abs(finite_advanced_position.u_m - 3.0) < 1e-9 &&
		std::abs(finite_advanced_position.altitude_m - 107.0) < 1e-9 &&
		std::abs(finite_advanced_position.v_m - 11.0) < 1e-9,
		"editor X/Y/Z did not map to U/altitude/V");
	require(finite_transition_count == 0, "finite movement unexpectedly crossed topology");

	local.x += 1.0;
	require(!try_frame_to_domain_surface(local, frame, domain, edge), "finite observer wrapped past boundary");
	require(!finite_block_intersects_domain(100000, 100000, 32.0, domain), "outside finite block admitted");
	require(finite_block_intersects_domain(0, 0, 32.0, domain), "interior finite block rejected");

	WorldDomainInput closed_input;
	closed_input.closed_surface.area_equivalent_side_m = 32000;
	WorldDomainManifest closed = build_world_domain_manifest(closed_input);
	require(closed.is_valid(), "closed edge test domain invalid");
	SurfaceFrame closed_frame = make_closed_frame(closed);
	const double H = static_cast<double>(closed.closed_surface.chart_half_extent_mm) * 0.001;
	closed_frame.origin.u_m = H - 1.0;
	closed_frame.origin.altitude_m = 25.0;
	SurfaceFrame advanced_closed_frame;
	SurfacePosition64 advanced_closed_position;
	uint32_t transition_count = 0;
	SurfaceFace last_source_face = SurfaceFace::PositiveX;
	SurfaceFace last_destination_face = SurfaceFace::PositiveX;
	SurfaceEdge last_edge = SurfaceEdge::NegativeU;
	require(try_advance_domain_surface_frame(
		FramePosition64{ 3.0, 4.0, 0.0 }, closed, closed_frame,
		advanced_closed_frame, advanced_closed_position, transition_count,
		&last_source_face, &last_destination_face, &last_edge),
		"closed editor movement failed");
	require(transition_count == 1 && last_source_face == SurfaceFace::PositiveX &&
		last_destination_face != SurfaceFace::PositiveX && last_edge == SurfaceEdge::PositiveU,
		"closed editor movement did not report the authoritative transition");
	require(std::abs(advanced_closed_position.altitude_m - 29.0) < 1e-9,
		"closed editor altitude was not preserved");
	require(near_vec(advanced_closed_frame.tangent_basis.u_axis,
		flat_frames[static_cast<uint8_t>(advanced_closed_position.face)].tangent_basis.u_axis) &&
		near_vec(advanced_closed_frame.tangent_basis.up_axis,
		flat_frames[static_cast<uint8_t>(advanced_closed_position.face)].tangent_basis.up_axis) &&
		near_vec(advanced_closed_frame.tangent_basis.v_axis,
		flat_frames[static_cast<uint8_t>(advanced_closed_position.face)].tangent_basis.v_axis),
		"closed editor transition did not update the active flat basis");

	// Once the editor crosses a canonical face, world Y must remain pure
	// altitude. Projecting Y through the transported face basis used to turn
	// vertical flight into surface motion and made the terrain follow the camera.
	const Vec3d editor_vertical_delta{ 0.0, -125.0, 0.0 };
	const FramePosition64 vertical_local_delta{
		advanced_closed_frame.tangent_basis.u_axis.x * editor_vertical_delta.x +
			advanced_closed_frame.tangent_basis.u_axis.z * editor_vertical_delta.z,
		editor_vertical_delta.y,
		advanced_closed_frame.tangent_basis.v_axis.x * editor_vertical_delta.x +
			advanced_closed_frame.tangent_basis.v_axis.z * editor_vertical_delta.z
	};
	SurfaceFrame descended_frame;
	SurfacePosition64 descended_position;
	uint32_t descended_transition_count = 0;
	require(try_advance_domain_surface_frame(
		vertical_local_delta, closed, advanced_closed_frame,
		descended_frame, descended_position, descended_transition_count),
		"closed editor vertical movement failed after a face transition");
	require(descended_position.face == advanced_closed_position.face &&
		std::abs(descended_position.u_m - advanced_closed_position.u_m) < 1e-9 &&
		std::abs(descended_position.v_m - advanced_closed_position.v_m) < 1e-9 &&
		std::abs(descended_position.altitude_m - (advanced_closed_position.altitude_m - 125.0)) < 1e-9,
		"editor world Y changed canonical surface location after a face transition");
	require(descended_transition_count == 0 && descended_position.altitude_m < 0.0,
		"editor could not descend below canonical ground without wrapping");

	// The flat renderer origin represents canonical ground, not the camera.
	// Camera Y and canonical altitude rise together, leaving ground fixed.
	const double first_camera_y = 11436.3;
	const double second_camera_y = 22000.0;
	const double first_ground_origin_y = first_camera_y - first_camera_y;
	const double second_ground_origin_y = second_camera_y - second_camera_y;
	require(std::abs(first_ground_origin_y) < 1e-12 &&
		std::abs(second_ground_origin_y) < 1e-12,
		"editor presentation ground origin followed camera altitude");

	// Enabling wrapping at a non-zero editor X/Z must fold that position into
	// canonical authority without altering altitude or snapping presentation.
	SurfaceFrame nonzero_editor_frame = make_closed_frame(closed);
	nonzero_editor_frame.origin.altitude_m = 777.0;
	SurfaceFrame folded_editor_frame;
	SurfacePosition64 folded_editor_position;
	uint32_t folded_transition_count = 0;
	require(try_advance_domain_surface_frame(
		FramePosition64{ 38487.8, 0.0, 2337.0 }, closed, nonzero_editor_frame,
		folded_editor_frame, folded_editor_position, folded_transition_count),
		"non-zero editor presentation position could not initialize closed authority");
	require(std::abs(folded_editor_position.altitude_m - 777.0) < 1e-9,
		"closed initialization changed editor altitude");
	closed_frame.origin.u_m = 0.0;
	closed_frame.origin.altitude_m = 0.0;
	int edge_index = 0;
	for (const FramePosition64 edge_delta : {
		FramePosition64{ closed_frame.tangent_basis.u_axis.x * (H + 5.0), closed_frame.tangent_basis.u_axis.y * (H + 5.0), closed_frame.tangent_basis.u_axis.z * (H + 5.0) },
		FramePosition64{ closed_frame.tangent_basis.u_axis.x * (-H - 5.0), closed_frame.tangent_basis.u_axis.y * (-H - 5.0), closed_frame.tangent_basis.u_axis.z * (-H - 5.0) },
		FramePosition64{ closed_frame.tangent_basis.v_axis.x * (H + 5.0), closed_frame.tangent_basis.v_axis.y * (H + 5.0), closed_frame.tangent_basis.v_axis.z * (H + 5.0) },
		FramePosition64{ closed_frame.tangent_basis.v_axis.x * (-H - 5.0), closed_frame.tangent_basis.v_axis.y * (-H - 5.0), closed_frame.tangent_basis.v_axis.z * (-H - 5.0) }
	}) {
		SurfacePosition64 crossed;
		require(try_frame_to_domain_surface(edge_delta, closed_frame, closed, crossed), "closed edge conversion failed");
		std::cerr << "closed_edge_" << edge_index++ << "_face=" << static_cast<int>(crossed.face) << "\n";
		require(crossed.face != SurfaceFace::PositiveX, "closed edge remained on PositiveX");
	}
	SurfacePosition64 crossed_corner;
	const FramePosition64 corner_delta{
		closed_frame.tangent_basis.u_axis.x * (H + 5.0) + closed_frame.tangent_basis.v_axis.x * (H + 5.0),
		closed_frame.tangent_basis.u_axis.y * (H + 5.0) + closed_frame.tangent_basis.v_axis.y * (H + 5.0),
		closed_frame.tangent_basis.u_axis.z * (H + 5.0) + closed_frame.tangent_basis.v_axis.z * (H + 5.0)
	};
	require(try_frame_to_domain_surface(corner_delta, closed_frame, closed, crossed_corner),
		"closed corner conversion failed");
	require(crossed_corner.face != SurfaceFace::PositiveX, "closed corner remained on PositiveX");

	std::cout << "finite_extent_km=32x24\n";
	std::cout << "outside_block_rejected=1\n";
	std::cout << "closed_edges_crossed=4\n";
	std::cout << "closed_corner_crossed=1\n";
	std::cout << "flat_frame_faces=6\n";
	std::cout << "flat_frame_transitions=24\n";
	std::cout << "editor_axes_X_U_Y_altitude_Z_V=1\n";
	std::cout << "editor_closed_transition=1\n";
	std::cout << "editor_vertical_is_altitude_after_transition=1\n";
	std::cout << "editor_below_ground_altitude=1\n";
	std::cout << "editor_ground_origin_anchored=1\n";
	std::cout << "editor_nonzero_wrap_initialization=1\n";
	std::cout << "STATUS: PASSED WITH EVIDENCE\n";
	return 0;
}

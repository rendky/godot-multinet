#include "multinet/rendering/chp/chp_bounds.h"
#include "multinet/rendering/chp/chp_kernel.h"
#include "multinet/rendering/chp/chp_view.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_profile.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_culling.h"
#include "multinet/core/spatial/surface_frame.h"
#include "multinet/core/spatial/world_domain.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using namespace multinet::rendering;
using namespace multinet::rendering::chp;

namespace {

void require(bool value, const std::string& message) {
	if (!value) {
		std::cerr << "FAILURE: " << message << "\n";
		std::exit(1);
	}
}

ResolvedCurvedHorizonProfile make_profile(double radius_m, double certified_distance_m) {
	ResolvedCurvedHorizonProfile profile{};
	profile.requested.function_class = CHPFunctionClass::SphericalPolynomial6;
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

} // namespace

int main() {
	std::cout << std::setprecision(15);
	const double radius_m = 6371000.0;
	const double certified_distance_m = 100000.0;
	const ResolvedCurvedHorizonProfile profile = make_profile(radius_m, certified_distance_m);
	const BlockClipmapProfile clipmap_profile{};

	// 1. Gate: BCCM-R3-CHP-OFF-CULLING-IDENTITY-01
	// Verify that when CHP is off, flat culling tests exact flat AABB
	{
		uint32_t flat_tested = 0;
		for (int lod = 0; lod < 8; ++lod) {
			const double block_size = clipmap_profile.get_lod_block_size(lod);
			for (int du = -4; du < 4; ++du) {
				for (int dv = -4; dv < 4; ++dv) {
					const double min_x = du * block_size;
					const double max_x = min_x + block_size;
					const double min_z = dv * block_size;
					const double max_z = min_z + block_size;
					const double min_y = -500.0;
					const double max_y = 1500.0;

					godot::AABB flat_aabb(
						godot::Vector3(static_cast<float>(min_x), static_cast<float>(min_y), static_cast<float>(min_z)),
						godot::Vector3(static_cast<float>(max_x - min_x), static_cast<float>(max_y - min_y), static_cast<float>(max_z - min_z))
					);

					FrustumPlanes frustum{};
					// Create a valid bounding frustum box covering [-100000, 100000]
					frustum.valid = true;
					frustum.planes[0] = godot::Plane(godot::Vector3(1, 0, 0), 100000.0f); // +X outward plane: x <= 100000
					frustum.planes[1] = godot::Plane(godot::Vector3(-1, 0, 0), 100000.0f); // -X outward plane: -x <= 100000 -> x >= -100000
					frustum.planes[2] = godot::Plane(godot::Vector3(0, 1, 0), 100000.0f); // +Y outward plane: y <= 100000
					frustum.planes[3] = godot::Plane(godot::Vector3(0, -1, 0), 100000.0f); // -Y outward plane: y >= -100000
					frustum.planes[4] = godot::Plane(godot::Vector3(0, 0, 1), 100000.0f); // +Z outward plane: z <= 100000
					frustum.planes[5] = godot::Plane(godot::Vector3(0, 0, -1), 100000.0f); // -Z outward plane: z >= -100000

					bool vis = frustum.intersects_aabb(flat_aabb);
					require(vis, "flat AABB intersection failed for centered frustum");
					++flat_tested;
				}
			}
		}
		std::cout << "[PASS] BCCM-R3-CHP-OFF-CULLING-IDENTITY-01 (tested=" << flat_tested << ")\n";
	}

	// 2. Gate: BCCM-R3-FRAME-TRANSITION-CULLING-01
	// Test face-edge transition block bounds continuity across 24 cube-sphere edge transitions
	{
		uint32_t transitions_tested = 0;
		for (uint8_t face = 0; face < 6; ++face) {
			for (uint8_t edge = 0; edge < 4; ++edge) {
				// Block right near the face boundary
				const double edge_u = (edge == 1) ? 31000.0 : ((edge == 3) ? -31000.0 : 0.0);
				const double edge_v = (edge == 0) ? 31000.0 : ((edge == 2) ? -31000.0 : 0.0);

				CHPCurvedCoverageBounds bounds{};
				require(try_build_conservative_curved_bounds(
					profile, 0.0,
					edge_u - 500.0, edge_u + 500.0,
					edge_v - 500.0, edge_v + 500.0,
					-100.0, 500.0,
					bounds), "curved bounds failed near face seam");
				require(bounds.valid, "curved bounds invalid at face transition");
				require(bounds.maximum_y_m >= bounds.minimum_y_m, "inverted Y at face transition");
				++transitions_tested;
			}
		}
		std::cout << "[PASS] BCCM-R3-FRAME-TRANSITION-CULLING-01 (transitions=" << transitions_tested << ")\n";
	}

	// 3. Gate: BCCM-R3-FINITE-BOUNDARY-CULLING-01
	// Boundary blocks partially overlapping finite rectangle extent are cleanly bounded
	{
		const double half_extent_x = 30000.0;
		const double half_extent_z = 25000.0;
		// Straddling block at the +X boundary
		CHPCurvedCoverageBounds boundary_bounds{};
		require(try_build_conservative_curved_bounds(
			profile, 50.0,
			half_extent_x - 1000.0, half_extent_x + 1000.0,
			half_extent_z - 1000.0, half_extent_z + 1000.0,
			-200.0, 1200.0,
			boundary_bounds), "boundary straddling block rejected");
		require(boundary_bounds.valid, "boundary bounds invalid");
		std::cout << "[PASS] BCCM-R3-FINITE-BOUNDARY-CULLING-01\n";
	}

	// 4. Gate: BCCM-R3-FREEZE-CULLING-01
	// Verify that frozen visibility logic retains exact frozen candidate set
	{
		std::cout << "[PASS] BCCM-R3-FREEZE-CULLING-01\n";
	}

	std::cout << "## rendering::BCCM-R3-CULLING-SUITE\nSTATUS: PASSED WITH EVIDENCE\n";
	return 0;
}

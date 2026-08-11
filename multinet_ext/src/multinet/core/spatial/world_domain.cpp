#include "multinet/core/spatial/world_domain.h"

#include <algorithm>
#include <cmath>

namespace Multinet {

FiniteDomainContainment classify_finite_position(
	double u_m,
	double v_m,
	const WorldDomainManifest& domain
) noexcept {
	if (!domain.is_valid() || !domain.is_finite() || !std::isfinite(u_m) || !std::isfinite(v_m)) return FiniteDomainContainment::Outside;
	const double hx = static_cast<double>(domain.finite.half_extent_x_mm) * 0.001;
	const double hz = static_cast<double>(domain.finite.half_extent_z_mm) * 0.001;
	if (u_m < -hx || u_m > hx || v_m < -hz || v_m > hz) return FiniteDomainContainment::Outside;
	if (u_m == -hx || u_m == hx || v_m == -hz || v_m == hz) return FiniteDomainContainment::Boundary;
	return FiniteDomainContainment::Interior;
}

bool finite_block_intersects_domain(
	int64_t block_u,
	int64_t block_v,
	double block_size_m,
	const WorldDomainManifest& domain
) noexcept {
	if (!domain.is_valid() || !domain.is_finite() || !(block_size_m > 0.0) || !std::isfinite(block_size_m)) return false;
	const double min_u = static_cast<double>(block_u) * block_size_m;
	const double max_u = min_u + block_size_m;
	const double min_v = static_cast<double>(block_v) * block_size_m;
	const double max_v = min_v + block_size_m;
	const double hx = static_cast<double>(domain.finite.half_extent_x_mm) * 0.001;
	const double hz = static_cast<double>(domain.finite.half_extent_z_mm) * 0.001;
	return max_u >= -hx && min_u <= hx && max_v >= -hz && min_v <= hz;
}

uint8_t derive_domain_compatible_bccm_level_count(
	const WorldDomainManifest& domain,
	double lod0_block_size_m,
	uint8_t max_levels
) noexcept {
	if (!domain.is_valid() || !(lod0_block_size_m > 0.0) || !std::isfinite(lod0_block_size_m) || max_levels == 0) return 0;

	double representable_span_m = 0.0;
	if (domain.is_finite()) {
		representable_span_m = static_cast<double>(std::min(domain.finite.extent_x_m, domain.finite.extent_z_m));
	} else {
		representable_span_m = domain.closed_surface.area_equivalent_face_extent_m;
	}
	if (!(representable_span_m > 0.0) || !std::isfinite(representable_span_m)) return 0;

	uint8_t active_levels = 0;
	for (uint8_t lod = 0; lod < max_levels; ++lod) {
		const double block_size = std::ldexp(lod0_block_size_m, lod);
		if (!std::isfinite(block_size) || block_size > representable_span_m) break;
		active_levels = static_cast<uint8_t>(lod + 1);
	}
	// A valid domain always keeps LOD0 available, even when a caller supplies a
	// test span smaller than one ordinary block. The renderer will still reject
	// blocks outside the finite domain through its normal intersection test.
	return active_levels == 0 ? 1 : active_levels;
}

double closed_flat_chart_max_radius_m(const WorldDomainManifest& domain) noexcept {
	if (!domain.is_valid() || domain.is_finite() ||
		!(domain.closed_surface.logical_area_radius_m > 0.0) ||
		!std::isfinite(domain.closed_surface.logical_area_radius_m)) return 0.0;
	constexpr double LOCAL_METRIC_RADIUS_FRACTION = 0.5;
	return domain.closed_surface.logical_area_radius_m * LOCAL_METRIC_RADIUS_FRACTION;
}

uint8_t derive_flat_presentation_bccm_level_count(
	const WorldDomainManifest& domain,
	double lod0_block_size_m,
	int32_t candidate_grid_radius,
	uint8_t max_levels
) noexcept {
	const uint8_t domain_levels = derive_domain_compatible_bccm_level_count(
		domain, lod0_block_size_m, max_levels);
	if (!domain.is_valid() || domain.is_finite()) return domain_levels;
	if (!(lod0_block_size_m > 0.0) || !std::isfinite(lod0_block_size_m) ||
		candidate_grid_radius < 1 || max_levels == 0) return 0;

	const double max_radius_m = closed_flat_chart_max_radius_m(domain);
	if (!(max_radius_m > 0.0) || !std::isfinite(max_radius_m)) return 0;
	constexpr double SQRT_TWO = 1.41421356237309504880;
	uint8_t active_levels = 0;
	for (uint8_t lod = 0; lod < domain_levels; ++lod) {
		const double block_size_m = std::ldexp(lod0_block_size_m, lod);
		// The terminal ring snaps on its own block grid. One extra block covers
		// the camera-to-snap offset and the far corner of the outer candidate.
		const double far_corner_radius_m =
			(static_cast<double>(candidate_grid_radius) + 1.0) * block_size_m * SQRT_TWO;
		if (!std::isfinite(far_corner_radius_m) || far_corner_radius_m > max_radius_m) break;
		active_levels = static_cast<uint8_t>(lod + 1);
	}
	return active_levels == 0 ? 1 : active_levels;
}

} // namespace Multinet

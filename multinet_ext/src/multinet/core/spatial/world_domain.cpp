#include "multinet/core/spatial/world_domain.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Multinet {

namespace {
	constexpr uint64_t MAX_CANONICAL_EXTENT_M = 0xFFFFFFFFULL;

	[[nodiscard]] bool checked_area(uint64_t x_m, uint64_t z_m, uint64_t& out_area_m2) noexcept {
		if (x_m == 0 || z_m == 0 || z_m > (std::numeric_limits<uint64_t>::max)() / x_m) return false;
		out_area_m2 = x_m * z_m;
		return true;
	}

	[[nodiscard]] uint64_t round_canonical_metres(long double metres) noexcept {
		if (!std::isfinite(static_cast<double>(metres)) || metres <= 0.0L ||
			metres > static_cast<long double>(MAX_CANONICAL_EXTENT_M)) return 0;
		const long double rounded = std::floor(metres + 0.5L);
		if (rounded < 1.0L || rounded > static_cast<long double>(MAX_CANONICAL_EXTENT_M)) return 0;
		return static_cast<uint64_t>(rounded);
	}

	[[nodiscard]] WorldExtentConversionResult make_result(
		uint64_t x_m,
		uint64_t z_m,
		uint64_t requested_area_m2
	) noexcept {
		WorldExtentConversionResult result;
		result.extent_x_m = x_m;
		result.extent_z_m = z_m;
		result.requested_area_m2 = requested_area_m2;
		if (!checked_area(x_m, z_m, result.actual_area_m2)) return result;
		result.area_delta_m2 = static_cast<long double>(result.actual_area_m2) -
			static_cast<long double>(requested_area_m2);
		result.valid = true;
		return result;
	}
}

WorldExtentConversionResult square_extent_preserving_area(
	uint64_t extent_x_m,
	uint64_t extent_z_m
) noexcept {
	uint64_t requested_area_m2 = 0;
	if (!checked_area(extent_x_m, extent_z_m, requested_area_m2)) return {};
	const uint64_t side_m = round_canonical_metres(std::sqrt(static_cast<long double>(requested_area_m2)));
	return make_result(side_m, side_m, requested_area_m2);
}

WorldExtentConversionResult finite_extent_from_closed_side(
	uint64_t closed_side_m,
	uint64_t prior_extent_x_m,
	uint64_t prior_extent_z_m,
	bool has_prior_aspect
) noexcept {
	uint64_t requested_area_m2 = 0;
	if (!checked_area(closed_side_m, closed_side_m, requested_area_m2)) return {};
	if (!has_prior_aspect || prior_extent_x_m == 0 || prior_extent_z_m == 0) {
		return make_result(closed_side_m, closed_side_m, requested_area_m2);
	}
	const long double side = static_cast<long double>(closed_side_m);
	const long double ratio = static_cast<long double>(prior_extent_x_m) /
		static_cast<long double>(prior_extent_z_m);
	const uint64_t x_m = round_canonical_metres(side * std::sqrt(ratio));
	const uint64_t z_m = round_canonical_metres(side / std::sqrt(ratio));
	return make_result(x_m, z_m, requested_area_m2);
}

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

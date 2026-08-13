#ifndef MULTINET_WORLD_DOMAIN_H
#define MULTINET_WORLD_DOMAIN_H

#include "multinet/core/spatial/world_manifests.h"

#include <cstdint>

namespace Multinet {

enum class FiniteDomainContainment : uint8_t {
	Outside = 0,
	Boundary = 1,
	Interior = 2
};

// Editor/configuration conversion result. These helpers deliberately do not
// participate in WorldDomainManifest hashing; they only turn user-facing
// dimensions into canonical whole-metre extents.
struct WorldExtentConversionResult {
	uint64_t extent_x_m{ 0 };
	uint64_t extent_z_m{ 0 };
	uint64_t requested_area_m2{ 0 };
	uint64_t actual_area_m2{ 0 };
	long double area_delta_m2{ 0.0L };
	bool valid{ false };
};

[[nodiscard]] WorldExtentConversionResult square_extent_preserving_area(
	uint64_t extent_x_m,
	uint64_t extent_z_m
) noexcept;

[[nodiscard]] WorldExtentConversionResult finite_extent_from_closed_side(
	uint64_t closed_side_m,
	uint64_t prior_extent_x_m,
	uint64_t prior_extent_z_m,
	bool has_prior_aspect
) noexcept;

[[nodiscard]] FiniteDomainContainment classify_finite_position(
	double u_m,
	double v_m,
	const WorldDomainManifest& domain
) noexcept;

[[nodiscard]] bool finite_block_intersects_domain(
	int64_t block_u,
	int64_t block_v,
	double block_size_m,
	const WorldDomainManifest& domain
) noexcept;

// Returns the deterministic prefix of ordinary BCCM levels whose block
// footprint fits inside the smallest representable face-local domain span.
// LOD0 spacing/block geometry remains unchanged; this only gates outer levels
// for domains smaller than their ordinary block size.
[[nodiscard]] uint8_t derive_domain_compatible_bccm_level_count(
	const WorldDomainManifest& domain,
	double lod0_block_size_m = 32.0,
	uint8_t max_levels = 8
) noexcept;

// WP6 flat presentation is local. Beyond half a logical radius, even a
// continuous sphere chart changes metre scale enough to be visibly misleading.
// Distant coverage belongs to the later curvature/macro presentation work.
[[nodiscard]] double closed_flat_chart_max_radius_m(
	const WorldDomainManifest& domain
) noexcept;

[[nodiscard]] uint8_t derive_flat_presentation_bccm_level_count(
	const WorldDomainManifest& domain,
	double lod0_block_size_m = 32.0,
	int32_t candidate_grid_radius = 4,
	uint8_t max_levels = 8
) noexcept;

} // namespace Multinet

#endif // MULTINET_WORLD_DOMAIN_H

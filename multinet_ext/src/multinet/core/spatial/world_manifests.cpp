#include "multinet/core/spatial/world_manifests.h"
#include "multinet/core/spatial/surface_topology.h"
#include <cmath>
#include <limits>
#include <algorithm>

namespace Multinet {

namespace {
	inline void hash_combine_u64_le(uint64_t &hash, uint64_t val) {
		for (int i = 0; i < 8; ++i) {
			hash ^= (val & 0xFF);
			hash *= 1099511628211ULL;
			val >>= 8;
		}
	}
	inline void hash_combine_u32_le(uint64_t &hash, uint32_t val) {
		for (int i = 0; i < 4; ++i) {
			hash ^= (val & 0xFF);
			hash *= 1099511628211ULL;
			val >>= 8;
		}
	}
}

WorldScaleManifest build_world_scale_manifest(const WorldScaleInput& input) noexcept {
	WorldScaleManifest manifest;
	manifest.input = input;
	manifest.topology_version = 1;
	manifest.projection_version = 1; // SixParameterCOBEV1
	manifest.presentation_version = 1; // retained compatibility payload, never hashed
	
	// Overflow check
	if (input.area_equivalent_side_m == 0 || input.area_equivalent_side_m > 0xFFFFFFFFULL) {
		manifest.manifest_hash = 0; // Mark as invalid
		return manifest;
	}
	manifest.total_surface_area_m2 = input.area_equivalent_side_m * input.area_equivalent_side_m;
	manifest.logical_area_radius_m = std::sqrt(static_cast<double>(manifest.total_surface_area_m2) / (4.0 * 3.14159265358979323846));
	manifest.equivalent_diameter_m = 2.0 * manifest.logical_area_radius_m;
	manifest.great_circle_circumference_m = 2.0 * 3.14159265358979323846 * manifest.logical_area_radius_m;
	
	manifest.equal_face_area_m2 = static_cast<double>(manifest.total_surface_area_m2) / 6.0;
	manifest.area_equivalent_face_extent_m = static_cast<double>(input.area_equivalent_side_m) / std::sqrt(6.0);
	
	double ideal_face_extent_m = manifest.area_equivalent_face_extent_m;
	manifest.chart_half_extent_mm = static_cast<int64_t>(std::rint(ideal_face_extent_m * 500.0));
	
	double address_chart_extent_m = static_cast<double>(manifest.chart_half_extent_mm) * 0.002;
	manifest.regions_per_face_axis = static_cast<uint32_t>(std::round(address_chart_extent_m / 1024.0));
	if (manifest.regions_per_face_axis == 0) manifest.regions_per_face_axis = 1;
	
	manifest.actual_region_extent_m = address_chart_extent_m / static_cast<double>(manifest.regions_per_face_axis);
	
	uint64_t h = 14695981039346656037ULL; // FNV-1a 64-bit offset basis
	hash_combine_u32_le(h, WORLD_SCALE_MANIFEST_HASH_VERSION);
	hash_combine_u64_le(h, input.area_equivalent_side_m);
	hash_combine_u32_le(h, input.scale_schema_version);
	hash_combine_u32_le(h, manifest.topology_version);
	hash_combine_u32_le(h, manifest.projection_version);
	hash_combine_u64_le(h, static_cast<uint64_t>(manifest.chart_half_extent_mm));
	hash_combine_u32_le(h, manifest.regions_per_face_axis);
	
	if (h == 0) h = 1; // Must be nonzero
	manifest.manifest_hash = h;
	return manifest;
}

namespace {
	constexpr double TARGET_REGION_EXTENT_M = 1024.0;

	[[nodiscard]] bool checked_mul_u64(uint64_t a, uint64_t b, uint64_t& out) noexcept {
		if (a != 0 && b > (std::numeric_limits<uint64_t>::max)() / a) return false;
		out = a * b;
		return true;
	}

	[[nodiscard]] bool checked_metres_to_mm(uint64_t metres, int64_t& out_full_mm) noexcept {
		if (metres > static_cast<uint64_t>((std::numeric_limits<int64_t>::max)() / 1000)) return false;
		out_full_mm = static_cast<int64_t>(metres * 1000ULL);
		return true;
	}

	[[nodiscard]] uint32_t derive_region_count(uint64_t extent_m) noexcept {
		double count = std::round(static_cast<double>(extent_m) / TARGET_REGION_EXTENT_M);
		if (!std::isfinite(count) || count < 1.0 || count > static_cast<double>((std::numeric_limits<uint32_t>::max)())) return 0;
		return static_cast<uint32_t>(count);
	}
}

WorldDomainManifest build_world_domain_manifest(const WorldDomainInput& input) noexcept {
	WorldDomainManifest domain;
	domain.input = input;
	if (input.domain_schema_version != WORLD_DOMAIN_SCHEMA_VERSION_1) return domain;

	uint64_t canonical_area = 0;
	uint64_t hash = 14695981039346656037ULL;
	hash_combine_u32_le(hash, WORLD_DOMAIN_MANIFEST_HASH_VERSION_1);
	hash_combine_u32_le(hash, static_cast<uint32_t>(input.topology));
	hash_combine_u32_le(hash, input.domain_schema_version);

	if (input.topology == WorldDomainTopology::FiniteRectangle) {
		const uint64_t x = input.finite.extent_x_m;
		const uint64_t z = input.finite.extent_z_m;
		if (x == 0 || z == 0 || !checked_mul_u64(x, z, canonical_area)) return domain;

		int64_t full_x_mm = 0;
		int64_t full_z_mm = 0;
		if (!checked_metres_to_mm(x, full_x_mm) || !checked_metres_to_mm(z, full_z_mm)) return domain;

		domain.finite.extent_x_m = x;
		domain.finite.extent_z_m = z;
		domain.finite.half_extent_x_mm = full_x_mm / 2;
		domain.finite.half_extent_z_mm = full_z_mm / 2;
		domain.finite.regions_x = derive_region_count(x);
		domain.finite.regions_z = derive_region_count(z);
		if (domain.finite.regions_x == 0 || domain.finite.regions_z == 0) return domain;
		domain.finite.actual_region_extent_x_m = static_cast<double>(x) / domain.finite.regions_x;
		domain.finite.actual_region_extent_z_m = static_cast<double>(z) / domain.finite.regions_z;
		domain.canonical_area_m2 = canonical_area;
		domain.topology_version = TOPOLOGY_VERSION_FINITE_RECTANGLE_V1;
		domain.projection_version = PROJECTION_VERSION_NONE;

		hash_combine_u64_le(hash, x);
		hash_combine_u64_le(hash, z);
		hash_combine_u32_le(hash, domain.finite.regions_x);
		hash_combine_u32_le(hash, domain.finite.regions_z);
		hash_combine_u32_le(hash, domain.topology_version);
		hash_combine_u32_le(hash, domain.projection_version);
	} else if (input.topology == WorldDomainTopology::ClosedSurfaceSixFace) {
		WorldScaleInput closed_input;
		closed_input.area_equivalent_side_m = input.closed_surface.area_equivalent_side_m;
		closed_input.scale_schema_version = 1;
		domain.closed_surface = build_world_scale_manifest(closed_input);
		if (!domain.closed_surface.is_valid()) return domain;
		domain.canonical_area_m2 = domain.closed_surface.total_surface_area_m2;
		domain.topology_version = domain.closed_surface.topology_version;
		domain.projection_version = domain.closed_surface.projection_version;

		hash_combine_u64_le(hash, domain.closed_surface.manifest_hash);
		hash_combine_u32_le(hash, domain.topology_version);
		hash_combine_u32_le(hash, domain.projection_version);
	} else {
		return domain;
	}

	if (hash == 0) hash = 1;
	domain.domain_manifest_hash = hash;
	return domain;
}

WorldScaleManifest make_compatibility_scale_manifest(const WorldDomainManifest& domain) noexcept {
	if (!domain.is_valid()) return WorldScaleManifest{};
	if (!domain.is_finite()) return domain.closed_surface;
	WorldScaleManifest scale{};
	scale.input.area_equivalent_side_m = std::max(domain.finite.extent_x_m, domain.finite.extent_z_m);
	scale.total_surface_area_m2 = domain.canonical_area_m2;
	scale.chart_half_extent_mm = std::max(domain.finite.half_extent_x_mm, domain.finite.half_extent_z_mm);
	scale.regions_per_face_axis = std::max(domain.finite.regions_x, domain.finite.regions_z);
	scale.actual_region_extent_m = std::max(domain.finite.actual_region_extent_x_m, domain.finite.actual_region_extent_z_m);
	scale.logical_area_radius_m = std::sqrt(static_cast<double>(domain.canonical_area_m2) / (4.0 * 3.14159265358979323846));
	scale.topology_version = domain.topology_version;
	scale.projection_version = domain.projection_version;
	scale.manifest_hash = domain.domain_manifest_hash;
	return scale;
}

WorldPresentationManifest build_world_presentation_manifest(
	const WorldDomainManifest& domain,
	const WorldPresentationInput& input
) noexcept {
	WorldPresentationManifest manifest;
	if (!domain.is_valid()) return manifest;
	manifest.domain_manifest_hash = domain.domain_manifest_hash;
	manifest.chp_enabled = input.chp_enabled;
	manifest.presentation_version = input.presentation_version;
	const uint64_t area_equivalent_radius_mm = static_cast<uint64_t>(std::llround(
		std::sqrt(static_cast<double>(domain.canonical_area_m2) / (4.0 * 3.14159265358979323846)) * 1000.0));
	if (area_equivalent_radius_mm == 0) return WorldPresentationManifest{};

	if (!input.chp_enabled) {
		// Inactive editor values are not an active presentation contract. Keep a
		// deterministic, valid manifest so disabling CHP cannot fail on stale
		// hidden explicit-radius settings.
		manifest.chp_radius_policy = domain.is_finite()
			? CHPRadiusPolicy::AreaEquivalent
			: CHPRadiusPolicy::CanonicalClosedSurface;
		manifest.resolved_chp_radius_mm = domain.is_finite()
			? area_equivalent_radius_mm
			: static_cast<uint64_t>(std::llround(domain.closed_surface.logical_area_radius_m * 1000.0));
		manifest.chp_kernel_version = 0;
	} else {
		if (input.chp_kernel_version != CHP_KERNEL_CONTRACT_VERSION_1) return WorldPresentationManifest{};
		manifest.chp_kernel_version = CHP_KERNEL_CONTRACT_VERSION_1;
		if (!domain.is_finite()) {
			// A closed surface has one canonical equal-area presentation radius.
			manifest.chp_radius_policy = CHPRadiusPolicy::CanonicalClosedSurface;
			manifest.resolved_chp_radius_mm = static_cast<uint64_t>(std::llround(domain.closed_surface.logical_area_radius_m * 1000.0));
		} else if (input.chp_radius_policy == CHPRadiusPolicy::AreaEquivalent ||
			input.chp_radius_policy == CHPRadiusPolicy::CanonicalClosedSurface) {
			// CanonicalClosedSurface has no finite-domain meaning. Normalize it to
			// the finite area-equivalent policy before hashing the manifest.
			manifest.chp_radius_policy = CHPRadiusPolicy::AreaEquivalent;
			manifest.resolved_chp_radius_mm = area_equivalent_radius_mm;
		} else if (input.chp_radius_policy == CHPRadiusPolicy::Explicit) {
			if (input.explicit_chp_radius_mm == 0) return WorldPresentationManifest{};
			manifest.chp_radius_policy = CHPRadiusPolicy::Explicit;
			manifest.resolved_chp_radius_mm = input.explicit_chp_radius_mm;
		} else {
			return WorldPresentationManifest{};
		}
	}

	uint64_t h = 14695981039346656037ULL;
	hash_combine_u64_le(h, manifest.domain_manifest_hash);
	hash_combine_u32_le(h, manifest.chp_enabled ? 1u : 0u);
	hash_combine_u32_le(h, static_cast<uint32_t>(manifest.chp_radius_policy));
	hash_combine_u64_le(h, manifest.resolved_chp_radius_mm);
	hash_combine_u32_le(h, manifest.chp_kernel_version);
	hash_combine_u32_le(h, manifest.presentation_version);
	if (h == 0) h = 1;
	manifest.presentation_manifest_hash = h;
	return manifest;
}

} // namespace Multinet

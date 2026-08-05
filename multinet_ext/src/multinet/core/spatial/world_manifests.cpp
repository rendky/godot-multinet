#include "multinet/core/spatial/world_manifests.h"
#include <cmath>

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
	manifest.presentation_version = 1;
	
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
	hash_combine_u32_le(h, manifest.presentation_version);
	hash_combine_u64_le(h, static_cast<uint64_t>(manifest.chart_half_extent_mm));
	hash_combine_u32_le(h, manifest.regions_per_face_axis);
	
	if (h == 0) h = 1; // Must be nonzero
	manifest.manifest_hash = h;
	return manifest;
}

} // namespace Multinet

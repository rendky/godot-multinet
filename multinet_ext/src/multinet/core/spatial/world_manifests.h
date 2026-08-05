#ifndef MULTINET_WORLD_MANIFESTS_H
#define MULTINET_WORLD_MANIFESTS_H

#include <cstdint>

namespace Multinet {

struct WorldScaleInput {
	uint64_t area_equivalent_side_m{ 5000000 };
	uint32_t scale_schema_version{ 1 };
};

struct WorldScaleManifest {
	WorldScaleInput input{};

	uint64_t total_surface_area_m2{ 0 };

	double logical_area_radius_m{ 0.0 };
	double equivalent_diameter_m{ 0.0 };
	double great_circle_circumference_m{ 0.0 };

	double equal_face_area_m2{ 0.0 };
	double area_equivalent_face_extent_m{ 0.0 };

	int64_t chart_half_extent_mm{ 0 };
	uint32_t regions_per_face_axis{ 0 };
	double actual_region_extent_m{ 0.0 };

	uint32_t topology_version{ 1 };
	uint32_t projection_version{ 1 };
	uint32_t presentation_version{ 1 };

	uint64_t manifest_hash{ 0 };

	[[nodiscard]] bool is_valid() const noexcept {
		return manifest_hash != 0;
	}
};

constexpr uint32_t WORLD_SCALE_MANIFEST_HASH_VERSION = 1;

struct WorldPresentationManifest {
	WorldScaleManifest world_scale{};

	uint64_t canonical_radius_mm{ 0 };

	uint32_t chp_kernel_version{ 1 };
	uint32_t presentation_version{ 1 };
};

WorldScaleManifest build_world_scale_manifest(const WorldScaleInput& input) noexcept;

} // namespace Multinet

#endif // MULTINET_WORLD_MANIFESTS_H

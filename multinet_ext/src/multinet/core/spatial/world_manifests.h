#ifndef MULTINET_WORLD_MANIFESTS_H
#define MULTINET_WORLD_MANIFESTS_H

#include <cstdint>
#include <limits>

namespace Multinet {

enum class WorldDomainTopology : uint8_t {
	FiniteRectangle = 0,
	ClosedSurfaceSixFace = 1
};

static_assert(static_cast<uint8_t>(WorldDomainTopology::FiniteRectangle) == 0);
static_assert(static_cast<uint8_t>(WorldDomainTopology::ClosedSurfaceSixFace) == 1);

constexpr uint32_t WORLD_DOMAIN_SCHEMA_VERSION_1 = 1;
constexpr uint32_t WORLD_DOMAIN_MANIFEST_HASH_VERSION_1 = 1;
constexpr uint32_t TOPOLOGY_VERSION_FINITE_RECTANGLE_V1 = 1;
constexpr uint32_t PROJECTION_VERSION_NONE = 0;
constexpr float CANONICAL_ANALYTIC_NORMAL_SAMPLE_STEP_M = 0.5f;
constexpr uint32_t BCCM_ANALYTIC_NORMAL_VERSION_2 = 2;

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

constexpr uint32_t WORLD_SCALE_MANIFEST_HASH_VERSION = 2;

struct FiniteWorldExtentInput {
	uint64_t extent_x_m{ 5000000 };
	uint64_t extent_z_m{ 5000000 };
};

struct ClosedSurfaceScaleInput {
	uint64_t area_equivalent_side_m{ 5000000 };
};

struct WorldDomainInput {
	WorldDomainTopology topology{ WorldDomainTopology::ClosedSurfaceSixFace };
	FiniteWorldExtentInput finite{};
	ClosedSurfaceScaleInput closed_surface{};
	uint32_t domain_schema_version{ WORLD_DOMAIN_SCHEMA_VERSION_1 };
};

struct FiniteWorldManifest {
	uint64_t extent_x_m{ 0 };
	uint64_t extent_z_m{ 0 };
	int64_t half_extent_x_mm{ 0 };
	int64_t half_extent_z_mm{ 0 };
	uint32_t regions_x{ 0 };
	uint32_t regions_z{ 0 };
	double actual_region_extent_x_m{ 0.0 };
	double actual_region_extent_z_m{ 0.0 };
};

struct WorldDomainManifest {
	WorldDomainInput input{};
	uint64_t canonical_area_m2{ 0 };
	FiniteWorldManifest finite{};
	WorldScaleManifest closed_surface{};
	uint32_t topology_version{ 0 };
	uint32_t projection_version{ PROJECTION_VERSION_NONE };
	uint64_t domain_manifest_hash{ 0 };

	[[nodiscard]] bool is_valid() const noexcept {
		return domain_manifest_hash != 0;
	}
	[[nodiscard]] bool is_finite() const noexcept {
		return input.topology == WorldDomainTopology::FiniteRectangle;
	}
};

enum class CHPRadiusPolicy : uint8_t {
	CanonicalClosedSurface = 0,
	AreaEquivalent = 1,
	Explicit = 2
};

struct WorldPresentationInput {
	bool chp_enabled{ false };
	CHPRadiusPolicy chp_radius_policy{ CHPRadiusPolicy::CanonicalClosedSurface };
	uint64_t explicit_chp_radius_mm{ 0 };
	uint32_t chp_kernel_version{ 1 };
	uint32_t presentation_version{ 1 };
};

struct WorldPresentationManifest {
	uint64_t domain_manifest_hash{ 0 };
	bool chp_enabled{ false };
	CHPRadiusPolicy chp_radius_policy{ CHPRadiusPolicy::CanonicalClosedSurface };
	uint64_t resolved_chp_radius_mm{ 0 };
	uint32_t chp_kernel_version{ 1 };
	uint32_t presentation_version{ 1 };
	uint64_t presentation_manifest_hash{ 0 };

	[[nodiscard]] bool is_valid() const noexcept {
		return domain_manifest_hash != 0 && presentation_manifest_hash != 0;
	}
};

WorldScaleManifest build_world_scale_manifest(const WorldScaleInput& input) noexcept;
WorldDomainManifest build_world_domain_manifest(const WorldDomainInput& input) noexcept;
WorldScaleManifest make_compatibility_scale_manifest(const WorldDomainManifest& domain) noexcept;
WorldPresentationManifest build_world_presentation_manifest(
	const WorldDomainManifest& domain,
	const WorldPresentationInput& input
) noexcept;

} // namespace Multinet

#endif // MULTINET_WORLD_MANIFESTS_H

#ifndef MULTINET_TERRAIN_COMMITTED_DELTA_H
#define MULTINET_TERRAIN_COMMITTED_DELTA_H

#include "multinet/core/spatial/surface_address.h"
#include "multinet/core/spatial/surface_topology.h"
#include "multinet/core/spatial/surface_projection.h"
#include "multinet/core/spatial/world_manifests.h"
#include "multinet/core/spatial/world_domain.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_ids.h"
#include "multinet/rendering/terrain/block_clipmap/block_clipmap_profile.h"

#include <cstdint>
#include <memory>
#include <cmath>
#include <algorithm>
#include <array>

namespace Multinet {

struct FaceBounds {
	SurfaceFace face{ SurfaceFace::PositiveX };
	double min_u_m{ 0.0 };
	double max_u_m{ 0.0 };
	double min_v_m{ 0.0 };
	double max_v_m{ 0.0 };
	bool active{ false };
};

struct SurfaceBounds {
	std::array<FaceBounds, 6> face_bounds{};

	SurfaceBounds() {
		for (uint8_t f = 0; f < 6; ++f) {
			face_bounds[f].face = static_cast<SurfaceFace>(f);
			face_bounds[f].active = false;
		}
	}

	[[nodiscard]] bool is_empty() const noexcept {
		for (uint8_t f = 0; f < 6; ++f) {
			if (face_bounds[f].active) return false;
		}
		return true;
	}

	void add_bounds(SurfaceFace face, double min_u, double max_u, double min_v, double max_v) noexcept {
		uint8_t f = static_cast<uint8_t>(face);
		if (f >= 6) return;
		if (!face_bounds[f].active) {
			face_bounds[f].face = face;
			face_bounds[f].min_u_m = min_u;
			face_bounds[f].max_u_m = max_u;
			face_bounds[f].min_v_m = min_v;
			face_bounds[f].max_v_m = max_v;
			face_bounds[f].active = true;
		} else {
			face_bounds[f].min_u_m = std::min(face_bounds[f].min_u_m, min_u);
			face_bounds[f].max_u_m = std::max(face_bounds[f].max_u_m, max_u);
			face_bounds[f].min_v_m = std::min(face_bounds[f].min_v_m, min_v);
			face_bounds[f].max_v_m = std::max(face_bounds[f].max_v_m, max_v);
		}
	}

	[[nodiscard]] bool intersects(const SurfaceBounds& other) const noexcept {
		if (is_empty() || other.is_empty()) return false;
		for (uint8_t f = 0; f < 6; ++f) {
			if (face_bounds[f].active && other.face_bounds[f].active) {
				if (face_bounds[f].min_u_m < other.face_bounds[f].max_u_m &&
				    face_bounds[f].max_u_m > other.face_bounds[f].min_u_m &&
				    face_bounds[f].min_v_m < other.face_bounds[f].max_v_m &&
				    face_bounds[f].max_v_m > other.face_bounds[f].min_v_m) {
					return true;
				}
			}
		}
		return false;
	}
};

struct TerrainDeltaEnvelope {
	float minimum_delta_m{ 0.0f };
	float maximum_delta_m{ 0.0f };
	float maximum_abs_gradient{ 0.0f };
};

class TerrainCommittedDeltaField {
public:
	virtual ~TerrainCommittedDeltaField() = default;

	[[nodiscard]] virtual float sample_delta(
		SurfacePosition64 position
	) const noexcept = 0;

	[[nodiscard]] virtual bool block_may_have_nonzero_delta(
		const multinet::rendering::TerrainRenderBlockKey& key,
		const WorldScaleManifest& manifest,
		const multinet::rendering::BlockClipmapProfile& profile,
		double required_apron_m
	) const noexcept = 0;

	[[nodiscard]] virtual uint32_t get_block_content_version(
		const multinet::rendering::TerrainRenderBlockKey& key,
		const WorldScaleManifest& manifest,
		const multinet::rendering::BlockClipmapProfile& profile,
		double required_apron_m
	) const noexcept = 0;

	[[nodiscard]] virtual TerrainDeltaEnvelope get_conservative_envelope() const noexcept = 0;

	// Domain-aware compatibility hooks. Existing WP5 fields keep their closed
	// WorldScaleManifest implementation; finite callers add an exact boundary
	// admission check before falling back to that legacy footprint query.
	[[nodiscard]] virtual bool block_may_have_nonzero_delta(
		const multinet::rendering::TerrainRenderBlockKey& key,
		const WorldDomainManifest& domain,
		const multinet::rendering::BlockClipmapProfile& profile,
		double required_apron_m
	) const noexcept {
		if (!domain.is_valid()) return false;
		if (domain.is_finite() && !finite_block_intersects_domain(key.block_u, key.block_v, profile.get_lod_block_size(key.lod), domain)) return false;
		return block_may_have_nonzero_delta(key, make_compatibility_scale_manifest(domain), profile, required_apron_m);
	}

	[[nodiscard]] virtual uint32_t get_block_content_version(
		const multinet::rendering::TerrainRenderBlockKey& key,
		const WorldDomainManifest& domain,
		const multinet::rendering::BlockClipmapProfile& profile,
		double required_apron_m
	) const noexcept {
		if (!domain.is_valid()) return 1;
		if (domain.is_finite() && !finite_block_intersects_domain(key.block_u, key.block_v, profile.get_lod_block_size(key.lod), domain)) return 1;
		return get_block_content_version(key, make_compatibility_scale_manifest(domain), profile, required_apron_m);
	}
};

struct TerrainCommittedDeltaSnapshot {
	uint32_t contract_version{ 1 };
	uint32_t publication_version{ 1 };
	uint32_t source_version{ 1 };

	float minimum_delta_m{ 0.0f };
	float maximum_delta_m{ 0.0f };
	float maximum_abs_gradient{ 0.0f };

	SurfaceBounds support_bounds{};
	SurfaceBounds dirty_bounds{};

	std::shared_ptr<const TerrainCommittedDeltaField> field{ nullptr };
};

class NullTerrainCommittedDeltaField final : public TerrainCommittedDeltaField {
public:
	[[nodiscard]] float sample_delta(SurfacePosition64) const noexcept override {
		return 0.0f;
	}

	[[nodiscard]] bool block_may_have_nonzero_delta(
		const multinet::rendering::TerrainRenderBlockKey&,
		const WorldScaleManifest&,
		const multinet::rendering::BlockClipmapProfile&,
		double
	) const noexcept override {
		return false;
	}

	[[nodiscard]] uint32_t get_block_content_version(
		const multinet::rendering::TerrainRenderBlockKey&,
		const WorldScaleManifest&,
		const multinet::rendering::BlockClipmapProfile&,
		double
	) const noexcept override {
		return 1;
	}

	[[nodiscard]] TerrainDeltaEnvelope get_conservative_envelope() const noexcept override {
		return TerrainDeltaEnvelope{ 0.0f, 0.0f, 0.0f };
	}
};

// Diagnostic radial mound / depression field (TEST/DEBUG-ONLY)
class DiagnosticTerrainCommittedDeltaField final : public TerrainCommittedDeltaField {
private:
	SurfacePosition64 center{};
	double radius_m{ 1000.0 };
	float amplitude_m{ 50.0f };
	uint32_t content_version{ 1 };

public:
	DiagnosticTerrainCommittedDeltaField(
		SurfacePosition64 p_center,
		double p_radius_m,
		float p_amplitude_m,
		uint32_t p_content_version = 1
	) : center(p_center), radius_m(p_radius_m), amplitude_m(p_amplitude_m), content_version(p_content_version) {}

	[[nodiscard]] float sample_delta(SurfacePosition64 position) const noexcept override {
		if (radius_m <= 0.0) return 0.0f;
		if (position.face != center.face) return 0.0f;

		double du = position.u_m - center.u_m;
		double dv = position.v_m - center.v_m;
		double dist = std::sqrt(du * du + dv * dv);
		if (dist >= radius_m) return 0.0f;
		double t = dist / radius_m;
		double cos_shape = 0.5 * (1.0 + std::cos(t * 3.14159265358979323846));
		return static_cast<float>(amplitude_m * cos_shape);
	}

	[[nodiscard]] bool block_may_have_nonzero_delta(
		const multinet::rendering::TerrainRenderBlockKey& key,
		const WorldScaleManifest& manifest,
		const multinet::rendering::BlockClipmapProfile& profile,
		double required_apron_m
	) const noexcept override {
		if (key.face != center.face) return false;
		double block_size = profile.get_lod_block_size(key.lod);
		double b_min_u = key.block_u * block_size - required_apron_m;
		double b_max_u = (key.block_u + 1) * block_size + required_apron_m;
		double b_min_v = key.block_v * block_size - required_apron_m;
		double b_max_v = (key.block_v + 1) * block_size + required_apron_m;

		double c_min_u = center.u_m - radius_m;
		double c_max_u = center.u_m + radius_m;
		double c_min_v = center.v_m - radius_m;
		double c_max_v = center.v_m + radius_m;

		return !(b_min_u >= c_max_u || b_max_u <= c_min_u || b_min_v >= c_max_v || b_max_v <= c_min_v);
	}

	[[nodiscard]] uint32_t get_block_content_version(
		const multinet::rendering::TerrainRenderBlockKey& key,
		const WorldScaleManifest& manifest,
		const multinet::rendering::BlockClipmapProfile& profile,
		double required_apron_m
	) const noexcept override {
		if (block_may_have_nonzero_delta(key, manifest, profile, required_apron_m)) {
			return content_version;
		}
		return 1;
	}

	[[nodiscard]] TerrainDeltaEnvelope get_conservative_envelope() const noexcept override {
		float min_d = std::min(0.0f, amplitude_m);
		float max_d = std::max(0.0f, amplitude_m);
		float max_grad = (radius_m > 0.0) ? static_cast<float>(0.5 * std::abs(amplitude_m) * 3.14159265358979323846 / radius_m) : 0.0f;
		return TerrainDeltaEnvelope{ min_d, max_d, max_grad };
	}
};

// Canonical 6-face diagnostic radial mound / depression field
class CanonicalDiagnosticTerrainCommittedDeltaField final : public TerrainCommittedDeltaField {
private:
	SurfacePosition64 center{};
	double radius_m{ 1000.0 };
	float amplitude_m{ 50.0f };
	uint32_t content_version{ 1 };
	WorldScaleManifest scale_manifest{};

	[[nodiscard]] FramePosition64 to_canonical_3d(SurfacePosition64 pos) const noexcept {
		SurfaceAddress addr;
		addr.face = pos.face;
		addr.u_mm = static_cast<int64_t>(std::round(pos.u_m * 1000.0));
		addr.v_mm = static_cast<int64_t>(std::round(pos.v_m * 1000.0));
		addr.topology_version = scale_manifest.topology_version;
		addr.projection_version = scale_manifest.projection_version;

		SurfaceAddress canon = canonicalize_surface_address(addr, scale_manifest);
		double half_side = static_cast<double>(scale_manifest.chart_half_extent_mm) * 0.001;
		if (half_side <= 0.0) half_side = 500000.0;

		double norm_u = (canon.u_mm * 0.001) / half_side;
		double norm_v = (canon.v_mm * 0.001) / half_side;

		norm_u = std::clamp(norm_u, -1.0, 1.0);
		norm_v = std::clamp(norm_v, -1.0, 1.0);

		return ProjectionCOBE::map_forward(static_cast<int>(canon.face), norm_u, norm_v);
	}

public:
	CanonicalDiagnosticTerrainCommittedDeltaField(
		SurfacePosition64 p_center,
		double p_radius_m,
		float p_amplitude_m,
		const WorldScaleManifest& p_scale,
		uint32_t p_content_version = 1
	) : center(p_center), radius_m(p_radius_m), amplitude_m(p_amplitude_m), scale_manifest(p_scale), content_version(p_content_version) {}

	[[nodiscard]] float sample_delta(SurfacePosition64 position) const noexcept override {
		if (radius_m <= 0.0) return 0.0f;

		double R = scale_manifest.logical_area_radius_m;
		if (R <= 0.0) R = 6371000.0;

		FramePosition64 p_center = to_canonical_3d(center);
		FramePosition64 p_pos = to_canonical_3d(position);

		double dx = (p_pos.x - p_center.x) * R;
		double dy = (p_pos.y - p_center.y) * R;
		double dz = (p_pos.z - p_center.z) * R;
		double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

		if (dist >= radius_m) return 0.0f;
		double t = dist / radius_m;
		double cos_shape = 0.5 * (1.0 + std::cos(t * 3.14159265358979323846));
		return static_cast<float>(amplitude_m * cos_shape);
	}

	[[nodiscard]] bool block_may_have_nonzero_delta(
		const multinet::rendering::TerrainRenderBlockKey& key,
		const WorldScaleManifest& manifest,
		const multinet::rendering::BlockClipmapProfile& profile,
		double required_apron_m
	) const noexcept override {
		if (radius_m <= 0.0) return false;

		double R = manifest.logical_area_radius_m;
		if (R <= 0.0) R = 6371000.0;

		FramePosition64 p_center = to_canonical_3d(center);

		double block_size = profile.get_lod_block_size(key.lod);
		double b_min_u = key.block_u * block_size - required_apron_m;
		double b_max_u = (key.block_u + 1) * block_size + required_apron_m;
		double b_min_v = key.block_v * block_size - required_apron_m;
		double b_max_v = (key.block_v + 1) * block_size + required_apron_m;

		double mid_u = (b_min_u + b_max_u) * 0.5;
		double mid_v = (b_min_v + b_max_v) * 0.5;

		SurfacePosition64 block_pos;
		block_pos.face = key.face;
		block_pos.u_m = mid_u;
		block_pos.v_m = mid_v;
		block_pos.altitude_m = 0.0;
		block_pos.topology_version = manifest.topology_version;
		block_pos.projection_version = manifest.projection_version;

		FramePosition64 p_block = to_canonical_3d(block_pos);

		double dx = (p_block.x - p_center.x) * R;
		double dy = (p_block.y - p_center.y) * R;
		double dz = (p_block.z - p_center.z) * R;
		double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

		double block_diag = std::sqrt(2.0) * (block_size + required_apron_m * 2.0);
		return dist <= (radius_m + block_diag);
	}

	[[nodiscard]] uint32_t get_block_content_version(
		const multinet::rendering::TerrainRenderBlockKey& key,
		const WorldScaleManifest& manifest,
		const multinet::rendering::BlockClipmapProfile& profile,
		double required_apron_m
	) const noexcept override {
		if (block_may_have_nonzero_delta(key, manifest, profile, required_apron_m)) {
			return content_version;
		}
		return 1;
	}

	[[nodiscard]] TerrainDeltaEnvelope get_conservative_envelope() const noexcept override {
		float min_d = std::min(0.0f, amplitude_m);
		float max_d = std::max(0.0f, amplitude_m);
		float max_grad = (radius_m > 0.0) ? static_cast<float>(0.5 * std::abs(amplitude_m) * 3.14159265358979323846 / radius_m) : 0.0f;
		return TerrainDeltaEnvelope{ min_d, max_d, max_grad };
	}
};

constexpr uint32_t TERRAIN_PAGE_CONTRACT_VERSION_1 = 1;

inline TerrainCommittedDeltaSnapshot make_null_committed_delta_snapshot(uint32_t publication_version = 1) {
	TerrainCommittedDeltaSnapshot snap{};
	snap.publication_version = publication_version;
	snap.contract_version = TERRAIN_PAGE_CONTRACT_VERSION_1;
	snap.minimum_delta_m = 0.0f;
	snap.maximum_delta_m = 0.0f;
	snap.maximum_abs_gradient = 0.0f;
	snap.field = std::make_shared<NullTerrainCommittedDeltaField>();
	return snap;
}

inline TerrainCommittedDeltaSnapshot make_diagnostic_committed_delta_snapshot(
	SurfacePosition64 center,
	double radius_m,
	float amplitude_m,
	uint32_t content_version = 1,
	uint32_t publication_version = 1
) {
	TerrainCommittedDeltaSnapshot snap{};
	snap.publication_version = publication_version;
	snap.contract_version = TERRAIN_PAGE_CONTRACT_VERSION_1;
	auto diag_field = std::make_shared<DiagnosticTerrainCommittedDeltaField>(center, radius_m, amplitude_m, content_version);
	snap.field = diag_field;
	TerrainDeltaEnvelope env = diag_field->get_conservative_envelope();
	snap.minimum_delta_m = env.minimum_delta_m;
	snap.maximum_delta_m = env.maximum_delta_m;
	snap.maximum_abs_gradient = env.maximum_abs_gradient;
	snap.support_bounds.add_bounds(center.face, center.u_m - radius_m, center.u_m + radius_m, center.v_m - radius_m, center.v_m + radius_m);
	snap.dirty_bounds = snap.support_bounds;
	return snap;
}

} // namespace Multinet

#endif // MULTINET_TERRAIN_COMMITTED_DELTA_H

#ifndef MULTINET_TERRAIN_QUERIES_H
#define MULTINET_TERRAIN_QUERIES_H

#include "multinet/core/coordinates.h"
#include "multinet/core/span.h"
#include "multinet/core/memory/arena_allocator.h"
#include "multinet/world/terrain/heightfield_generator.h"
#include "multinet/world/terrain/canonical_terrain_signal.h"

#include "multinet/core/spatial/surface_projection.h"
#include "multinet/core/spatial/surface_address.h"
#include "multinet/core/spatial/surface_frame.h"
#include "multinet/core/spatial/world_manifests.h"

#include <stdexcept>

namespace Multinet {

enum class TerrainQueryFlags : uint32_t {
	None = 0,
	Normals = 1 << 0,
	Materials = 1 << 1
};

inline TerrainQueryFlags operator|(TerrainQueryFlags a, TerrainQueryFlags b) {
	return static_cast<TerrainQueryFlags>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline bool operator&(TerrainQueryFlags a, TerrainQueryFlags b) {
	return (static_cast<uint32_t>(a) & static_cast<uint32_t>(b)) != 0;
}

struct TerrainHeightEvaluation {
	bool valid{ false };
	double height{ 0.0 };
	SurfaceNormal normal{};
	uint32_t version{ 0 };
	// material identity placeholder
};

class TerrainFieldEvaluator {
private:
	CanonicalTerrainSignalV1 canonical_generator;
	LegacyPlanarTerrainSignalV1 legacy_generator;
	WorldScaleManifest scale;
	uint32_t current_version{ 1 };
	bool is_legacy{ false };

public:
	TerrainFieldEvaluator() = default;
	
	explicit TerrainFieldEvaluator(const CanonicalTerrainSignalV1 &p_generator, const WorldScaleManifest &p_scale, uint32_t p_version = 1)
		: canonical_generator(p_generator), scale(p_scale), current_version(p_version), is_legacy(false) {
		const auto& identity = canonical_generator.get_recipe().identity;
		if (identity.topology_version != scale.topology_version ||
		    identity.projection_version != scale.projection_version ||
		    identity.manifest_hash != scale.manifest_hash ||
		    identity.deterministic_algorithm_id != 0x53513502u) {
			throw std::invalid_argument("Terrain recipe identity mismatch");
		}
	}

	explicit TerrainFieldEvaluator(const LegacyPlanarTerrainSignalV1 &p_generator, const WorldScaleManifest &p_scale, uint32_t p_version = 1)
		: legacy_generator(p_generator), scale(p_scale), current_version(p_version), is_legacy(true) {}

	[[nodiscard]] double evaluate_height_canonical(SurfacePosition64 position) const noexcept {
		if (is_legacy) {
			return legacy_generator.evaluate_height(position.u_m, position.v_m);
		} else {
			return canonical_generator.evaluate_height(position);
		}
	}

	[[nodiscard]] TerrainHeightEvaluation evaluate(
		SurfacePosition64 position,
		TerrainQueryFlags flags = TerrainQueryFlags::None
	) const noexcept {
		TerrainHeightEvaluation result;
		result.version = current_version;
		
		if (!position.is_valid()) return result;
		if (!std::isfinite(position.u_m) || !std::isfinite(position.v_m) || !std::isfinite(position.altitude_m)) return result;
		
		if (!is_legacy) {
			if (position.topology_version != scale.topology_version || position.projection_version != scale.projection_version) return result;
			
			SurfaceAddress addr;
			addr.face = position.face;
			addr.u_mm = static_cast<int64_t>(std::round(position.u_m * 1000.0));
			addr.v_mm = static_cast<int64_t>(std::round(position.v_m * 1000.0));
			addr.topology_version = position.topology_version;
			addr.projection_version = position.projection_version;

			SurfaceAddress canon = canonicalize_surface_address(addr, scale);
			if (!canon.is_valid()) return result;
		}

		result.height = evaluate_height_canonical(position);
		result.valid = true;
		
		if (flags & TerrainQueryFlags::Normals) {
			double step = 0.5; // 500mm sample step
			
			SurfacePosition64 pos_u_pos = position; pos_u_pos.u_m += step;
			SurfacePosition64 pos_u_neg = position; pos_u_neg.u_m -= step;
			SurfacePosition64 pos_v_pos = position; pos_v_pos.v_m += step;
			SurfacePosition64 pos_v_neg = position; pos_v_neg.v_m -= step;
			
			double hR = evaluate_height_canonical(pos_u_pos);
			double hL = evaluate_height_canonical(pos_u_neg);
			double hU = evaluate_height_canonical(pos_v_pos);
			double hD = evaluate_height_canonical(pos_v_neg);
			
			// Compute finite differences in the local U/V parametric space of the queried face
			// Note: The caller's requested face coordinate frame provides the tangent space!
			double du = (hR - hL) / (2.0 * step);
			double dv = (hU - hD) / (2.0 * step);
			
			double len = std::sqrt(du * du + 1.0 + dv * dv);
			result.normal.nx = static_cast<float>(-du / len);
			result.normal.ny = static_cast<float>(1.0 / len);
			result.normal.nz = static_cast<float>(-dv / len);
		}
		
		return result;
	}
};

} // namespace Multinet

#endif // MULTINET_TERRAIN_QUERIES_H

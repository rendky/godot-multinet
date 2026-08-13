#ifndef MULTINET_COMPOSITE_TERRAIN_FIELD_EVALUATOR_H
#define MULTINET_COMPOSITE_TERRAIN_FIELD_EVALUATOR_H

#include "multinet/world/terrain/canonical_terrain_signal.h"
#include "multinet/world/terrain/terrain_committed_delta.h"

namespace Multinet {

class CompositeTerrainFieldEvaluator {
private:
	CanonicalTerrainSignalV1 base_evaluator{};
	TerrainCommittedDeltaSnapshot delta_snapshot{};

public:
	CompositeTerrainFieldEvaluator() = default;

	CompositeTerrainFieldEvaluator(
		const CanonicalTerrainSignalV1& p_base,
		const TerrainCommittedDeltaSnapshot& p_delta
	) : base_evaluator(p_base), delta_snapshot(p_delta) {}

	[[nodiscard]] double evaluate_height(const SurfacePosition64& position) const noexcept {
		double base_h = base_evaluator.evaluate_height(position);
		if (delta_snapshot.field) {
			float delta_h = delta_snapshot.field->sample_delta(position);
			return base_h + static_cast<double>(delta_h);
		}
		return base_h;
	}

	[[nodiscard]] godot::Vector3 evaluate_normal(
		const SurfacePosition64& position,
		double lod_spacing
	) const noexcept {
		SurfacePosition64 pos_rt = position; pos_rt.u_m += lod_spacing;
		SurfacePosition64 pos_lf = position; pos_lf.u_m -= lod_spacing;
		SurfacePosition64 pos_dn = position; pos_dn.v_m += lod_spacing;
		SurfacePosition64 pos_up = position; pos_up.v_m -= lod_spacing;

		float h_rt = static_cast<float>(evaluate_height(pos_rt));
		float h_lf = static_cast<float>(evaluate_height(pos_lf));
		float h_dn = static_cast<float>(evaluate_height(pos_dn));
		float h_up = static_cast<float>(evaluate_height(pos_up));

		godot::Vector3 du(static_cast<float>(2.0 * lod_spacing), h_rt - h_lf, 0.0f);
		godot::Vector3 dv(0.0f, h_dn - h_up, static_cast<float>(2.0 * lod_spacing));

		return dv.cross(du).normalized();
	}
};

} // namespace Multinet

#endif // MULTINET_COMPOSITE_TERRAIN_FIELD_EVALUATOR_H

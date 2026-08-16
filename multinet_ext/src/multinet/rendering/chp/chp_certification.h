#ifndef MULTINET_RENDERING_CHP_CERTIFICATION_H
#define MULTINET_RENDERING_CHP_CERTIFICATION_H

#include "multinet/rendering/chp/chp_kernel.h"

namespace multinet::rendering::chp {

[[nodiscard]] bool try_resolve_curved_horizon_profile(
	const Multinet::WorldPresentationManifest& presentation,
	const CurvedHorizonProfile& requested,
	ResolvedCurvedHorizonProfile& out_profile
) noexcept;

} // namespace multinet::rendering::chp

#endif // MULTINET_RENDERING_CHP_CERTIFICATION_H

#ifndef MULTINET_RENDERING_CHP_KERNEL_H
#define MULTINET_RENDERING_CHP_KERNEL_H

#include "multinet/rendering/chp/chp_profile.h"

namespace multinet::rendering::chp {

[[nodiscard]] const char *get_function_class_name(CHPFunctionClass value) noexcept;

[[nodiscard]] bool try_evaluate_flat(
	const CHPIntrinsicSample& sample,
	CHPEvaluation& out_evaluation
) noexcept;

[[nodiscard]] bool try_evaluate_curved(
	const ResolvedCurvedHorizonProfile& profile,
	const CHPIntrinsicSample& sample,
	CHPEvaluation& out_evaluation
) noexcept;

// Independent scalar reference. This is intentionally not used by the
// production polynomial path; certification and fixtures are its only users.
[[nodiscard]] bool try_evaluate_exact_sphere(
	double radius_m,
	const CHPIntrinsicSample& sample,
	CHPEvaluation& out_evaluation
) noexcept;

} // namespace multinet::rendering::chp

#endif // MULTINET_RENDERING_CHP_KERNEL_H

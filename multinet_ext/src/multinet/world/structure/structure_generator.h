#ifndef MULTINET_STRUCTURE_GENERATOR_H
#define MULTINET_STRUCTURE_GENERATOR_H

#include "../settlement/settlement_types.h"
#include "structure_package.h"
#include "structure_layout_solver.h"

namespace Multinet {

// ============================================================================
// StructureGenerator - M4 Broad Structure Generator
// Consumes a BuildingDevelopmentRequest from Settlement and generates a
// CompiledBuildingPackage (foundation, rooms, portals, structural graph)
// by delegating to the procedural StructureLayoutSolver.
// ============================================================================

class StructureGenerator {
public:
	[[nodiscard]] static constexpr bool generate_from_request(
			const BuildingDevelopmentRequest &p_request,
			CompiledBuildingPackage &out_package) noexcept {
		
		return StructureLayoutSolver::solve_layout(p_request, out_package);
	}
};

} // namespace Multinet

#endif // MULTINET_STRUCTURE_GENERATOR_H

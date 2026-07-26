#ifndef MULTINET_DIRTY_BOUNDS_H
#define MULTINET_DIRTY_BOUNDS_H

#include "core/coordinates.h"

#include <algorithm>
#include <cstdint>

namespace Multinet {

struct DirtyBounds3D {
	RegionPosition min_pos{};
	RegionPosition max_pos{};
	bool is_dirty{ false };

	DirtyBounds3D() = default;

	void expand(const RegionPosition &p_pos) noexcept {
		if (!is_dirty) {
			min_pos = p_pos;
			max_pos = p_pos;
			is_dirty = true;
			return;
		}

		min_pos.cell_x = std::min(min_pos.cell_x, p_pos.cell_x);
		min_pos.cell_y = std::min(min_pos.cell_y, p_pos.cell_y);
		min_pos.cell_z = std::min(min_pos.cell_z, p_pos.cell_z);

		max_pos.cell_x = std::max(max_pos.cell_x, p_pos.cell_x);
		max_pos.cell_y = std::max(max_pos.cell_y, p_pos.cell_y);
		max_pos.cell_z = std::max(max_pos.cell_z, p_pos.cell_z);
	}

	void clear() noexcept {
		min_pos = RegionPosition{};
		max_pos = RegionPosition{};
		is_dirty = false;
	}
};

} // namespace Multinet

#endif // MULTINET_DIRTY_BOUNDS_H

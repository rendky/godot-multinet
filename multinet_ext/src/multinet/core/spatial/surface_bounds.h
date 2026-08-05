#ifndef MULTINET_SURFACE_BOUNDS_H
#define MULTINET_SURFACE_BOUNDS_H

#include "multinet/core/spatial/surface_address.h"
#include <algorithm>

namespace Multinet {

struct SurfaceFaceBounds {
	int64_t min_u_mm{ 0 };
	int64_t max_u_mm{ 0 };
	int64_t min_v_mm{ 0 };
	int64_t max_v_mm{ 0 };
	bool is_dirty{ false };
};

struct SurfaceBounds {
	SurfaceFaceBounds faces[6]{};
	bool is_dirty{ false };

	void expand(const SurfaceAddress &p_pos) noexcept {
		if (!p_pos.is_valid()) return;
		int face_idx = static_cast<int>(p_pos.face);
		auto &b = faces[face_idx];

		if (!b.is_dirty) {
			b.min_u_mm = p_pos.u_mm;
			b.max_u_mm = p_pos.u_mm;
			b.min_v_mm = p_pos.v_mm;
			b.max_v_mm = p_pos.v_mm;
			b.is_dirty = true;
			is_dirty = true;
		} else {
			b.min_u_mm = std::min(b.min_u_mm, p_pos.u_mm);
			b.max_u_mm = std::max(b.max_u_mm, p_pos.u_mm);
			b.min_v_mm = std::min(b.min_v_mm, p_pos.v_mm);
			b.max_v_mm = std::max(b.max_v_mm, p_pos.v_mm);
		}
	}

	void clear() noexcept {
		for (int i = 0; i < 6; ++i) {
			faces[i].is_dirty = false;
		}
		is_dirty = false;
	}
};

} // namespace Multinet

#endif // MULTINET_SURFACE_BOUNDS_H

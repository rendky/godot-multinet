#ifndef MULTINET_BLOCK_CLIPMAP_CULLING_H
#define MULTINET_BLOCK_CLIPMAP_CULLING_H

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/plane.hpp>
#include <godot_cpp/variant/aabb.hpp>

#include <cmath>
#include <cstdint>
#include <array>

namespace multinet::rendering {

// Signed floor division for integer world lattice placement
inline int64_t floor_div(int64_t a, int64_t b) {
	int64_t res = a / b;
	int64_t rem = a % b;
	if (rem != 0 && ((a < 0) ^ (b < 0))) {
		res--;
	}
	return res;
}

inline int64_t floor_div_f(double coord, double block_size) {
	return static_cast<int64_t>(std::floor(coord / block_size));
}

struct FrustumPlanes {
	std::array<godot::Plane, 6> planes;
	bool valid{ false };

	static FrustumPlanes extract_from_camera(godot::Camera3D *p_camera) {
		FrustumPlanes f;
		if (!p_camera) return f;

		godot::TypedArray<godot::Plane> godot_planes = p_camera->get_frustum();
		if (godot_planes.size() >= 6) {
			for (int i = 0; i < 6; ++i) {
				f.planes[i] = godot_planes[i];
			}
			f.valid = true;
		}
		return f;
	}

	bool intersects_aabb(const godot::AABB &p_aabb) const {
		if (!valid) return true; // Conservatively include block if frustum extraction failed

		godot::Vector3 min_p = p_aabb.position;
		godot::Vector3 max_p = p_aabb.position + p_aabb.size;

		for (int i = 0; i < 6; ++i) {
			const godot::Plane &p = planes[i];
			// Find the vertex of the AABB that is *least* in the direction of the outward normal
			godot::Vector3 n_vertex(
				p.normal.x >= 0 ? min_p.x : max_p.x,
				p.normal.y >= 0 ? min_p.y : max_p.y,
				p.normal.z >= 0 ? min_p.z : max_p.z
			);
			
			// If even the most-inward vertex is in the positive half-space (outside), cull it
			if (p.distance_to(n_vertex) > 0.0f) {
				return false;
			}
		}
		return true;
	}
};

} // namespace multinet::rendering

#endif // MULTINET_BLOCK_CLIPMAP_CULLING_H

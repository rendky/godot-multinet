#ifndef MULTINET_SURFACE_FACE_H
#define MULTINET_SURFACE_FACE_H

#include <cstdint>

namespace Multinet {

enum class SurfaceFace : uint8_t {
	PositiveX = 0,
	NegativeX = 1,
	PositiveY = 2,
	NegativeY = 3,
	PositiveZ = 4,
	NegativeZ = 5
};

[[nodiscard]] constexpr bool is_valid_surface_face(SurfaceFace p_face) noexcept {
	return static_cast<uint8_t>(p_face) <= 5;
}

[[nodiscard]] constexpr const char *get_surface_face_debug_name(SurfaceFace p_face) noexcept {
	switch (p_face) {
		case SurfaceFace::PositiveX: return "PositiveX (Prime/East)";
		case SurfaceFace::NegativeX: return "NegativeX (West)";
		case SurfaceFace::PositiveY: return "PositiveY (North)";
		case SurfaceFace::NegativeY: return "NegativeY (South)";
		case SurfaceFace::PositiveZ: return "PositiveZ (Far)";
		case SurfaceFace::NegativeZ: return "NegativeZ (Near)";
		default: return "InvalidFace";
	}
}

} // namespace Multinet

#endif // MULTINET_SURFACE_FACE_H

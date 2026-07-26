#ifndef MULTINET_COORDINATES_H
#define MULTINET_COORDINATES_H

#include <cmath>
#include <cstdint>

namespace Multinet {

struct WorldPosition64 {
	double x{ 0.0 };
	double y{ 0.0 };
	double z{ 0.0 };

	[[nodiscard]] constexpr bool operator==(const WorldPosition64 &p_other) const noexcept {
		return x == p_other.x && y == p_other.y && z == p_other.z;
	}
};

struct RegionPosition {
	static constexpr double CELL_SIZE = 1024.0; // 1024m region cell

	int64_t cell_x{ 0 };
	int64_t cell_y{ 0 };
	int64_t cell_z{ 0 };

	float local_x{ 0.0f };
	float local_y{ 0.0f };
	float local_z{ 0.0f };

	static RegionPosition from_world(const WorldPosition64 &p_world) noexcept {
		RegionPosition pos{};

		double cx = std::floor(p_world.x / CELL_SIZE);
		double cy = std::floor(p_world.y / CELL_SIZE);
		double cz = std::floor(p_world.z / CELL_SIZE);

		pos.cell_x = static_cast<int64_t>(cx);
		pos.cell_y = static_cast<int64_t>(cy);
		pos.cell_z = static_cast<int64_t>(cz);

		pos.local_x = static_cast<float>(p_world.x - (cx * CELL_SIZE));
		pos.local_y = static_cast<float>(p_world.y - (cy * CELL_SIZE));
		pos.local_z = static_cast<float>(p_world.z - (cz * CELL_SIZE));

		return pos;
	}

	[[nodiscard]] WorldPosition64 to_world() const noexcept {
		return WorldPosition64{
			static_cast<double>(cell_x) * CELL_SIZE + static_cast<double>(local_x),
			static_cast<double>(cell_y) * CELL_SIZE + static_cast<double>(local_y),
			static_cast<double>(cell_z) * CELL_SIZE + static_cast<double>(local_z)
		};
	}
};

struct QuantizedTransform {
	int16_t pos_x{ 0 };
	int16_t pos_y{ 0 };
	int16_t pos_z{ 0 };
	int16_t rot_x{ 0 };
	int16_t rot_y{ 0 };
	int16_t rot_z{ 0 };
	int16_t rot_w{ 32767 };
};

} // namespace Multinet

#endif // MULTINET_COORDINATES_H

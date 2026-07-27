#ifndef MULTINET_SQUIRREL_NOISE5_H
#define MULTINET_SQUIRREL_NOISE5_H

#include <cstdint>

namespace Multinet {

// ============================================================================
// SquirrelNoise5 v1 (Canonical Bit-Noise Hash - Rule 4)
// Identical bits/seed/version match across C++, SIMD, Godot & shaders.
// ============================================================================

constexpr uint32_t SQ5_BIT_NOISE1 = 0xD6E8FEB8ULL;
constexpr uint32_t SQ5_BIT_NOISE2 = 0x9656979DULL;
constexpr uint32_t SQ5_BIT_NOISE3 = 0x5D588B65ULL;
constexpr uint32_t SQ5_BIT_NOISE4 = 0xE16B0127ULL;
constexpr uint32_t SQ5_BIT_NOISE5 = 0x2A01A19CULL;

[[nodiscard]] inline constexpr uint32_t squirrel_noise5(int32_t p_position_1d, uint32_t p_seed) noexcept {
	uint32_t mangled = static_cast<uint32_t>(p_position_1d);
	mangled *= SQ5_BIT_NOISE1;
	mangled += p_seed;
	mangled ^= (mangled >> 9);
	mangled += SQ5_BIT_NOISE2;
	mangled ^= (mangled >> 11);
	mangled *= SQ5_BIT_NOISE3;
	mangled ^= (mangled >> 13);
	mangled += SQ5_BIT_NOISE4;
	mangled ^= (mangled >> 17);
	mangled *= SQ5_BIT_NOISE5;
	mangled ^= (mangled >> 19);
	return mangled;
}

[[nodiscard]] inline constexpr uint32_t squirrel_noise5_2d(int32_t p_x, int32_t p_y, uint32_t p_seed) noexcept {
	constexpr int32_t PRIME_Y = 198491317;
	return squirrel_noise5(p_x + (p_y * PRIME_Y), p_seed);
}

[[nodiscard]] inline constexpr uint32_t squirrel_noise5_3d(int32_t p_x, int32_t p_y, int32_t p_z, uint32_t p_seed) noexcept {
	constexpr int32_t PRIME_Y = 198491317;
	constexpr int32_t PRIME_Z = 6542989;
	return squirrel_noise5(p_x + (p_y * PRIME_Y) + (p_z * PRIME_Z), p_seed);
}

[[nodiscard]] inline constexpr float squirrel_noise5_zero_to_one(int32_t p_position_1d, uint32_t p_seed) noexcept {
	constexpr double ONE_OVER_MAX = 1.0 / 4294967295.0;
	return static_cast<float>(squirrel_noise5(p_position_1d, p_seed) * ONE_OVER_MAX);
}

[[nodiscard]] inline constexpr float squirrel_noise5_2d_zero_to_one(int32_t p_x, int32_t p_y, uint32_t p_seed) noexcept {
	constexpr double ONE_OVER_MAX = 1.0 / 4294967295.0;
	return static_cast<float>(squirrel_noise5_2d(p_x, p_y, p_seed) * ONE_OVER_MAX);
}

} // namespace Multinet

#endif // MULTINET_SQUIRREL_NOISE5_H

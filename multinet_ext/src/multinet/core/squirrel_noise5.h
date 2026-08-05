#ifndef MULTINET_SQUIRREL_NOISE5_H
#define MULTINET_SQUIRREL_NOISE5_H

#include <cstdint>

namespace Multinet {

// ============================================================================
// SquirrelNoise5 v1 (Canonical Bit-Noise Hash - Rule 4)
// Identical bits/seed/version match across C++, SIMD, Godot & shaders.
// ============================================================================

enum class DeterministicAlgorithm : uint32_t {
    SquirrelNoise5U2V1 = 0x53513501u,
    SquirrelNoise5U3V1 = 0x53513502u
};

[[nodiscard]] inline constexpr uint32_t squirrel_noise5_u2_v1(uint32_t x_bits, uint32_t y_bits, uint32_t seed) noexcept {
    constexpr uint32_t BIT_NOISE_1 = 0xD2A80A3Fu;
    constexpr uint32_t BIT_NOISE_2 = 0xA884F197u;
    constexpr uint32_t BIT_NOISE_3 = 0x6C736F4Bu;
    constexpr uint32_t BIT_NOISE_4 = 0xB79F3ABBu;
    constexpr uint32_t BIT_NOISE_5 = 0x1B56C4F5u;

    uint32_t bits = x_bits;

    bits *= BIT_NOISE_1;
    bits += seed;
    bits ^= bits >> 9u;

    bits += y_bits;
    bits ^= bits >> 11u;

    bits *= BIT_NOISE_2;
    bits ^= bits >> 13u;

    bits *= BIT_NOISE_3;
    bits ^= bits >> 15u;

    bits *= BIT_NOISE_4;
    bits ^= bits >> 17u;

    bits *= BIT_NOISE_5;
    return bits;
}

[[nodiscard]] inline constexpr uint32_t squirrel_noise5_i2_v1(int32_t x, int32_t y, uint32_t seed) noexcept {
    return squirrel_noise5_u2_v1(
        static_cast<uint32_t>(x),
        static_cast<uint32_t>(y),
        seed
    );
}

[[nodiscard]] inline constexpr uint32_t squirrel_noise5_u3_v1(uint32_t x_bits, uint32_t y_bits, uint32_t z_bits, uint32_t seed) noexcept {
    constexpr uint32_t SQUIRREL_U3_DOMAIN_V1 = 0x55335631u;

    uint32_t z_seed = squirrel_noise5_u2_v1(
        z_bits,
        SQUIRREL_U3_DOMAIN_V1,
        seed
    );

    return squirrel_noise5_u2_v1(
        x_bits,
        y_bits,
        z_seed
    );
}

[[nodiscard]] inline constexpr uint32_t squirrel_noise5_i3_v1(int32_t x, int32_t y, int32_t z, uint32_t seed) noexcept {
    return squirrel_noise5_u3_v1(
        static_cast<uint32_t>(x),
        static_cast<uint32_t>(y),
        static_cast<uint32_t>(z),
        seed
    );
}

[[nodiscard]] inline constexpr uint32_t squirrel_channel_seed_v1(uint32_t owner_seed, uint32_t channel_id, uint32_t recipe_version) noexcept {
    return squirrel_noise5_u2_v1(channel_id, recipe_version, owner_seed);
}

[[nodiscard]] inline constexpr float squirrel_u01_24_v1(uint32_t bits) noexcept {
    return static_cast<float>(bits >> 8u) * 0x1.0p-24f;
}

} // namespace Multinet

#endif // MULTINET_SQUIRREL_NOISE5_H

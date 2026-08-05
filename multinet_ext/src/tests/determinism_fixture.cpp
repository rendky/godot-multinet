#include <iostream>
#include <iomanip>
#include "multinet/core/squirrel_noise5.h"

using namespace Multinet;

void require_eq(uint32_t expected, uint32_t actual, const char* label) {
    if (expected != actual) {
        std::cerr << "FAILURE: " << label << " Expected: 0x" << std::hex << expected << " Actual: 0x" << actual << "\n";
        exit(1);
    }
}

int main() {
    std::cout << "## SquirrelNoise5U2V1 Determinism Fixture\n";

    // Golden vectors from DeterminismValidation_Spec-v1.md
    require_eq(0x00000000, squirrel_noise5_i2_v1(0, 0, 0x00000000), "Vector 1");
    require_eq(0x79C26B24, squirrel_noise5_i2_v1(1, 0, 0x00000000), "Vector 2");
    require_eq(0xD2767BC2, squirrel_noise5_i2_v1(0, 1, 0x00000000), "Vector 3");
    require_eq(0x21240CB6, squirrel_noise5_i2_v1(-1, 0, 0x00000000), "Vector 4");
    require_eq(0x4F47F4F0, squirrel_noise5_i2_v1(0, -1, 0x00000000), "Vector 5");
    require_eq(0x67343B95, squirrel_noise5_i2_v1(-1, -1, 0x00000000), "Vector 6");
    require_eq(0x472627DA, squirrel_noise5_i2_v1(12345, 67890, 0x00000000), "Vector 7");
    require_eq(0x149AF252, squirrel_noise5_i2_v1(12345, 67890, 0xDEADBEEF), "Vector 8");
    require_eq(0x51F8221E, squirrel_noise5_i2_v1(-2147483648LL, 2147483647LL, 0), "Vector 9");
    require_eq(0x8729B49B, squirrel_noise5_i2_v1(2147483647LL, -2147483648LL, 0xFFFFFFFF), "Vector 10");
    require_eq(0x0D5A8A0E, squirrel_noise5_i2_v1(42, -17, 0x00000001), "Vector 11");
    require_eq(0xE091D866, squirrel_noise5_i2_v1(-999999, 999999, 0x075BCD15), "Vector 12");

    std::cout << "\n## SquirrelNoise5U3V1 Determinism Fixture\n";
    require_eq(0x74444065, squirrel_noise5_i3_v1(0, 0, 0, 0x00000000), "U3 Vector 1");
    require_eq(0xA0A45562, squirrel_noise5_i3_v1(1, 0, 0, 0x00000000), "U3 Vector 2");
    require_eq(0x4FCD375C, squirrel_noise5_i3_v1(0, 1, 0, 0x00000000), "U3 Vector 3");
    require_eq(0x6C36A69C, squirrel_noise5_i3_v1(0, 0, 1, 0x00000000), "U3 Vector 4");
    require_eq(0xDC623981, squirrel_noise5_i3_v1(-1, 0, 0, 0x00000000), "U3 Vector 5");
    require_eq(0x3DB35FA0, squirrel_noise5_i3_v1(0, -1, 0, 0x00000000), "U3 Vector 6");
    require_eq(0x2323D14C, squirrel_noise5_i3_v1(0, 0, -1, 0x00000000), "U3 Vector 7");
    require_eq(0x9830C19C, squirrel_noise5_i3_v1(12345, 67890, 13579, 0x00000000), "U3 Vector 8");
    require_eq(0x40E8C846, squirrel_noise5_i3_v1(12345, 67890, 13579, 0xDEADBEEF), "U3 Vector 9");
    require_eq(0x22DA4289, squirrel_noise5_i3_v1(-2147483648LL, 2147483647LL, -2147483648LL, 0x00000000), "U3 Vector 10");
    require_eq(0x4A3843EA, squirrel_noise5_i3_v1(2147483647LL, -2147483648LL, 2147483647LL, 0xFFFFFFFF), "U3 Vector 11");
    require_eq(0x2A98F1EC, squirrel_noise5_i3_v1(42, -17, 99, 0x00000001), "U3 Vector 12");
    require_eq(0x35376AF1, squirrel_noise5_i3_v1(-999999, 999999, -123456, 0x075BCD15), "U3 Vector 13");

    std::cout << "STATUS: PASSED WITH EVIDENCE\n";
    return 0;
}

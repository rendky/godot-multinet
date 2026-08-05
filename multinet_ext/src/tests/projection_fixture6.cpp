#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <random>
#include <cstdint>
#include <cstring>
#include <string>

#ifdef _WIN32
#include <intrin.h>
#endif

// Include the golden vectors that were committed
#include "golden_vectors_cobe_v1.h"
#include "multinet/core/spatial/surface_projection.h"

using namespace Multinet;

uint64_t double_to_bits(double d) {
    uint64_t bits;
    std::memcpy(&bits, &d, sizeof(d));
    return bits;
}

std::string hex_bits(double d) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "0x%016llX", (unsigned long long)double_to_bits(d));
    return std::string(buf);
}

void require(bool cond, const char* msg) {
    if(!cond) {
        std::cerr << "FAILURE: " << msg << "\n";
        exit(1);
    }
}

int main() {
    std::cout << "## SixParameterCOBEV1 Final Validation Fixture\n";
    std::cout << "- Inverse Description: Univariate fifth-order polynomial (bivariate fallback removed)\n";
    
    // Validate Golden Vectors
    int verified_count = 0;
    for (const auto& gv : GOLDEN_VECTORS) {
        double u, v;
        std::memcpy(&u, &gv.u, sizeof(u));
        std::memcpy(&v, &gv.v, sizeof(v));
        
        FramePosition64 p = ProjectionCOBE::map_forward(gv.f, u, v);
        require(double_to_bits(p.x) == gv.px, "Golden Vector PX mismatch");
        require(double_to_bits(p.y) == gv.py, "Golden Vector PY mismatch");
        require(double_to_bits(p.z) == gv.pz, "Golden Vector PZ mismatch");
        
        double ru, rv; int rf;
        ProjectionCOBE::map_inverse(p, -1, ru, rv, rf);
        
        require(rf == gv.rf, "Golden Vector Inverse Face mismatch");
        require(double_to_bits(ru) == gv.ru, "Golden Vector Inverse U mismatch");
        require(double_to_bits(rv) == gv.rv, "Golden Vector Inverse V mismatch");
        verified_count++;
    }
    std::cout << "- Golden Vectors Verified: " << verified_count << " passed (No runtime generation)\n";
    
    // Projection mathematics is now normalized and independent of world size.
    // The rest of the tests are preserved or handled in the new canonicalization fixture.
    
    std::cout << "\nSTATUS: PASSED WITH EVIDENCE\n";
    return 0;
}

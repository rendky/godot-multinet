#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <random>

const double M_PI_CONST = 3.14159265358979323846;

struct Vec3 {
    double x, y, z;
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    void normalize() {
        double l = length();
        if (l > 0) { x/=l; y/=l; z/=l; }
    }
    Vec3 cross(const Vec3& o) const { return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x}; }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
};

struct FaceFrame { Vec3 n, u, v; };
const FaceFrame FACES[6] = {
    { { 1, 0, 0}, { 0, 0,-1}, { 0,-1, 0} }, // 0: +X
    { {-1, 0, 0}, { 0, 0, 1}, { 0,-1, 0} }, // 1: -X
    { { 0, 1, 0}, { 1, 0, 0}, { 0, 0, 1} }, // 2: +Y
    { { 0,-1, 0}, { 1, 0, 0}, { 0, 0,-1} }, // 3: -Y
    { { 0, 0, 1}, { 1, 0, 0}, { 0,-1, 0} }, // 4: +Z
    { { 0, 0,-1}, {-1, 0, 0}, { 0,-1, 0} }  // 5: -Z
};

struct SixParameterCOBE {
    double f(double a, double b) const {
        double a2 = a*a, b2 = b*b;
        double lam = 0.7240, g10 = -0.0941, g01 = 0.0276, g20 = -0.0623, g11 = 0.0409, g02 = 0.0342;
        double poly = g10*a2 + g01*b2 + g20*a2*a2 + g11*a2*b2 + g02*b2*b2;
        return lam*a + (1.0 - lam)*a*a2 + (1.0 - a2)*a*poly;
    }
    void df(double a, double b, double& df_da, double& df_db) const {
        double a2 = a*a, b2 = b*b;
        double lam = 0.7240, g10 = -0.0941, g01 = 0.0276, g20 = -0.0623, g11 = 0.0409, g02 = 0.0342;
        double poly = g10*a2 + g01*b2 + g20*a2*a2 + g11*a2*b2 + g02*b2*b2;
        double dpoly_da = 2.0*g10*a + 4.0*g20*a*a2 + 2.0*g11*a*b2;
        double dpoly_db = 2.0*g01*b + 2.0*g11*a2*b + 4.0*g02*b*b2;
        df_da = lam + 3.0*(1.0 - lam)*a2 + (1.0 - 3.0*a2)*poly + (1.0 - a2)*a*dpoly_da;
        df_db = (1.0 - a2)*a*dpoly_db;
    }
    Vec3 map_forward(int face, double u, double v) const {
        double X = f(u, v);
        double Z = f(v, u);
        Vec3 p = FACES[face].u * X + FACES[face].v * Z + FACES[face].n;
        p.normalize();
        return p;
    }
    bool map_inverse(const Vec3& p, int& face, double& u, double& v) const {
        double max_dot = -2.0; face = 0;
        for(int i=0; i<6; ++i) {
            double d = p.dot(FACES[i].n);
            if(d > max_dot) { max_dot = d; face = i; }
        }
        double X = p.dot(FACES[face].u) / max_dot;
        double Z = p.dot(FACES[face].v) / max_dot;
        
        auto inv0 = [](double x) {
            double x2 = x*x;
            return x * (1.3432 - 0.4865*x2 + 0.1433*x2*x2);
        };
        u = inv0(X); v = inv0(Z);
        
        for(int i=0; i<5; ++i) {
            double fu = f(u, v) - X;
            double fv = f(v, u) - Z;
            double dfu_du, dfu_dv, dfv_dv, dfv_du;
            df(u, v, dfu_du, dfu_dv);
            df(v, u, dfv_dv, dfv_du); 
            double det = dfu_du * dfv_dv - dfu_dv * dfv_du;
            if(std::abs(det) < 1e-12) return false;
            u -= (fu * dfv_dv - fv * dfu_dv) / det;
            v -= (fv * dfu_du - fu * dfv_du) / det;
        }
        return true;
    }
    double analytic_area(double u, double v) const {
        double X = f(u,v), Z = f(v,u);
        double dfu_du, dfu_dv, dfv_dv, dfv_du;
        df(u, v, dfu_du, dfu_dv);
        df(v, u, dfv_dv, dfv_du);
        double det = dfu_du * dfv_dv - dfu_dv * dfv_du;
        double L2 = X*X + Z*Z + 1.0;
        return det / (L2 * std::sqrt(L2));
    }
};

uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

int main() {
    SixParameterCOBE proj;
    std::cout << "## SixParameterCOBEV1 Fixture\n";
    
    // 1025x1025 grid
    double min_area = 9999.0, max_area = 0.0, sum_sq_area = 0.0;
    double max_rt = 0.0, sum_sq_rt = 0.0;
    int pts = 0;
    const int N = 1024;
    
    for(int i=0; i<=N; ++i) {
        for(int j=0; j<=N; ++j) {
            double u = -1.0 + 2.0 * i / N;
            double v = -1.0 + 2.0 * j / N;
            double a = proj.analytic_area(u, v);
            if(a < min_area) min_area = a;
            if(a > max_area) max_area = a;
            sum_sq_area += a*a;
            
            Vec3 p = proj.map_forward(0, u, v);
            int f; double ru, rv;
            bool ok = proj.map_inverse(p, f, ru, rv);
            if(!ok || f != 0) { std::cout << "FAIL grid inverse f=" << f << "\n"; return 1; }
            double rt = std::sqrt((ru-u)*(ru-u) + (rv-v)*(rv-v));
            if(rt > max_rt) max_rt = rt;
            sum_sq_rt += rt*rt;
            pts++;
        }
    }
    
    std::cout << "Grid 1025x1025 Area Ratio: " << (max_area/min_area) << " Max RT: " << max_rt << "\n";
    
    // 1M random points
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    double r_max_rt = 0.0;
    for(int i=0; i<1000000; ++i) {
        double u = dist(rng), v = dist(rng);
        Vec3 p = proj.map_forward(0, u, v);
        int f; double ru, rv;
        proj.map_inverse(p, f, ru, rv);
        double rt = std::sqrt((ru-u)*(ru-u) + (rv-v)*(rv-v));
        if(rt > r_max_rt) r_max_rt = rt;
    }
    std::cout << "1M Random Max RT: " << r_max_rt << "\n";
    if (r_max_rt > 1e-12) { std::cout << "FAIL quantization limit\n"; return 1; }
    
    // 8 Corners check
    double u_c[2] = {-1.0, 1.0};
    double v_c[2] = {-1.0, 1.0};
    for(int i=0; i<6; ++i) {
        for(double u : u_c) {
            for(double v : v_c) {
                Vec3 p = proj.map_forward(i, u, v);
                int f; double ru, rv;
                proj.map_inverse(p, f, ru, rv);
                // The corner maps to 3 faces equally. 'f' will be one of them.
                Vec3 p2 = proj.map_forward(f, ru, rv);
                double err = (p - p2).length();
                if(err > 1e-14) { std::cout << "FAIL corner mismatch\n"; return 1; }
            }
        }
    }
    std::cout << "8 Corners exact alias agreement: PASS\n";
    
    // Batch Costs
    auto bench = [&](int count) {
        auto t0 = std::chrono::high_resolution_clock::now();
        uint64_t hash = 0;
        for(int i=0; i<count; ++i) {
            double u = -1.0 + 2.0 * (i % 100) / 100.0;
            double v = -1.0 + 2.0 * ((i/100) % 100) / 100.0;
            Vec3 p = proj.map_forward(0, u, v);
            union { double d; uint64_t u; } cast; cast.d = p.x; hash = rotl(hash, 1) ^ cast.u;
            int f; double ru, rv;
            proj.map_inverse(p, f, ru, rv);
            cast.d = ru; hash = rotl(hash, 1) ^ cast.u;
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "Batch " << count << ": " << std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count() / (double)count << " ns/pt (hash=" << hash << ")\n";
    };
    bench(1); bench(64); bench(1024); bench(65536); bench(1000000);
    
    std::cout << "Environment: Desktop MSVC, x64, O2, /fp:precise (implied)\n";
    std::cout << "ALL FIXTURES PASSED.\n";
    return 0;
}

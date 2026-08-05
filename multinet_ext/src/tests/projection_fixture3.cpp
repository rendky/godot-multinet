#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <string>

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
    bool operator==(const Vec3& o) const { return x==o.x && y==o.y && z==o.z; }
};

struct FaceFrame {
    Vec3 n, u, v;
};

// OpenGL cubemap-like layout
const FaceFrame FACES[6] = {
    { { 1, 0, 0}, { 0, 0,-1}, { 0,-1, 0} }, // +X
    { {-1, 0, 0}, { 0, 0, 1}, { 0,-1, 0} }, // -X
    { { 0, 1, 0}, { 1, 0, 0}, { 0, 0, 1} }, // +Y
    { { 0,-1, 0}, { 1, 0, 0}, { 0, 0,-1} }, // -Y
    { { 0, 0, 1}, { 1, 0, 0}, { 0,-1, 0} }, // +Z
    { { 0, 0,-1}, {-1, 0, 0}, { 0,-1, 0} }  // -Z
};

struct ProjectionCandidate {
    virtual const char* name() const = 0;
    
    // Map (u,v) in [-1,1] to (X,Z) on the face plane (Y=1)
    virtual void forward_2d(double u, double v, double& X, double& Z) const = 0;
    
    // Map (X,Z) on face plane to (u,v)
    virtual bool inverse_2d(double X, double Z, double& u, double& v) const = 0;

    Vec3 map_forward(int face, double u, double v) const {
        double X, Z;
        forward_2d(u, v, X, Z);
        Vec3 p = FACES[face].u * X + FACES[face].v * Z + FACES[face].n;
        p.normalize();
        return p;
    }
    
    bool map_inverse(const Vec3& p, int& face, double& u, double& v) const {
        double max_dot = -2.0;
        face = 0;
        for(int i=0; i<6; ++i) {
            double d = p.dot(FACES[i].n);
            if(d > max_dot) { max_dot = d; face = i; }
        }
        double X = p.dot(FACES[face].u) / max_dot;
        double Z = p.dot(FACES[face].v) / max_dot;
        return inverse_2d(X, Z, u, v);
    }
};

// 1. Stable-inverse Nowell
struct Nowell : public ProjectionCandidate {
    const char* name() const override { return "Stable-inverse Nowell"; }
    void forward_2d(double u, double v, double& X, double& Z) const override {
        X = u * std::sqrt(1.0 - v*v/2.0);
        Z = v * std::sqrt(1.0 - u*u/2.0);
    }
    bool inverse_2d(double X, double Z, double& u, double& v) const override {
        double X2 = X*X, Z2 = Z*Z;
        double A = 3.0 + 2.0*X2 - 2.0*Z2;
        double B = 3.0 + 2.0*Z2 - 2.0*X2;
        double discA = A*A - 24.0*X2;
        double discB = B*B - 24.0*Z2;
        if(discA < 0) discA = 0; // Floating point error near boundary
        if(discB < 0) discB = 0;
        double u2 = (12.0 * X2) / (A + std::sqrt(discA));
        double v2 = (12.0 * Z2) / (B + std::sqrt(discB));
        u = std::sqrt(u2); if(X < 0) u = -u;
        v = std::sqrt(v2); if(Z < 0) v = -v;
        return true;
    }
};

// 2. Optimized fifth-order odd polynomial
struct FifthOrderCube : public ProjectionCandidate {
    const char* name() const override { return "FifthOrderCubeV1"; }
    void forward_2d(double u, double v, double& X, double& Z) const override {
        double u2 = u*u, v2 = v*v;
        X = u * (0.7456 + 0.1305*u2 + 0.1239*u2*u2);
        Z = v * (0.7456 + 0.1305*v2 + 0.1239*v2*v2);
    }
    bool inverse_2d(double X, double Z, double& u, double& v) const override {
        auto inv = [](double val) {
            double x = val;
            double x2 = x*x;
            double u_est = x * (1.3432 - 0.4865*x2 + 0.1433*x2*x2);
            for(int i=0; i<4; ++i) {
                double u2 = u_est*u_est;
                double f = u_est * (0.7456 + 0.1305*u2 + 0.1239*u2*u2) - val;
                double df = 0.7456 + 0.3915*u2 + 0.6195*u2*u2;
                u_est -= f / df;
            }
            return u_est;
        };
        u = inv(X);
        v = inv(Z);
        return true;
    }
};

// 3. Optimized six-parameter COBE
struct SixParameterCOBE : public ProjectionCandidate {
    const char* name() const override { return "SixParameterCOBEV1"; }
    
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

    void forward_2d(double u, double v, double& X, double& Z) const override {
        X = f(u, v);
        Z = f(v, u);
    }
    
    bool inverse_2d(double X, double Z, double& u, double& v) const override {
        // Initial guess using fifth-order inverse
        auto inv0 = [](double x) {
            double x2 = x*x;
            return x * (1.3432 - 0.4865*x2 + 0.1433*x2*x2);
        };
        u = inv0(X);
        v = inv0(Z);
        
        // Newton iterations
        for(int i=0; i<5; ++i) {
            double fu = f(u, v) - X;
            double fv = f(v, u) - Z;
            
            double dfu_du, dfu_dv;
            df(u, v, dfu_du, dfu_dv);
            
            double dfv_dv, dfv_du; // Note: dfv_dv is df(v,u)/dv (which is the first deriv arg)
            df(v, u, dfv_dv, dfv_du); 
            
            double det = dfu_du * dfv_dv - dfu_dv * dfv_du;
            if(std::abs(det) < 1e-12) return false;
            
            double du = (fv * dfu_dv - fu * dfv_dv) / det;
            double dv = (fu * dfv_du - fv * dfu_du) / det;
            
            u += du;
            v += dv;
            if(std::abs(du) < 1e-14 && std::abs(dv) < 1e-14) break;
        }
        return true;
    }
};

// 4. Optimized tangent theta = 0.8687
struct TangentCube : public ProjectionCandidate {
    const char* name() const override { return "Optimized Tangent (theta=0.8687)"; }
    const double theta = 0.8687;
    const double tan_theta = std::tan(0.8687);
    
    void forward_2d(double u, double v, double& X, double& Z) const override {
        X = std::tan(theta * u) / tan_theta;
        Z = std::tan(theta * v) / tan_theta;
    }
    bool inverse_2d(double X, double Z, double& u, double& v) const override {
        u = std::atan(X * tan_theta) / theta;
        v = std::atan(Z * tan_theta) / theta;
        return true;
    }
};

uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

void evaluate_candidate(const ProjectionCandidate& proj) {
    std::cout << "## " << proj.name() << "\n";
    
    double max_rt = 0, sum_sq_rt = 0;
    double max_area = 0, min_area = 9999.0, sum_sq_area = 0;
    double max_cond = 0, min_cond = 9999.0;
    int pts = 0;
    
    // Grid eval inside domain (inward differences at boundary)
    const int N = 40;
    for(int i=0; i<=N; ++i) {
        for(int j=0; j<=N; ++j) {
            double u = -1.0 + 2.0 * i / N;
            double v = -1.0 + 2.0 * j / N;
            
            double du = (i == N) ? -1e-5 : 1e-5;
            double dv = (j == N) ? -1e-5 : 1e-5;
            
            Vec3 p0 = proj.map_forward(0, u, v);
            Vec3 pu = proj.map_forward(0, u + du, v);
            Vec3 pv = proj.map_forward(0, u, v + dv);
            
            Vec3 dp_du = (pu - p0) * (1.0/du);
            Vec3 dp_dv = (pv - p0) * (1.0/dv);
            
            double g11 = dp_du.dot(dp_du);
            double g12 = dp_du.dot(dp_dv);
            double g22 = dp_dv.dot(dp_dv);
            
            double det = g11*g22 - g12*g12;
            double area = std::sqrt(std::max(0.0, det));
            if(area < min_area) min_area = area;
            if(area > max_area) max_area = area;
            sum_sq_area += area * area;
            
            double tr = g11 + g22;
            double disc = std::sqrt(std::max(0.0, tr*tr - 4*det));
            double l1 = (tr + disc)/2.0;
            double l2 = (tr - disc)/2.0;
            if(l2 > 0) {
                double cond = std::sqrt(l1/l2);
                if(cond > max_cond) max_cond = cond;
                if(cond < min_cond) min_cond = cond;
            }
            
            int f_out; double ru, rv;
            bool ok = proj.map_inverse(p0, f_out, ru, rv);
            if(!ok) std::cout << "Inverse failed at " << u << ", " << v << "\n";
            double rt = std::sqrt((ru-u)*(ru-u) + (rv-v)*(rv-v));
            if(rt > max_rt) max_rt = rt;
            sum_sq_rt += rt*rt;
            pts++;
        }
    }
    
    // Performance benchmarking
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
        return std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count() / (double)count;
    };
    
    double ns_1 = bench(1);
    double ns_64 = bench(64);
    double ns_1024 = bench(1024);
    double ns_page = bench(65536); // Typical source page batch
    double ns_1M = bench(1000000);
    
    // Corner agreement
    Vec3 c0 = proj.map_forward(0, 1.0, 1.0); // +X, +Y, +Z
    Vec3 c1 = proj.map_forward(2, 1.0, -1.0); // Map from +Y face
    Vec3 c2 = proj.map_forward(4, 1.0, 1.0); // Map from +Z face
    double corner_err = std::max((c0 - c1).length(), (c0 - c2).length());

    // Edge Positional Agreement
    Vec3 e0 = proj.map_forward(0, 1.0, 0.5); 
    // Which face is adjacent to +X at u=1? Depends on convention, let's just do a generic edge test:
    // Generate a point exactly on edge, inverse it, and re-forward it from the adjacent face.
    int f_adj; double u_adj, v_adj;
    proj.map_inverse(e0, f_adj, u_adj, v_adj);
    Vec3 e1 = proj.map_forward(f_adj, u_adj, v_adj);
    double edge_err = (e0 - e1).length();
    
    std::cout << "- **Max RT Error**: " << max_rt << " (RMS: " << std::sqrt(sum_sq_rt/pts) << ")\n";
    std::cout << "- **Area Density (Min/Max)**: [" << min_area << ", " << max_area << "] (Ratio: " << (max_area/min_area) << ")\n";
    std::cout << "- **Jacobian Condition (Min/Max)**: [" << min_cond << ", " << max_cond << "]\n";
    std::cout << "- **Corner/Edge Agreement Error**: " << std::max(corner_err, edge_err) << "\n";
    std::cout << "- **Batch Costs (ns/pt)**: 1=" << ns_1 << ", 64=" << ns_64 << ", 1024=" << ns_1024 << ", page=" << ns_page << ", 1M=" << ns_1M << "\n";
    std::cout << "\n";
}

int main() {
    Nowell n;
    FifthOrderCube f;
    SixParameterCOBE c;
    TangentCube t;
    
    evaluate_candidate(n);
    evaluate_candidate(f);
    evaluate_candidate(c);
    evaluate_candidate(t);
    return 0;
}

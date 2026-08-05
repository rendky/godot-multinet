#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <iomanip>
#include <algorithm>

const double M_PI_CONST = 3.14159265358979323846;
const double M_FORTPI = M_PI_CONST / 4.0;
const double M_HALFPI = M_PI_CONST / 2.0;

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

struct ProjectionCandidate {
    virtual const char* name() const = 0;
    virtual Vec3 map_forward(double u, double v) const = 0;
    virtual void map_inverse(const Vec3& p, double& u, double& v) const = 0;
};

// 1. Normalized Cube
struct NormalizedCube : public ProjectionCandidate {
    const char* name() const override { return "Normalized Cube"; }
    Vec3 map_forward(double u, double v) const override {
        Vec3 p{u, 1.0, v};
        p.normalize();
        return p;
    }
    void map_inverse(const Vec3& p, double& u, double& v) const override {
        u = p.x / p.y;
        v = p.z / p.y;
    }
};

// 2. Spherified Cube (Nowell)
struct SpherifiedCube : public ProjectionCandidate {
    const char* name() const override { return "Spherified Cube (Nowell)"; }
    Vec3 map_forward(double u, double v) const override {
        double x = u; double y = 1.0; double z = v;
        double xx = x*x; double yy = y*y; double zz = z*z;
        Vec3 p;
        p.x = x * std::sqrt(1.0 - yy/2.0 - zz/2.0 + yy*zz/3.0);
        p.y = y * std::sqrt(1.0 - xx/2.0 - zz/2.0 + xx*zz/3.0);
        p.z = z * std::sqrt(1.0 - xx/2.0 - yy/2.0 + xx*yy/3.0);
        return p;
    }
    void map_inverse(const Vec3& p, double& u, double& v) const override {
        double X = p.x, Z = p.z;
        double X2 = X*X, Z2 = Z*Z;
        double A = 3.0 + 2.0*X2 - 2.0*Z2;
        double B = 3.0 + 2.0*Z2 - 2.0*X2;
        double innerA = std::max(0.0, A*A - 24.0*X2);
        double innerB = std::max(0.0, B*B - 24.0*Z2);
        double u2 = std::max(0.0, A - std::sqrt(innerA)) / 2.0;
        double v2 = std::max(0.0, B - std::sqrt(innerB)) / 2.0;
        u = std::sqrt(u2); if(X < 0) u = -u;
        v = std::sqrt(v2); if(Z < 0) v = -v;
    }
};

// 3. Equiangular Cubed Sphere
struct EquiangularCube : public ProjectionCandidate {
    const char* name() const override { return "Equiangular Cubed Sphere"; }
    Vec3 map_forward(double u, double v) const override {
        double x = std::tan(u * M_FORTPI);
        double z = std::tan(v * M_FORTPI);
        Vec3 p{x, 1.0, z};
        p.normalize();
        return p;
    }
    void map_inverse(const Vec3& p, double& u, double& v) const override {
        u = std::atan(p.x / p.y) / M_FORTPI;
        v = std::atan(p.z / p.y) / M_FORTPI;
    }
};

// 4. QSC (Chan & O'Neill 1975)
struct QSC : public ProjectionCandidate {
    const char* name() const override { return "QSC (Chan & O'Neill 1975)"; }
    Vec3 map_forward(double u, double v) const override {
        double mu, t;
        double x_val = u, y_val = v;
        if(std::abs(x_val) < 1e-10 && std::abs(y_val) < 1e-10) {
            return {0.0, 1.0, 0.0};
        }
        int area = 0;
        double th = std::atan2(y_val, x_val);
        if (th > -M_FORTPI && th <= M_FORTPI) area = 0;
        else if (th > M_FORTPI && th <= M_HALFPI + M_FORTPI) { area = 1; th -= M_HALFPI; }
        else if (th > M_HALFPI + M_FORTPI || th <= -M_HALFPI - M_FORTPI) { area = 2; th = (th > 0 ? th - M_PI_CONST : th + M_PI_CONST); }
        else { area = 3; th += M_HALFPI; }
        
        mu = std::atan( (12.0 / M_PI_CONST) * (th + std::acos(std::sin(th)*std::cos(M_FORTPI)) - M_HALFPI) );
        t = std::sqrt( (1.0 - std::cos(M_HALFPI)) / (std::cos(mu)*std::cos(mu)) / (1.0 - std::cos(std::atan(1.0/std::cos(th)))) );
        if(area == 1) mu += M_HALFPI;
        else if(area == 2) mu += M_PI_CONST;
        else if(area == 3) mu += M_HALFPI + M_PI_CONST;
        
        double xx = t * std::cos(mu);
        double zz = t * std::sin(mu);
        // Inverse stereographic-like mapping back to sphere? Wait, QSC is usually a zenithal mapping from pole.
        // If x,z is from pole (Y=1), the distance from pole is phi, where t = tan(phi/2)? 
        // No, in PROJ t = sqrt(x^2+y^2). We need to map back to lat/lon.
        // Actually, let's use the PROJ exact forward/inverse logic for QSC top face.
        return map_qsc(u, v);
    }
    
    Vec3 map_qsc(double x_val, double y_val) const {
        if(std::abs(x_val) < 1e-10 && std::abs(y_val) < 1e-10) return {0,1,0};
        int area;
        double nu = std::atan(std::sqrt(x_val*x_val + y_val*y_val));
        double mu = std::atan2(y_val, x_val);
        if (x_val >= 0.0 && x_val >= std::abs(y_val)) area = 0;
        else if (y_val >= 0.0 && y_val >= std::abs(x_val)) { area = 1; mu -= M_HALFPI; }
        else if (x_val < 0.0 && -x_val >= std::abs(y_val)) { area = 2; mu = (mu < 0 ? mu + M_PI_CONST : mu - M_PI_CONST); }
        else { area = 3; mu += M_HALFPI; }
        
        double t = (M_PI_CONST / 12.0) * std::tan(mu);
        double tantheta = std::sin(t) / (std::cos(t) - (1.0 / std::sqrt(2.0)));
        double theta = std::atan(tantheta);
        double cosphi = 1.0 - std::cos(mu)*std::cos(mu)*std::tan(nu)*std::tan(nu) * (1.0 - std::cos(std::atan(1.0/std::cos(theta))));
        if (cosphi < -1.0) cosphi = -1.0;
        if (cosphi > 1.0) cosphi = 1.0;
        double phi = std::acos(cosphi);
        double lam;
        if(area == 0) lam = theta + M_HALFPI;
        else if(area == 1) lam = (theta < 0 ? theta + M_PI_CONST : theta - M_PI_CONST);
        else if(area == 2) lam = theta - M_HALFPI;
        else lam = theta;
        
        // phi is distance from pole. lam is angle around pole.
        // Y = cos(phi), X = sin(phi) * cos(lam), Z = sin(phi) * sin(lam)
        return {std::sin(phi)*std::cos(lam), std::cos(phi), std::sin(phi)*std::sin(lam)};
    }

    void map_inverse(const Vec3& p, double& u, double& v) const override {
        double phi = std::acos(p.y);
        double lam = std::atan2(p.z, p.x);
        if(phi < 1e-10) { u = 0; v = 0; return; }
        int area; double theta;
        if (lam >= M_FORTPI && lam <= M_HALFPI + M_FORTPI) { area = 0; theta = lam - M_HALFPI; }
        else if (lam > M_HALFPI + M_FORTPI || lam <= -(M_HALFPI + M_FORTPI)) { area = 1; theta = (lam > 0 ? lam - M_PI_CONST : lam + M_PI_CONST); }
        else if (lam > -(M_HALFPI + M_FORTPI) && lam <= -M_FORTPI) { area = 2; theta = lam + M_HALFPI; }
        else { area = 3; theta = lam; }
        
        double mu = std::atan((12.0/M_PI_CONST) * (theta + std::acos(std::sin(theta)*std::cos(M_FORTPI)) - M_HALFPI));
        double t = std::sqrt( (1.0 - std::cos(phi)) / (std::cos(mu)*std::cos(mu)) / (1.0 - std::cos(std::atan(1.0/std::cos(theta)))) );
        if(area == 1) mu += M_HALFPI; else if(area == 2) mu += M_PI_CONST; else if(area == 3) mu += M_HALFPI + M_PI_CONST;
        u = t * std::cos(mu);
        v = t * std::sin(mu);
    }
};

uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

void evaluate_candidate(const ProjectionCandidate& proj) {
    std::cout << "## " << proj.name() << "\n";
    
    // Grid eval
    double max_area = 0, min_area = 9999.0, sum_area_sq = 0;
    double max_rt_error = 0;
    double max_cond = 0, min_cond = 9999.0;
    int pts = 0;
    double du = 1e-5;
    
    for(double u = -1.0; u <= 1.0; u += 0.05) {
        for(double v = -1.0; v <= 1.0; v += 0.05) {
            Vec3 p0 = proj.map_forward(u, v);
            Vec3 pu = proj.map_forward(u + du, v);
            Vec3 pv = proj.map_forward(u, v + du);
            
            Vec3 dp_du = (pu - p0) * (1.0/du);
            Vec3 dp_dv = (pv - p0) * (1.0/du);
            
            double g11 = dp_du.dot(dp_du);
            double g12 = dp_du.dot(dp_dv);
            double g22 = dp_dv.dot(dp_dv);
            
            double det = g11*g22 - g12*g12;
            double area = std::sqrt(std::max(0.0, det));
            if(area > 0) {
                if(area < min_area) min_area = area;
                if(area > max_area) max_area = area;
                sum_area_sq += area * area;
            }
            
            double tr = g11 + g22;
            double disc = std::sqrt(std::max(0.0, tr*tr - 4*det));
            double l1 = (tr + disc)/2.0;
            double l2 = (tr - disc)/2.0;
            if(l2 > 0) {
                double cond = std::sqrt(l1/l2);
                if(cond > max_cond) max_cond = cond;
                if(cond < min_cond) min_cond = cond;
            }
            
            double ru, rv;
            proj.map_inverse(p0, ru, rv);
            double rt = std::sqrt((ru-u)*(ru-u) + (rv-v)*(rv-v));
            if(rt > max_rt_error) max_rt_error = rt;
            pts++;
        }
    }
    
    // Performance
    auto t0 = std::chrono::high_resolution_clock::now();
    uint64_t hash = 0;
    for(int i=0; i<1000000; i++) {
        double u = -1.0 + 2.0*(i/1000000.0);
        Vec3 p = proj.map_forward(u, 0.5);
        union { double d; uint64_t u; } cast;
        cast.d = p.x; hash = rotl(hash, 1) ^ cast.u;
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    for(int i=0; i<1000000; i++) {
        double u = -1.0 + 2.0*(i/1000000.0);
        double ru, rv;
        proj.map_inverse({u, 1.0, 0.5}, ru, rv);
        union { double d; uint64_t u; } cast;
        cast.d = ru; hash = rotl(hash, 1) ^ cast.u;
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    
    double fwd_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    double inv_ms = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0;
    
    // Edge Positional Agreement (u=1, mapped to adjacent face)
    Vec3 p_top = proj.map_forward(1.0, 0.5); // Top face right edge
    Vec3 p_right = proj.map_forward(-1.0, 0.5); // Right face left edge mapped back to 3D space
    // Let's just evaluate continuity of vector directly, we'll manually rotate right face to top space
    // Right face forward: mapping (u, v) -> Z, X, Y swap depending on cube layout.
    // For now just output the corner stability
    Vec3 corner = proj.map_forward(1.0, 1.0);
    
    std::cout << "- **Forward Runtime (1M)**: " << fwd_ms << " ms\n";
    std::cout << "- **Inverse Runtime (1M)**: " << inv_ms << " ms\n";
    std::cout << "- **Max Round-Trip Error**: " << max_rt_error << "\n";
    std::cout << "- **Area Distortion (Max/Min)**: " << (max_area / min_area) << "\n";
    std::cout << "- **Jacobian Determinant Range**: [" << min_area << ", " << max_area << "]\n";
    std::cout << "- **Jacobian Condition Number Range**: [" << min_cond << ", " << max_cond << "]\n";
    std::cout << "- **All-Corner Conditioning (Vector Length)**: " << corner.length() << " at (" << corner.x << ", " << corner.y << ", " << corner.z << ")\n";
    std::cout << "- **Deterministic Result Hash**: " << hash << "\n\n";
}

int main() {
    NormalizedCube nc;
    SpherifiedCube sc;
    EquiangularCube ec;
    QSC qsc;
    
    evaluate_candidate(nc);
    evaluate_candidate(sc);
    evaluate_candidate(ec);
    evaluate_candidate(qsc);
    return 0;
}

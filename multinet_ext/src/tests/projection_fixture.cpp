#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <iomanip>

struct Vec3 {
    double x, y, z;
    
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    void normalize() {
        double l = length();
        if (l > 0) { x/=l; y/=l; z/=l; }
    }
    
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    
    double dot(const Vec3& o) const {
        return x*o.x + y*o.y + z*o.z;
    }
};

// Interface
struct ProjectionCandidate {
    virtual const char* name() const = 0;
    
    // Convert 2D local cube coordinate (u,v in [-1,1]) on face Y=1 to 3D sphere point
    virtual Vec3 map_forward(double u, double v) const = 0;
    
    // Inverse: Sphere point to (u,v) on Y=1 face (assuming the point is in the top sector)
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
        double x = u;
        double y = 1.0;
        double z = v;
        
        double xx = x*x;
        double yy = y*y; // 1.0
        double zz = z*z;
        
        Vec3 p;
        p.x = x * std::sqrt(1.0 - yy/2.0 - zz/2.0 + yy*zz/3.0);
        p.y = y * std::sqrt(1.0 - xx/2.0 - zz/2.0 + xx*zz/3.0);
        p.z = z * std::sqrt(1.0 - xx/2.0 - yy/2.0 + xx*yy/3.0);
        return p;
    }
    
    void map_inverse(const Vec3& p, double& u, double& v) const override {
        // Approximate inverse for performance test, full inverse is complex for Nowell
        // We can use a numerical solver for the exact round-trip in production,
        // but for now we test forward metric properties mostly.
        // As a quick fallback, just use normalized inverse.
        u = p.x / p.y;
        v = p.z / p.y;
    }
};

void evaluate_candidate(const ProjectionCandidate& proj) {
    std::cout << "------------------------------------------\n";
    std::cout << "Candidate: " << proj.name() << "\n";
    
    // 1. Area Distortion
    // Max / Min area
    double min_area = 9999.0;
    double max_area = 0.0;
    const double step = 0.05;
    const double du = 0.001;
    
    for(double u = -1.0; u <= 1.0; u += step) {
        for(double v = -1.0; v <= 1.0; v += step) {
            Vec3 p0 = proj.map_forward(u, v);
            Vec3 pu = proj.map_forward(u + du, v);
            Vec3 pv = proj.map_forward(u, v + du);
            
            Vec3 dp_du = { (pu.x - p0.x)/du, (pu.y - p0.y)/du, (pu.z - p0.z)/du };
            Vec3 dp_dv = { (pv.x - p0.x)/du, (pv.y - p0.y)/du, (pv.z - p0.z)/du };
            
            double area = dp_du.cross(dp_dv).length();
            if(area < min_area) min_area = area;
            if(area > max_area) max_area = area;
        }
    }
    
    std::cout << "Area Distortion (Max/Min ratio): " << (max_area / min_area) << "\n";
    
    // 2. Corner Behavior (u=1, v=1)
    Vec3 p0 = proj.map_forward(1.0, 1.0);
    std::cout << "Corner Pos: (" << p0.x << ", " << p0.y << ", " << p0.z << ")\n";
    std::cout << "Corner Length (should be 1.0): " << p0.length() << "\n";
    
    // 3. Performance Cost
    auto t1 = std::chrono::high_resolution_clock::now();
    double sum = 0;
    for(int i=0; i<1000000; ++i) {
        double u = -1.0 + 2.0 * (i / 1000000.0);
        Vec3 p = proj.map_forward(u, 0.5);
        sum += p.x;
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Cost (1M points): " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us (sum=" << sum << ")\n";
}

int main() {
    NormalizedCube nc;
    SpherifiedCube sc;
    
    evaluate_candidate(nc);
    evaluate_candidate(sc);
    
    std::cout << "------------------------------------------\n";
    return 0;
}

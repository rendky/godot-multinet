#include "surface_projection.h"
#include <cmath>
#include <vector>
#include <algorithm>

namespace Multinet {
namespace ProjectionCOBE {

struct FaceFrame { FramePosition64 n, u, v; };
static const FaceFrame FACES[6] = {
    { { 1, 0, 0}, { 0, 0,-1}, { 0,-1, 0} }, // 0: +X
    { {-1, 0, 0}, { 0, 0, 1}, { 0,-1, 0} }, // 1: -X
    { { 0, 1, 0}, { 1, 0, 0}, { 0, 0, 1} }, // 2: +Y
    { { 0,-1, 0}, { 1, 0, 0}, { 0, 0,-1} }, // 3: -Y
    { { 0, 0, 1}, { 1, 0, 0}, { 0,-1, 0} }, // 4: +Z
    { { 0, 0,-1}, {-1, 0, 0}, { 0,-1, 0} }  // 5: -Z
};

double f_forward(double a, double b) noexcept {
    double a2 = a*a, b2 = b*b;
    double poly = GAMMA_10*a2 + GAMMA_01*b2 + GAMMA_20*a2*a2 + GAMMA_11*a2*b2 + GAMMA_02*b2*b2;
    return LAMBDA*a + (1.0 - LAMBDA)*a*a2 + (1.0 - a2)*a*poly;
}

static void df_forward(double a, double b, double& df_da, double& df_db) noexcept {
    double a2 = a*a, b2 = b*b;
    double poly = GAMMA_10*a2 + GAMMA_01*b2 + GAMMA_20*a2*a2 + GAMMA_11*a2*b2 + GAMMA_02*b2*b2;
    double dpoly_da = 2.0*GAMMA_10*a + 4.0*GAMMA_20*a*a2 + 2.0*GAMMA_11*a*b2;
    double dpoly_db = 2.0*GAMMA_01*b + 2.0*GAMMA_11*a2*b + 4.0*GAMMA_02*b*b2;
    df_da = LAMBDA + 3.0*(1.0 - LAMBDA)*a2 + (1.0 - 3.0*a2)*poly + (1.0 - a2)*a*dpoly_da;
    df_db = (1.0 - a2)*a*dpoly_db;
}

FramePosition64 map_forward(int face, double u, double v) noexcept {
    double X = f_forward(u, v);
    double Z = f_forward(v, u);
    
    FramePosition64 p = {
        FACES[face].u.x * X + FACES[face].v.x * Z + FACES[face].n.x,
        FACES[face].u.y * X + FACES[face].v.y * Z + FACES[face].n.y,
        FACES[face].u.z * X + FACES[face].v.z * Z + FACES[face].n.z
    };
    
    double len = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    if(len > 0.0) { p.x /= len; p.y /= len; p.z /= len; }
    return p;
}

bool map_forward_differential(
    int face,
    double u,
    double v,
    FramePosition64& out_du,
    FramePosition64& out_dv
) noexcept {
    if (face < 0 || face >= 6 || !std::isfinite(u) || !std::isfinite(v)) return false;

    const double X = f_forward(u, v);
    const double Z = f_forward(v, u);
    const FramePosition64 q = {
        FACES[face].u.x * X + FACES[face].v.x * Z + FACES[face].n.x,
        FACES[face].u.y * X + FACES[face].v.y * Z + FACES[face].n.y,
        FACES[face].u.z * X + FACES[face].v.z * Z + FACES[face].n.z
    };
    const double q_length = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (!(q_length > 0.0) || !std::isfinite(q_length)) return false;
    const FramePosition64 p = { q.x / q_length, q.y / q_length, q.z / q_length };

    double dX_du = 0.0;
    double dX_dv = 0.0;
    double dZ_dv = 0.0;
    double dZ_du = 0.0;
    df_forward(u, v, dX_du, dX_dv);
    df_forward(v, u, dZ_dv, dZ_du);

    const FramePosition64 dq_du = {
        FACES[face].u.x * dX_du + FACES[face].v.x * dZ_du,
        FACES[face].u.y * dX_du + FACES[face].v.y * dZ_du,
        FACES[face].u.z * dX_du + FACES[face].v.z * dZ_du
    };
    const FramePosition64 dq_dv = {
        FACES[face].u.x * dX_dv + FACES[face].v.x * dZ_dv,
        FACES[face].u.y * dX_dv + FACES[face].v.y * dZ_dv,
        FACES[face].u.z * dX_dv + FACES[face].v.z * dZ_dv
    };
    const double radial_du = p.x * dq_du.x + p.y * dq_du.y + p.z * dq_du.z;
    const double radial_dv = p.x * dq_dv.x + p.y * dq_dv.y + p.z * dq_dv.z;
    out_du = {
        (dq_du.x - p.x * radial_du) / q_length,
        (dq_du.y - p.y * radial_du) / q_length,
        (dq_du.z - p.z * radial_du) / q_length
    };
    out_dv = {
        (dq_dv.x - p.x * radial_dv) / q_length,
        (dq_dv.y - p.y * radial_dv) / q_length,
        (dq_dv.z - p.z * radial_dv) / q_length
    };
    return std::isfinite(out_du.x) && std::isfinite(out_du.y) && std::isfinite(out_du.z) &&
        std::isfinite(out_dv.x) && std::isfinite(out_dv.y) && std::isfinite(out_dv.z);
}

bool map_inverse(const FramePosition64& p, int expected_canonical_face, double& out_u, double& out_v, int& out_face) noexcept {
    int best_face = 0;
    double max_dot = -2.0;
    
    for(int i=0; i<6; ++i) {
        double d = p.x*FACES[i].n.x + p.y*FACES[i].n.y + p.z*FACES[i].n.z;
        if(d > max_dot) {
            max_dot = d;
            best_face = i;
        }
    }
    
    out_face = expected_canonical_face != -1 ? expected_canonical_face : best_face;
    
    double X = (p.x*FACES[out_face].u.x + p.y*FACES[out_face].u.y + p.z*FACES[out_face].u.z) / 
               (p.x*FACES[out_face].n.x + p.y*FACES[out_face].n.y + p.z*FACES[out_face].n.z);
    double Z = (p.x*FACES[out_face].v.x + p.y*FACES[out_face].v.y + p.z*FACES[out_face].v.z) / 
               (p.x*FACES[out_face].n.x + p.y*FACES[out_face].n.y + p.z*FACES[out_face].n.z);
    
    // Univariate fifth-order initializer
    auto inv0 = [](double x) {
        double x2 = x*x;
        return x * (1.3432 - 0.4865*x2 + 0.1433*x2*x2);
    };
    double u = inv0(X), v = inv0(Z);
    
    for(int i=0; i<5; ++i) {
        double fu = f_forward(u, v) - X;
        double fv = f_forward(v, u) - Z;
        double dfu_du, dfu_dv, dfv_dv, dfv_du;
        df_forward(u, v, dfu_du, dfu_dv);
        df_forward(v, u, dfv_dv, dfv_du); 
        double det = dfu_du * dfv_dv - dfu_dv * dfv_du;
        if(std::abs(det) < 1e-10) return false;
        u -= (fu * dfv_dv - fv * dfu_dv) / det;
        v -= (fv * dfu_du - fu * dfv_du) / det;
    }
    
    if(!std::isfinite(u) || !std::isfinite(v)) return false;
    
    double dfu_du, dfu_dv, dfv_dv, dfv_du;
    df_forward(u, v, dfu_du, dfu_dv); df_forward(v, u, dfv_dv, dfv_du); 
    double final_det = dfu_du * dfv_dv - dfu_dv * dfv_du;
    if(final_det < 0.1) return false; // Minimum determinant bound
    
    if(u < -1.0001 || u > 1.0001 || v < -1.0001 || v > 1.0001) return false;
    
    double final_X = f_forward(u,v), final_Z = f_forward(v,u);
    double res = std::sqrt((final_X-X)*(final_X-X) + (final_Z-Z)*(final_Z-Z));
    if(res > 1e-10) return false; // Forward residual bound
    
    out_u = u; out_v = v;
    return true;
}

} // namespace ProjectionCOBE
} // namespace Multinet

#include "cubic_spline.h"
#include <cassert>
#include <cmath>
#include <cstdlib>

int main() {
    std::vector<Vec3> points{{0.f,0.f,0.f},{1.f,1.f,1.f},{2.f,2.f,2.f}};
    CubicSpline spline;
    spline.compute_coefficients(points);

    Vec3 p0 = spline.interpolate(0.f, 0);
    Vec3 p1 = spline.interpolate(1.f, 0);
    Vec3 p2 = spline.interpolate(0.f, 1);
    Vec3 p3 = spline.interpolate(1.f, 1);

    assert(std::fabs(p0.x - points[0].x) < 1e-5f);
    assert(std::fabs(p1.x - points[1].x) < 1e-5f);
    assert(std::fabs(p2.x - points[1].x) < 1e-5f);
    assert(std::fabs(p3.x - points[2].x) < 1e-5f);

    for (int i = 0; i < 100; ++i) {
        float v = GetRandomFloat(-1.f, 1.f);
        assert(v >= -1.f && v <= 1.f);
    }

    return 0;
}

#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include <vector>
#include <cmath>

struct Vec3 {
    float x;
    float y;
    float z;
};

struct CubicSpline {
    std::vector<float> x, y, z;
    std::vector<float> a_x, b_x, c_x, d_x;
    std::vector<float> a_y, b_y, c_y, d_y;
    std::vector<float> a_z, b_z, c_z, d_z;

    void compute_coefficients(const std::vector<Vec3>& points) {
        int n = points.size() - 1;

        x.resize(points.size());
        y.resize(points.size());
        z.resize(points.size());
        for (size_t i = 0; i < points.size(); i++) {
            x[i] = points[i].x;
            y[i] = points[i].y;
            z[i] = points[i].z;
        }

        a_x = x; a_y = y; a_z = z;
        b_x.resize(n); b_y.resize(n); b_z.resize(n);
        c_x.resize(n+1); c_y.resize(n+1); c_z.resize(n+1);
        d_x.resize(n); d_y.resize(n); d_z.resize(n);

        compute_spline_coeffs(x, a_x, b_x, c_x, d_x);
        compute_spline_coeffs(y, a_y, b_y, c_y, d_y);
        compute_spline_coeffs(z, a_z, b_z, c_z, d_z);
    }

    Vec3 interpolate(float t, int i) const {
        float dx = t;
        return {
            a_x[i] + b_x[i] * dx + c_x[i] * dx * dx + d_x[i] * dx * dx * dx,
            a_y[i] + b_y[i] * dx + c_y[i] * dx * dx + d_y[i] * dx * dx * dx,
            a_z[i] + b_z[i] * dx + c_z[i] * dx * dx + d_z[i] * dx * dx * dx
        };
    }

private:
    void compute_spline_coeffs(const std::vector<float>& values,
                               std::vector<float>& a,
                               std::vector<float>& b,
                               std::vector<float>& c,
                               std::vector<float>& d) {
        int n = values.size() - 1;
        std::vector<float> h(n);
        std::vector<float> alpha(n);
        std::vector<float> l(n + 1);
        std::vector<float> mu(n);
        std::vector<float> z(n + 1);

        for (int i = 0; i < n; i++) {
            h[i] = 1.0f;
        }

        for (int i = 1; i < n; i++) {
            alpha[i] = 3.0f * (values[i+1] - values[i]) / h[i]
                    - 3.0f * (values[i] - values[i-1]) / h[i-1];
        }

        l[0] = 1.0f;
        mu[0] = 0.0f;
        z[0] = 0.0f;

        for (int i = 1; i < n; i++) {
            l[i] = 2.0f * (h[i] + h[i-1]) - h[i-1] * mu[i-1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i-1] * z[i-1]) / l[i];
        }

        l[n] = 1.0f;
        z[n] = 0.0f;
        c[n] = 0.0f;

        for (int j = n-1; j >= 0; j--) {
            c[j] = z[j] - mu[j] * c[j+1];
            b[j] = (values[j+1] - values[j]) / h[j] - h[j] * (c[j+1] + 2.0f * c[j]) / 3.0f;
            d[j] = (c[j+1] - c[j]) / (3.0f * h[j]);
        }
    }
};

inline float GetRandomFloat(float min, float max) {
    return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

#endif // CUBIC_SPLINE_H

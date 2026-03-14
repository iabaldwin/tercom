// Standalone cubic spline test (no Three.js dependency needed)
// Run with: node tests/cubic_spline_test.js

// ---- Inline CubicSpline (same logic as app.js, without THREE.Vector3) ----

class CubicSpline {
    constructor() {
        this.a_x = []; this.b_x = []; this.c_x = []; this.d_x = [];
        this.a_y = []; this.b_y = []; this.c_y = []; this.d_y = [];
        this.a_z = []; this.b_z = []; this.c_z = []; this.d_z = [];
    }

    computeCoefficients(points) {
        const n = points.length - 1;
        const xVals = points.map(p => p.x);
        const yVals = points.map(p => p.y);
        const zVals = points.map(p => p.z);

        this.a_x = [...xVals]; this.a_y = [...yVals]; this.a_z = [...zVals];
        this.b_x = new Array(n); this.b_y = new Array(n); this.b_z = new Array(n);
        this.c_x = new Array(n + 1); this.c_y = new Array(n + 1); this.c_z = new Array(n + 1);
        this.d_x = new Array(n); this.d_y = new Array(n); this.d_z = new Array(n);

        this._computeCoeffs(xVals, this.a_x, this.b_x, this.c_x, this.d_x);
        this._computeCoeffs(yVals, this.a_y, this.b_y, this.c_y, this.d_y);
        this._computeCoeffs(zVals, this.a_z, this.b_z, this.c_z, this.d_z);
    }

    interpolate(t, i) {
        const dx = t;
        return {
            x: this.a_x[i] + this.b_x[i] * dx + this.c_x[i] * dx * dx + this.d_x[i] * dx * dx * dx,
            y: this.a_y[i] + this.b_y[i] * dx + this.c_y[i] * dx * dx + this.d_y[i] * dx * dx * dx,
            z: this.a_z[i] + this.b_z[i] * dx + this.c_z[i] * dx * dx + this.d_z[i] * dx * dx * dx
        };
    }

    _computeCoeffs(values, a, b, c, d) {
        const n = values.length - 1;
        const h = new Array(n).fill(1.0);
        const alpha = new Array(n).fill(0);
        const l = new Array(n + 1).fill(0);
        const mu = new Array(n).fill(0);
        const z = new Array(n + 1).fill(0);

        for (let i = 1; i < n; i++) {
            alpha[i] = 3.0 * (values[i + 1] - values[i]) / h[i]
                     - 3.0 * (values[i] - values[i - 1]) / h[i - 1];
        }

        l[0] = 1.0; mu[0] = 0.0; z[0] = 0.0;

        for (let i = 1; i < n; i++) {
            l[i] = 2.0 * (h[i] + h[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[n] = 1.0; z[n] = 0.0; c[n] = 0.0;

        for (let j = n - 1; j >= 0; j--) {
            c[j] = z[j] - mu[j] * c[j + 1];
            b[j] = (values[j + 1] - values[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        }
    }
}

function getRandomFloat(min, max) {
    return min + Math.random() * (max - min);
}

// ---- Tests ----

function assert(condition, msg) {
    if (!condition) throw new Error(`Assertion failed: ${msg}`);
}

function assertClose(a, b, eps = 1e-5, msg = '') {
    if (Math.abs(a - b) > eps) throw new Error(`assertClose failed: ${a} vs ${b} ${msg}`);
}

// Test 1: Interpolation at segment boundaries matches control points
const points = [
    { x: 0, y: 0, z: 0 },
    { x: 1, y: 1, z: 1 },
    { x: 2, y: 2, z: 2 },
];
const spline = new CubicSpline();
spline.computeCoefficients(points);

const p0 = spline.interpolate(0, 0);
const p1 = spline.interpolate(1, 0);
const p2 = spline.interpolate(0, 1);
const p3 = spline.interpolate(1, 1);

assertClose(p0.x, points[0].x, 1e-5, 'p0.x');
assertClose(p1.x, points[1].x, 1e-5, 'p1.x');
assertClose(p2.x, points[1].x, 1e-5, 'p2.x');
assertClose(p3.x, points[2].x, 1e-5, 'p3.x');

// Test 2: Random float in range
for (let i = 0; i < 100; i++) {
    const v = getRandomFloat(-1, 1);
    assert(v >= -1 && v <= 1, `getRandomFloat out of range: ${v}`);
}

console.log('All tests passed.');

import * as THREE from 'three';
import { PointerLockControls } from 'three/addons/controls/PointerLockControls.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';

// ============================================================
// Math utilities
// ============================================================

function getRandomFloat(min, max) {
    return min + Math.random() * (max - min);
}

// ============================================================
// Cubic Spline Interpolation
// ============================================================

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
        return new THREE.Vector3(
            this.a_x[i] + this.b_x[i] * dx + this.c_x[i] * dx * dx + this.d_x[i] * dx * dx * dx,
            this.a_y[i] + this.b_y[i] * dx + this.c_y[i] * dx * dx + this.d_y[i] * dx * dx * dx,
            this.a_z[i] + this.b_z[i] * dx + this.c_z[i] * dx * dx + this.d_z[i] * dx * dx * dx
        );
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

// ============================================================
// Kalman Filter (6D state: position + velocity)
// ============================================================

class KalmanFilter {
    constructor() {
        const STATE_DIM = 6;
        const MEAS_DIM = 3;
        const dt = 1.0 / 60.0;

        // State transition matrix (constant velocity model)
        this.A = Matrix.identity(STATE_DIM);
        for (let i = 0; i < 3; i++) this.A.data[i][i + 3] = dt;

        // State vector
        this.x = new Array(STATE_DIM).fill(0);

        // Covariance matrix
        this.P = Matrix.identity(STATE_DIM).scale(100.0);

        // Process noise
        this.Q = Matrix.identity(STATE_DIM);
        const posStd = 0.05, velStd = 0.05;
        for (let i = 0; i < 3; i++) {
            this.Q.data[i][i] = posStd * posStd;
            this.Q.data[i + 3][i + 3] = velStd * velStd;
        }

        // Measurement matrix (observe position only)
        this.H = Matrix.zeros(MEAS_DIM, STATE_DIM);
        for (let i = 0; i < 3; i++) this.H.data[i][i] = 1;

        // Measurement noise
        this.R = Matrix.identity(MEAS_DIM).scale(0.1);
    }

    predict() {
        this.x = this.A.mulVec(this.x);
        this.P = this.A.mul(this.P).mul(this.A.transpose()).add(this.Q);
    }

    update(measurement) {
        const z = [measurement.x, measurement.y, measurement.z];
        const Ht = this.H.transpose();
        const S = this.H.mul(this.P).mul(Ht).add(this.R);
        const K = this.P.mul(Ht).mul(S.inverse());

        const innovation = [];
        const Hx = this.H.mulVec(this.x);
        for (let i = 0; i < 3; i++) innovation.push(z[i] - Hx[i]);

        const Kinn = K.mulVec(innovation);
        for (let i = 0; i < this.x.length; i++) this.x[i] += Kinn[i];

        const I = Matrix.identity(this.x.length);
        this.P = I.sub(K.mul(this.H)).mul(this.P);
    }

    getPosition() {
        return new THREE.Vector3(this.x[0], this.x[1], this.x[2]);
    }

    getPositionCovariance() {
        return {
            xx: this.P.data[0][0], zz: this.P.data[2][2], xz: this.P.data[0][2]
        };
    }
}

// ============================================================
// Simple Matrix class (replaces Eigen)
// ============================================================

class Matrix {
    constructor(rows, cols, data) {
        this.rows = rows;
        this.cols = cols;
        this.data = data || Array.from({ length: rows }, () => new Array(cols).fill(0));
    }

    static identity(n) {
        const m = new Matrix(n, n);
        for (let i = 0; i < n; i++) m.data[i][i] = 1;
        return m;
    }

    static zeros(r, c) { return new Matrix(r, c); }

    scale(s) {
        const m = new Matrix(this.rows, this.cols);
        for (let i = 0; i < this.rows; i++)
            for (let j = 0; j < this.cols; j++)
                m.data[i][j] = this.data[i][j] * s;
        return m;
    }

    add(other) {
        const m = new Matrix(this.rows, this.cols);
        for (let i = 0; i < this.rows; i++)
            for (let j = 0; j < this.cols; j++)
                m.data[i][j] = this.data[i][j] + other.data[i][j];
        return m;
    }

    sub(other) {
        const m = new Matrix(this.rows, this.cols);
        for (let i = 0; i < this.rows; i++)
            for (let j = 0; j < this.cols; j++)
                m.data[i][j] = this.data[i][j] - other.data[i][j];
        return m;
    }

    mul(other) {
        const m = new Matrix(this.rows, other.cols);
        for (let i = 0; i < this.rows; i++)
            for (let j = 0; j < other.cols; j++)
                for (let k = 0; k < this.cols; k++)
                    m.data[i][j] += this.data[i][k] * other.data[k][j];
        return m;
    }

    mulVec(v) {
        const result = new Array(this.rows).fill(0);
        for (let i = 0; i < this.rows; i++)
            for (let j = 0; j < this.cols; j++)
                result[i] += this.data[i][j] * v[j];
        return result;
    }

    transpose() {
        const m = new Matrix(this.cols, this.rows);
        for (let i = 0; i < this.rows; i++)
            for (let j = 0; j < this.cols; j++)
                m.data[j][i] = this.data[i][j];
        return m;
    }

    inverse() {
        const n = this.rows;
        const aug = Array.from({ length: n }, (_, i) => {
            const row = [...this.data[i]];
            for (let j = 0; j < n; j++) row.push(i === j ? 1 : 0);
            return row;
        });

        for (let i = 0; i < n; i++) {
            let maxRow = i;
            for (let k = i + 1; k < n; k++)
                if (Math.abs(aug[k][i]) > Math.abs(aug[maxRow][i])) maxRow = k;
            [aug[i], aug[maxRow]] = [aug[maxRow], aug[i]];

            const pivot = aug[i][i];
            for (let j = 0; j < 2 * n; j++) aug[i][j] /= pivot;

            for (let k = 0; k < n; k++) {
                if (k === i) continue;
                const factor = aug[k][i];
                for (let j = 0; j < 2 * n; j++) aug[k][j] -= factor * aug[i][j];
            }
        }

        const result = new Matrix(n, n);
        for (let i = 0; i < n; i++)
            for (let j = 0; j < n; j++)
                result.data[i][j] = aug[i][n + j];
        return result;
    }
}

// ============================================================
// Game State
// ============================================================

const state = {
    currentPosition: new THREE.Vector3(0, 1, 0),
    velocity: new THREE.Vector3(),
    speed: 0.1,
    waypoints: [],         // { position: Vector3, reached: boolean }
    followingFigure8: true,
    figure8Points: [],
    kf: new KalmanFilter(),
    measurementCounter: 0,
    MEASUREMENT_INTERVAL: 10,
};

// ============================================================
// Figure-8 / Waypoint generation
// ============================================================

function generateFigure8(scale = 20.0, height = 1.0) {
    const points = [];
    const step = 0.05;
    for (let t = 0; t <= 2 * Math.PI; t += step) {
        points.push(new THREE.Vector3(scale * Math.cos(t), height, scale * Math.sin(t) * Math.cos(t)));
    }
    return points;
}

function populateFigure8Waypoints(scale = 20.0, height = 1.0) {
    const controlPoints = [];
    const tStep = 0.4;
    for (let t = 0; t <= 2 * Math.PI; t += tStep) {
        controlPoints.push(new THREE.Vector3(scale * Math.cos(t), height, scale * Math.sin(t) * Math.cos(t)));
    }
    controlPoints.push(controlPoints[0].clone());

    const spline = new CubicSpline();
    spline.computeCoefficients(controlPoints);

    const waypoints = [];
    const interpStep = 0.1;
    for (let i = 0; i < controlPoints.length - 1; i++) {
        for (let t = 0; t < 1.0; t += interpStep) {
            const pos = spline.interpolate(t, i);
            waypoints.push({ position: pos, reached: false });
        }
    }
    waypoints.push({ position: waypoints[0].position.clone(), reached: false });
    return waypoints;
}

// ============================================================
// Circular trajectory (original get_trajectory)
// ============================================================

function getCircularTrajectory(radius = 20.0) {
    const points = [];
    for (let theta = 0; theta < 2 * Math.PI; theta += 0.01) {
        points.push(new THREE.Vector3(radius * Math.cos(theta), 1.0, radius * Math.sin(theta)));
    }
    return points;
}

// ============================================================
// Three.js setup
// ============================================================

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xf5f5f5);

const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 10, 1);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(window.devicePixelRatio);
document.body.appendChild(renderer.domElement);

// Lighting
scene.add(new THREE.AmbientLight(0x666666));
const dirLight = new THREE.DirectionalLight(0xffffff, 1);
dirLight.position.set(10, 20, 10);
scene.add(dirLight);

// Controls (pointer lock for FPS-style camera)
const controls = new PointerLockControls(camera, renderer.domElement);
document.addEventListener('click', () => {
    if (!controls.isLocked) {
        controls.lock();
        return;
    }
    // Place waypoint when locked
    placeWaypoint();
});

// Movement
const moveState = { forward: false, backward: false, left: false, right: false };
document.addEventListener('keydown', e => {
    if (e.code === 'KeyW') moveState.forward = true;
    if (e.code === 'KeyS') moveState.backward = true;
    if (e.code === 'KeyA') moveState.left = true;
    if (e.code === 'KeyD') moveState.right = true;
});
document.addEventListener('keyup', e => {
    if (e.code === 'KeyW') moveState.forward = false;
    if (e.code === 'KeyS') moveState.backward = false;
    if (e.code === 'KeyA') moveState.left = false;
    if (e.code === 'KeyD') moveState.right = false;
});

// ============================================================
// Scene objects
// ============================================================

// Grid
const grid = new THREE.GridHelper(10, 10, 0x888888, 0xcccccc);
scene.add(grid);

// Coordinate axes
function createAxes() {
    const group = new THREE.Group();
    const matX = new THREE.LineBasicMaterial({ color: 0x0000ff });
    const matY = new THREE.LineBasicMaterial({ color: 0x00ff00 });
    const matZ = new THREE.LineBasicMaterial({ color: 0xff0000 });

    const geoX = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(10, 0, 0)]);
    const geoY = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, 10, 0)]);
    const geoZ = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, 0, 10)]);

    group.add(new THREE.Line(geoX, matZ)); // X axis = RED in original (Z in raylib maps to X in three)
    group.add(new THREE.Line(geoY, matY));
    group.add(new THREE.Line(geoZ, matX));
    return group;
}
scene.add(createAxes());

// World sphere wireframe
const sphereGeo = new THREE.SphereGeometry(10, 25, 25);
const sphereWire = new THREE.LineSegments(
    new THREE.WireframeGeometry(sphereGeo),
    new THREE.LineBasicMaterial({ color: 0xcccccc })
);
scene.add(sphereWire);

// Circular trajectory line
const circTraj = getCircularTrajectory();
const trajGeo = new THREE.BufferGeometry().setFromPoints(circTraj);
const trajLine = new THREE.Line(trajGeo, new THREE.LineBasicMaterial({ color: 0x0000ff }));
scene.add(trajLine);

// Figure-8 path line
state.figure8Points = generateFigure8();
const f8Geo = new THREE.BufferGeometry().setFromPoints([...state.figure8Points, state.figure8Points[0]]);
const f8Line = new THREE.Line(f8Geo, new THREE.LineBasicMaterial({ color: 0x888888 }));
scene.add(f8Line);

// Heightmap plane (simple flat terrain as placeholder since we can't load the original PNG via file://)
const terrainGeo = new THREE.PlaneGeometry(128, 128, 64, 64);
terrainGeo.rotateX(-Math.PI / 2);
const terrainMat = new THREE.MeshStandardMaterial({ color: 0xcc4444, wireframe: false, side: THREE.DoubleSide });
const terrain = new THREE.Mesh(terrainGeo, terrainMat);
scene.add(terrain);

// Hawk model (simple cone as placeholder — OBJ loading requires a server)
const hawkGeo = new THREE.ConeGeometry(0.5, 2, 8);
hawkGeo.rotateX(Math.PI / 2);
const hawkMat = new THREE.MeshStandardMaterial({ color: 0xff4444 });
const hawk = new THREE.Mesh(hawkGeo, hawkMat);
hawk.position.copy(state.currentPosition);
scene.add(hawk);

// Waypoint spheres group
const waypointGroup = new THREE.Group();
scene.add(waypointGroup);

// Uncertainty ellipse
const ellipseGeo = new THREE.BufferGeometry();
const ellipseMat = new THREE.LineBasicMaterial({ color: 0xff69b4 }); // PINK
const ellipseLine = new THREE.LineLoop(ellipseGeo, ellipseMat);
scene.add(ellipseLine);

// Uncertainty cross
const crossGroup = new THREE.Group();
const crossMat = new THREE.LineBasicMaterial({ color: 0xff69b4 });
const crossH = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(-0.5, 0, 0), new THREE.Vector3(0.5, 0, 0)]),
    crossMat
);
const crossV = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, -0.5), new THREE.Vector3(0, 0, 0.5)]),
    crossMat
);
crossGroup.add(crossH, crossV);
scene.add(crossGroup);

// ============================================================
// Waypoint placement via raycasting
// ============================================================

function placeWaypoint() {
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(new THREE.Vector2(0, 0), camera);

    // Intersect with y=0 ground plane
    const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    const intersection = new THREE.Vector3();
    raycaster.ray.intersectPlane(plane, intersection);

    if (intersection) {
        if (state.followingFigure8) {
            state.waypoints = [];
            state.followingFigure8 = false;
        }
        state.waypoints.push({ position: intersection.clone(), reached: false });
        rebuildWaypointVisuals();
    }
}

function rebuildWaypointVisuals() {
    waypointGroup.clear();
    for (const wp of state.waypoints) {
        const geo = new THREE.SphereGeometry(0.3, 12, 12);
        const mat = new THREE.MeshStandardMaterial({ color: wp.reached ? 0x00ff00 : 0xff0000 });
        const mesh = new THREE.Mesh(geo, mat);
        mesh.position.copy(wp.position);
        waypointGroup.add(mesh);
    }
}

// ============================================================
// Update uncertainty ellipse
// ============================================================

function updateUncertaintyEllipse() {
    const cov = state.kf.getPositionCovariance();
    const pos = state.kf.getPosition();

    const trace = cov.xx + cov.zz;
    const det = cov.xx * cov.zz - cov.xz * cov.xz;
    const disc = Math.sqrt(Math.max(0, trace * trace / 4 - det));
    const eval1 = trace / 2 + disc;
    const eval2 = trace / 2 - disc;
    const theta = Math.atan2(eval1 - cov.xx, cov.xz || 1e-10);

    const numPoints = 36;
    const points = [];
    for (let i = 0; i <= numPoints; i++) {
        const angle = i * 2 * Math.PI / numPoints;
        const x = Math.sqrt(Math.max(0, eval1)) * Math.cos(angle);
        const z = Math.sqrt(Math.max(0, eval2)) * Math.sin(angle);
        const rotX = x * Math.cos(theta) - z * Math.sin(theta);
        const rotZ = x * Math.sin(theta) + z * Math.cos(theta);
        points.push(new THREE.Vector3(pos.x + rotX, pos.y, pos.z + rotZ));
    }

    ellipseLine.geometry.dispose();
    ellipseLine.geometry = new THREE.BufferGeometry().setFromPoints(points);

    crossGroup.position.copy(pos);
}

// ============================================================
// Main simulation update
// ============================================================

function updateSimulation() {
    if (state.waypoints.length === 0) {
        state.followingFigure8 = true;
        state.waypoints = populateFigure8Waypoints();
        rebuildWaypointVisuals();
        return;
    }

    const nextWp = state.waypoints.find(wp => !wp.reached);
    if (!nextWp) return;

    const direction = new THREE.Vector3().subVectors(nextWp.position, state.currentPosition);
    const distance = direction.length();

    if (distance < 0.5) {
        nextWp.reached = true;

        const allReached = state.waypoints.every(wp => wp.reached);
        if (allReached) {
            const currentPos = state.currentPosition.clone();
            state.followingFigure8 = true;
            state.waypoints = populateFigure8Waypoints();

            const offset = new THREE.Vector3().subVectors(currentPos, state.waypoints[0].position);
            for (const wp of state.waypoints) wp.position.add(offset);
        }
        rebuildWaypointVisuals();
    } else {
        direction.normalize().multiplyScalar(state.speed);
        state.currentPosition.add(direction);

        // Kalman filter predict
        state.kf.predict();

        // Measurement update at intervals
        state.measurementCounter++;
        if (state.measurementCounter >= state.MEASUREMENT_INTERVAL) {
            const noisy = {
                x: state.currentPosition.x + getRandomFloat(-0.1, 0.1),
                y: state.currentPosition.y + getRandomFloat(-0.1, 0.1),
                z: state.currentPosition.z + getRandomFloat(-0.1, 0.1),
            };
            state.kf.update(noisy);
            state.measurementCounter = 0;
        }

        // Update hawk
        hawk.position.copy(state.currentPosition);
        const angle = Math.atan2(direction.x, direction.z) + Math.PI;
        hawk.rotation.set(0, angle, 0);
    }
}

// ============================================================
// Animation loop
// ============================================================

const moveSpeed = 0.3;

function animate() {
    requestAnimationFrame(animate);

    // Camera movement
    if (controls.isLocked) {
        if (moveState.forward) controls.moveForward(moveSpeed);
        if (moveState.backward) controls.moveForward(-moveSpeed);
        if (moveState.left) controls.moveRight(-moveSpeed);
        if (moveState.right) controls.moveRight(moveSpeed);
    }

    updateSimulation();
    updateUncertaintyEllipse();

    renderer.render(scene, camera);
}

// Handle resize
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// Initialize waypoints and start
state.waypoints = populateFigure8Waypoints();
rebuildWaypointVisuals();
animate();

// Export for testing
export { CubicSpline, KalmanFilter, Matrix, getRandomFloat };

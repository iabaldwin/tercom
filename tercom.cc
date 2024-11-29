#include <cmath>
#include <iostream>
#include <vector>
#include "raylib.h"
#include "raymath.h"
#include <Eigen/Dense>

struct Waypoint {
    Vector3 position;
    bool reached = false;
};

struct KalmanFilter {
    static const int STATE_DIM = 6;  // [x, y, z, vx, vy, vz]
    static const int MEAS_DIM = 3;   // [x, y, z]

    Eigen::MatrixXd A;  // State transition matrix
    Eigen::MatrixXd P;  // State covariance matrix
    Eigen::VectorXd x;  // State vector
    Eigen::MatrixXd Q;  // Process noise covariance
    Eigen::MatrixXd H;  // Measurement matrix
    Eigen::MatrixXd R;  // Measurement noise covariance

    KalmanFilter() {
        // Initialize state transition matrix (constant velocity model)
        A = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
        float dt = 1.0f/60.0f;  // Assuming 60 FPS
        // Fill in velocity components
        for (int i = 0; i < 3; i++) {
            A(i, i+3) = dt;
        }

        // Initialize state vector
        x = Eigen::VectorXd::Zero(STATE_DIM);

        // Initialize covariance matrix
        P = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 100.0;  // Large initial uncertainty

        // Initialize process noise
        Q = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
        float pos_std = 0.05f;  // Position process noise
        float vel_std = 0.05f;  // Velocity process noise
        for (int i = 0; i < 3; i++) {
            Q(i,i) = pos_std * pos_std;
            Q(i+3,i+3) = vel_std * vel_std;
        }

        // Initialize measurement matrix (we observe only position)
        H = Eigen::MatrixXd::Zero(MEAS_DIM, STATE_DIM);
        H.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3);

        // Initialize measurement noise with larger uncertainty
        R = Eigen::MatrixXd::Identity(MEAS_DIM, MEAS_DIM) * 0.1;  // Increased measurement noise
    }

    void predict() {
        // State prediction
        x = A * x;

        // Covariance prediction
        P = A * P * A.transpose() + Q;

        // Calculate total position uncertainty (trace of position covariance)
        float uncertainty = sqrt(P(0,0) + P(1,1) + P(2,2));

        TraceLog(LOG_INFO, "KF Predict - Pos est: (%.2f, %.2f, %.2f), Uncertainty: %.2f",
                 x(0), x(1), x(2), uncertainty);
    }

    Vector3 getPosition() const {
        return Vector3{
            static_cast<float>(x(0)),
            static_cast<float>(x(1)),
            static_cast<float>(x(2))
        };
    }

    Vector3 getVelocity() const {
        return Vector3{
            static_cast<float>(x(3)),
            static_cast<float>(x(4)),
            static_cast<float>(x(5))
        };
    }

    void update(const Vector3& measurement) {
        Eigen::Vector3d z;
        z << measurement.x, measurement.y, measurement.z;

        // Compute Kalman gain
        Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

        // Update state
        x = x + K * (z - H * x);

        // Update covariance
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
        P = (I - K * H) * P;

        // Calculate total position uncertainty (trace of position covariance)
        float uncertainty = sqrt(P(0,0) + P(1,1) + P(2,2));

        // Log the state and uncertainty
        TraceLog(LOG_INFO, "KF Update - Pos est: (%.2f, %.2f, %.2f), Uncertainty: %.2f",
                 x(0), x(1), x(2), uncertainty);
    }
};

struct GameState {
    float time = 0.0f;
    size_t currentTrajectoryIndex = 0;
    std::vector<Waypoint> waypoints;
    Vector3 currentPosition{0, 1, 0};
    Vector3 velocity{0, 0, 0};
    float speed = 0.1f;
    bool following_figure8 = true;
    std::vector<Vector3> figure8_points;
    float figure8_time = 0.0f;
    KalmanFilter kf;
    Vector3 terrainPosition;
    Mesh terrainMesh;
    int measurement_counter = 0;
    const int MEASUREMENT_INTERVAL = 10;
};

// Add this function before GetMouseWorldPosition
bool CheckCollisionRayPlane(Ray ray, Vector3 planePos, Vector3 planeNorm, float* distance) {
    float denom = Vector3DotProduct(planeNorm, ray.direction);
    if (fabs(denom) > 1e-6) {
        Vector3 diff = Vector3Subtract(planePos, ray.position);
        *distance = Vector3DotProduct(diff, planeNorm) / denom;
        return (*distance >= 0.0f);
    }
    return false;
}

// Add this function to get the ground point from mouse position
Vector3 GetCameraTargetPoint(const Camera3D& camera, float distance = 100.0f) {
    // Get the forward vector of the camera
    Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));

    // Create ray from camera position in forward direction
    Ray ray = { camera.position, forward };

    // Define the ground plane (y = 0)
    Vector3 planeNormal = { 0.0f, 1.0f, 0.0f };
    Vector3 planePoint = { 0.0f, 0.0f, 0.0f };

    float hitDistance = 0.0f;
    if (CheckCollisionRayPlane(ray, planePoint, planeNormal, &hitDistance)) {
        return Vector3Add(ray.position, Vector3Scale(ray.direction, hitDistance));
    }
    return { 0, 0, 0 };
}

void draw_world() {
  DrawSphereWires({0, 0, 0}, 10.f, 25, 25, LIGHTGRAY);
}

struct System {
  System(float x = 0, float y = 0, float z = 0) : x{x}, y{y}, z{z} {}
  float x;
  float y;
  float z;
  std::string reference{""};
};

void draw_coordinate_system(const System & system) {
  Vector3 centre{system.x, system.y, system.z};
  Vector3 X{0, 0, 10};
  DrawLine3D(centre, X, RED);
  Vector3 Y{0, 10, 0};
  DrawLine3D(centre, Y, GREEN);
  Vector3 Z{10, 0, 0};
  DrawLine3D(centre, Z, BLUE);
}

std::vector <System> get_trajectory() {
  std::vector <System> trajectory;
  const float RADIUS = 20.0;
  for (float theta = 0; theta < 2 * M_PI; theta += 0.01) {
    float x = RADIUS * cos(theta);
    const float y = 1.0;
    float z = RADIUS * sin(theta);
    trajectory.push_back({x, y, z});
  }
  return trajectory;
}

void draw_trajectory(const std::vector<System> & trajectory) {
  if (trajectory.empty()) {
    return;
  }
  for (int i = 1; i < trajectory.size(); ++i) {
    const Vector3 previous{trajectory.at(i-1).x,
                           trajectory.at(i-1).y,
                           trajectory.at(i-1).z};
    const Vector3 current{trajectory.at(i).x,
                          trajectory.at(i).y,
                          trajectory.at(i).z};
    DrawLine3D(previous, current, BLUE);
  }
}

// Add this helper function to calculate orbit position
Vector3 CalculateOrbitPosition(const Vector3& center, const Vector3& entryDir, float angle, float radius) {
    // Calculate the initial tangent point
    Vector3 right = Vector3Normalize((Vector3){-entryDir.z, 0, entryDir.x});

    // Rotate around center
    float x = center.x + radius * (right.x * cos(angle) - right.z * sin(angle));
    float z = center.z + radius * (right.x * sin(angle) + right.z * cos(angle));

    return (Vector3){x, center.y, z};
}

// Add this function to generate figure-8 points
std::vector<Vector3> generate_figure8(float scale = 20.0f, float height = 1.0f) {
    std::vector<Vector3> points;
    const float t_step = 0.05f;

    for (float t = 0; t <= 2 * M_PI; t += t_step) {
        // Using proper lemniscate equations
        float x = scale * cos(t);
        float z = scale * sin(t) * cos(t);
        points.push_back({x, height, z});
    }
    return points;
}

// Helper struct for cubic spline interpolation
struct CubicSpline {
    std::vector<float> x, y, z;
    std::vector<float> a_x, b_x, c_x, d_x;
    std::vector<float> a_y, b_y, c_y, d_y;
    std::vector<float> a_z, b_z, c_z, d_z;

    void compute_coefficients(const std::vector<Vector3>& points) {
        int n = points.size() - 1;

        // Extract coordinates
        x.resize(points.size());
        y.resize(points.size());
        z.resize(points.size());
        for (size_t i = 0; i < points.size(); i++) {
            x[i] = points[i].x;
            y[i] = points[i].y;
            z[i] = points[i].z;
        }

        // Initialize coefficient vectors
        a_x = x; a_y = y; a_z = z;
        b_x.resize(n); b_y.resize(n); b_z.resize(n);
        c_x.resize(n+1); c_y.resize(n+1); c_z.resize(n+1);
        d_x.resize(n); d_y.resize(n); d_z.resize(n);

        // Compute coefficients for each dimension
        compute_spline_coeffs(x, a_x, b_x, c_x, d_x);
        compute_spline_coeffs(y, a_y, b_y, c_y, d_y);
        compute_spline_coeffs(z, a_z, b_z, c_z, d_z);
    }

    Vector3 interpolate(float t, int i) const {
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

        // Step 1: Compute h_i
        for (int i = 0; i < n; i++) {
            h[i] = 1.0f;  // Assuming uniform spacing for simplicity
        }

        // Step 2: Compute alpha_i
        for (int i = 1; i < n; i++) {
            alpha[i] = 3.0f * (values[i+1] - values[i]) / h[i]
                    - 3.0f * (values[i] - values[i-1]) / h[i-1];
        }

        // Step 3: Compute l_i, mu_i, and z_i
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

        // Step 4: Compute coefficients
        for (int j = n-1; j >= 0; j--) {
            c[j] = z[j] - mu[j] * c[j+1];
            b[j] = (values[j+1] - values[j]) / h[j] - h[j] * (c[j+1] + 2.0f * c[j]) / 3.0f;
            d[j] = (c[j+1] - c[j]) / (3.0f * h[j]);
        }
    }
};

// Modify populate_figure8_waypoints to include spline interpolation
void populate_figure8_waypoints(std::vector<Waypoint>& waypoints, float scale = 20.0f, float height = 1.0f) {
    std::vector<Vector3> control_points;
    const float t_step = 0.4f;  // Larger steps for control points

    // Generate control points
    for (float t = 0; t <= 2 * M_PI; t += t_step) {
        Vector3 point = {
            scale * cos(t),
            height,
            scale * sin(t) * cos(t)
        };
        control_points.push_back(point);
    }

    // Ensure loop closure
    control_points.push_back(control_points[0]);

    // Create spline
    CubicSpline spline;
    spline.compute_coefficients(control_points);

    // Generate smooth waypoints
    waypoints.clear();
    const float interpolation_step = 0.1f;  // Smaller steps for smooth interpolation

    for (size_t i = 0; i < control_points.size() - 1; i++) {
        for (float t = 0.0f; t < 1.0f; t += interpolation_step) {
            Waypoint wp;
            wp.position = spline.interpolate(t, i);
            wp.reached = false;
            waypoints.push_back(wp);
        }
    }

    // Add final point to close the loop
    waypoints.push_back(waypoints[0]);

    TraceLog(LOG_INFO, "Generated %d waypoints for smooth figure-8", (int)waypoints.size());
}

// Add this helper function to draw uncertainty ellipse
void DrawUncertaintyEllipse(const KalmanFilter& kf, Color color) {
    // Get estimated position from Kalman filter state
    Vector3 estimatedPos = {
        (float)kf.x(0),  // x position estimate
        (float)kf.x(1),  // y position estimate
        (float)kf.x(2)   // z position estimate
    };

    // Extract position covariance (2x2 matrix for X-Z plane)
    float covX = kf.P(0,0);
    float covZ = kf.P(2,2);
    float covXZ = kf.P(0,2);

    // Calculate eigenvalues and eigenvectors of covariance matrix
    float trace = covX + covZ;
    float det = covX * covZ - covXZ * covXZ;
    float disc = sqrt((trace * trace) / 4 - det);
    float eval1 = trace/2 + disc;
    float eval2 = trace/2 - disc;

    // Calculate major axis angle
    float theta = atan2(eval1 - covX, covXZ);

    // Draw ellipse points
    const int numPoints = 36;
    Vector3 prevPoint;
    // Scale factor to make uncertainty visible but not too large
    float scale = 1.0f;  // Increased from 0.1f

    for (int i = 0; i <= numPoints; i++) {
        float angle = i * 2 * PI / numPoints;
        float x = scale * sqrt(eval1) * cos(angle);
        float z = scale * sqrt(eval2) * sin(angle);

        // Rotate points
        float rotX = x * cos(theta) - z * sin(theta);
        float rotZ = x * sin(theta) + z * cos(theta);

        Vector3 point = {
            estimatedPos.x + rotX,
            estimatedPos.y,
            estimatedPos.z + rotZ
        };

        if (i > 0) {
            DrawLine3D(prevPoint, point, color);
        }
        prevPoint = point;
    }

    // Add cross at center for visibility
    float crossSize = 0.5f;
    Vector3 center = estimatedPos;
    DrawLine3D(
        Vector3{center.x - crossSize, center.y, center.z},
        Vector3{center.x + crossSize, center.y, center.z},
        color
    );
    DrawLine3D(
        Vector3{center.x, center.y, center.z - crossSize},
        Vector3{center.x, center.y, center.z + crossSize},
        color
    );
}

// Add this helper function near the top of the file
float GetRandomFloat(float min, float max) {
    return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 3*800;
    const int screenHeight = 3*450;

    InitWindow(screenWidth, screenHeight, "raylib [core] example - 3d camera free");

    // Define the camera to look into our 3d world
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 00.0f, 10.0f, 1.0f };  // Camera position
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

    Vector3 cubePosition = { 0.0f, 0.0f, 0.0f };

    DisableCursor();                    // Limit cursor to relative movement inside the window

    SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

    Image image = LoadImage("thirdparty/raylib/examples/models/resources/heightmap.png");     // Load heightmap image (RAM)
    Texture2D texture = LoadTextureFromImage(image);        // Convert image to texture (VRAM)

    Mesh mesh = GenMeshHeightmap(image, (Vector3){ 16*8, 4, 16*8 }); // Generate heightmap mesh (RAM and VRAM)
    Model model = LoadModelFromMesh(mesh);                  // Load model from generated mesh

    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture; // Set map diffuse texture
    Vector3 mapPosition = { -0.0f, 0.0f, -0.0f };           // Define model position

    UnloadImage(image);             // Unload heightmap image from RAM, already uploaded to VRAM

    auto hawk = LoadModel("media/source/tomahawk.obj");

    const auto trajectory = get_trajectory();
    GameState state;
    state.figure8_points = generate_figure8();

    // Main game loop
    while (!WindowShouldClose())        // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        UpdateCamera(&camera, CAMERA_FREE);

        // Handle mouse input for waypoints
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Vector3 targetPos = GetCameraTargetPoint(camera);

            // Clear existing waypoints if we're currently following figure-8
            if (state.following_figure8) {
                state.waypoints.clear();
                state.following_figure8 = false;
            }

            // Add new waypoint
            Waypoint wp;
            wp.position = targetPos;
            wp.reached = false;
            state.waypoints.push_back(wp);
        }

        // Update missile position
        if (!state.waypoints.empty()) {
            auto it = std::find_if(state.waypoints.begin(), state.waypoints.end(),
                                 [](const Waypoint& wp) { return !wp.reached; });

            if (it != state.waypoints.end()) {
                Vector3 direction = Vector3Subtract(it->position, state.currentPosition);
                float distance = Vector3Length(direction);

                // If we're close enough to waypoint
                if (distance < 0.5f) {
                    it->reached = true;

                    // Check if all waypoints are reached
                    bool all_reached = std::all_of(state.waypoints.begin(), state.waypoints.end(),
                        [](const Waypoint& wp) { return wp.reached; });

                    if (all_reached) {
                        // All user waypoints reached, generate new figure-8 starting from current position
                        state.waypoints.clear();
                        state.following_figure8 = true;

                        // Generate figure-8 starting from current position
                        Vector3 currentPos = state.currentPosition;
                        populate_figure8_waypoints(state.waypoints);

                        // Translate all waypoints to start from current position
                        Vector3 offset = Vector3Subtract(currentPos, state.waypoints[0].position);
                        for (auto& wp : state.waypoints) {
                            wp.position = Vector3Add(wp.position, offset);
                        }

                        TraceLog(LOG_INFO, "Resuming figure-8 pattern from position (%.2f, %.2f, %.2f)",
                            currentPos.x, currentPos.y, currentPos.z);
                    }
                } else {
                    // Normal waypoint movement
                    direction = Vector3Scale(Vector3Normalize(direction), state.speed);
                    state.currentPosition = Vector3Add(state.currentPosition, direction);

                    // Always predict next state
                    state.kf.predict();

                    // Update measurement less frequently
                    state.measurement_counter++;
                    if (state.measurement_counter >= state.MEASUREMENT_INTERVAL) {
                        // Add measurement noise to simulate sensor uncertainty
                        Vector3 noisyMeasurement = {
                            state.currentPosition.x + GetRandomFloat(-0.1f, 0.1f),
                            state.currentPosition.y + GetRandomFloat(-0.1f, 0.1f),
                            state.currentPosition.z + GetRandomFloat(-0.1f, 0.1f)
                        };

                        // Update Kalman filter with noisy measurement
                        state.kf.update(noisyMeasurement);

                        // Reset counter
                        state.measurement_counter = 0;

                        TraceLog(LOG_INFO, "Measurement update at position (%.2f, %.2f, %.2f)",
                            noisyMeasurement.x, noisyMeasurement.y, noisyMeasurement.z);
                    }

                    // Update hawk transformation
                    hawk.transform = MatrixIdentity();
                    float rotationAngle = atan2(direction.x, direction.z) + M_PI;
                    hawk.transform = MatrixRotateY(rotationAngle);
                    hawk.transform = MatrixMultiply(hawk.transform,
                        MatrixTranslate(state.currentPosition.x,
                                       state.currentPosition.y,
                                       state.currentPosition.z));
                }
            }
        } else {
            // No waypoints, populate with figure-8 from current position
            state.following_figure8 = true;
            populate_figure8_waypoints(state.waypoints);
        }

        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(RAYWHITE);

            BeginMode3D(camera);
                //DrawCube(cubePosition, 2.0f, 2.0f, 2.0f, RED);
                //DrawCubeWires(cubePosition, 2.0f, 2.0f, 2.0f, MAROON);
                DrawGrid(10, 1.0f);
                //draw_world();
                DrawModel(model, mapPosition, 1.0f, RED);
                DrawModel(hawk, mapPosition, 1.0f, RED);
                System s;
                draw_coordinate_system(s);
                draw_trajectory(trajectory);

                hawk.transform.m12 += 0.1;

                // Draw waypoints
                for (const auto& waypoint : state.waypoints) {
                    Color wpColor = waypoint.reached ? GREEN : RED;
                    DrawSphere(waypoint.position, 0.3f, wpColor);
                }

                // Optionally draw the figure-8 path
                for (size_t i = 0; i < state.figure8_points.size(); i++) {
                    Vector3 p1 = state.figure8_points[i];
                    Vector3 p2 = state.figure8_points[(i + 1) % state.figure8_points.size()];
                    DrawLine3D(p1, p2, GRAY);
                }

                // Draw uncertainty ellipse
                DrawUncertaintyEllipse(state.kf, PINK);  // Or try RED, YELLOW, or other bright colors

            EndMode3D();

            DrawRectangle( 10, 10, 320, 133, Fade(SKYBLUE, 0.5f));
            DrawRectangleLines( 10, 10, 320, 133, BLUE);

            DrawText("Free camera default controls:", 20, 20, 10, BLACK);
            DrawText("- Mouse Wheel to Zoom in-out", 40, 40, 10, DARKGRAY);
            DrawText("- Mouse Wheel Pressed to Pan", 40, 60, 10, DARKGRAY);
            DrawText("- Alt + Mouse Wheel Pressed to Rotate", 40, 80, 10, DARKGRAY);
            //DrawText("- Alt + Ctrl + Mouse Wheel Pressed for Smooth Zoom", 40, 100, 10, DARKGRAY);
            DrawText("- Z to zoom to (0, 0, 0)", 40, 120, 10, DARKGRAY);

            // Draw reticle
            int screenWidth = GetScreenWidth();
            int screenHeight = GetScreenHeight();
            int centerX = screenWidth / 2;
            int centerY = screenHeight / 2;
            int reticleSize = 20;

            // Draw crosshair
            DrawLine(centerX - reticleSize, centerY, centerX + reticleSize, centerY, BLACK);
            DrawLine(centerX, centerY - reticleSize, centerX, centerY + reticleSize, BLACK);

            // Optional: Draw small circle in center
            DrawCircleLines(centerX, centerY, 4, BLACK);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}

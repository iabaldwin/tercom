#include <cmath>
#include <iostream>
#include <vector>
#include "raylib.h"
#include "raymath.h"

struct Waypoint {
    Vector3 position;
    bool reached = false;
};

struct GameState {
    float time = 0.0f;
    size_t currentTrajectoryIndex = 0;
    std::vector<Waypoint> waypoints;
    Vector3 currentPosition{0, 1, 0};
    Vector3 velocity{0, 0, 0};
    float speed = 0.5f;
    bool following_figure8 = true;
    std::vector<Vector3> figure8_points;
    float figure8_time = 0.0f;
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

// Helper function to create figure-8 waypoints
void populate_figure8_waypoints(std::vector<Waypoint>& waypoints, float scale = 20.0f, float height = 1.0f) {
    waypoints.clear();
    const float t_step = 0.2f;  // Larger steps for fewer waypoints
    
    // Generate one complete figure-8
    for (float t = 0; t <= 2 * M_PI + t_step; t += t_step) {
        Waypoint wp;
        // Proper lemniscate equations
        wp.position.x = scale * cos(t);  // Changed from sin(2*t)
        wp.position.y = height;
        wp.position.z = scale * sin(t) * cos(t);
        wp.reached = false;
        waypoints.push_back(wp);
    }

    // Debug output to verify waypoint positions
    for (const auto& wp : waypoints) {
        TraceLog(LOG_INFO, "Figure-8 waypoint: %f, %f, %f", 
            wp.position.x, wp.position.y, wp.position.z);
    }
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
            Waypoint wp;
            wp.position = targetPos;
            wp.reached = false;
            state.waypoints.push_back(wp);
            state.following_figure8 = false;
        }

        // Update missile position
        if (!state.waypoints.empty()) {
            // Find first unvisited waypoint
            auto it = std::find_if(state.waypoints.begin(), state.waypoints.end(),
                                 [](const Waypoint& wp) { return !wp.reached; });
            
            if (it != state.waypoints.end()) {
                // Calculate direction to waypoint
                Vector3 direction = Vector3Subtract(it->position, state.currentPosition);
                float distance = Vector3Length(direction);
                
                // If we're close enough to waypoint
                if (distance < 0.5f) {
                    it->reached = true;
                    
                    // If all waypoints are reached, repopulate with figure-8
                    if (it == state.waypoints.end() - 1) {
                        populate_figure8_waypoints(state.waypoints);
                    }
                } else {
                    // Normal waypoint movement
                    direction = Vector3Scale(Vector3Normalize(direction), state.speed);
                    state.currentPosition = Vector3Add(state.currentPosition, direction);
                    
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
            // No waypoints, populate with figure-8
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

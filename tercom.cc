#include <cmath>
#include <iostream>
#include <vector>

#include "raylib.h"
#include "raymath.h"

struct System {
  System(float x = 0, float y = 0, float z = 0) : x{x}, y{y}, z{z} {}
  float x;
  float y;
  float z;
  std::string reference{""};
};

void DrawCoordinateSystem(const System & system) {
  Vector3 centre{system.x, system.y, system.z};
  Vector3 X{0, 0, 10};
  DrawLine3D(centre, X, RED);
  Vector3 Y{0, 10, 0};
  DrawLine3D(centre, Y, GREEN);
  Vector3 Z{10, 0, 0};
  DrawLine3D(centre, Z, BLUE);
}

std::vector <Transform> create_trajectory() {
  std::vector <Transform> trajectory;
  const float RADIUS = 20.0;
  for (float theta = 0; theta < 2 * M_PI; theta += 0.01) {
    float x = RADIUS * cos(theta);
    const float y = 1.0;
    float z = RADIUS * sin(theta);
    const auto rotation = QuaternionFromEuler(0, -1 * std::atan2(z, x) + M_PI, 0);
    Transform transform;
    transform.translation = {x, y, z};
    transform.rotation = rotation;
    trajectory.push_back(transform);
  }
  return trajectory;
}

void DrawTrajectory(const std::vector<Transform> & trajectory) {
  if (trajectory.empty()) {
    return;
  }
  for (int i = 1; i < trajectory.size(); ++i) {
    const Vector3 previous{trajectory.at(i-1).translation.x, 
                           trajectory.at(i-1).translation.y, 
                           trajectory.at(i-1).translation.z};
    const Vector3 current{trajectory.at(i).translation.x, 
                          trajectory.at(i).translation.y, 
                          trajectory.at(i).translation.z};
    DrawLine3D(previous, current, BLUE);
  }
}

void update_tomahawk_position(Model & tomahawk, const std::vector<Transform> & trajectory, int counter) {
  if (trajectory.empty()) {
    return;
  }
  int relative_counter = counter % trajectory.size();
  const Transform & transform = trajectory.at(relative_counter);
  auto matrix = QuaternionToMatrix(transform.rotation);
  matrix.m12 = transform.translation.x;
  matrix.m13 = transform.translation.y;
  matrix.m14 = transform.translation.z;
  tomahawk.transform = matrix;
}

float measure_altitude(const Model & tomahawk, const Model & terrain) {
  const Ray ray{{tomahawk.transform.m12, tomahawk.transform.m13, tomahawk.transform.m14}, {0, -1, 0}};
  DrawRay(ray, RED);
  const auto collision = GetRayCollisionMesh(ray, terrain.meshes[0], terrain.transform);
  if (collision.hit) {
    std::cout << "Collision: " << collision.point.y << std::endl;
    return collision.distance;
  }
  return -1;
}

void draw_altitude(std::vector<std::pair<int, float>> & altitude_buffer) {
  if (altitude_buffer.empty()) {
    return;
  }
  for (int i = 1; i < altitude_buffer.size(); ++i) {
    const Vector3 previous{altitude_buffer.at(i-1).first, 
      altitude_buffer.at(i-1).second, 
      0};
    const Vector3 current{altitude_buffer.at(i).first, 
      altitude_buffer.at(i).second, 
      0};
    //DrawLine(previous, current, RED);
  }
}

int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 2*800;
    const int screenHeight = 2*450;

    InitWindow(screenWidth, screenHeight, "TERCOM");

    Camera3D camera = { 0 };
    camera.position = (Vector3){ -10.0f, 10.0f, -10.0f }; // Camera position
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

    DisableCursor();                    // Limit cursor to relative movement inside the window

    SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second

    Image image = LoadImage("thirdparty/raylib/examples/models/resources/heightmap.png");     // Load heightmap image (RAM)
    Texture2D texture = LoadTextureFromImage(image);        // Convert image to texture (VRAM)

    Mesh mesh = GenMeshHeightmap(image, (Vector3){ 16*8, 4, 16*8 }); // Generate heightmap mesh (RAM and VRAM)
    Model terrain = LoadModelFromMesh(mesh);

    terrain.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
    Vector3 terrainPosition = { -0.0f, 0.0f, -0.0f };           // Define map position

    UnloadImage(image);

    auto tomahawk = LoadModel("media/source/tomahawk.obj");
    const auto trajectory = create_trajectory();

    int counter = 0;

    Rectangle inset = { 200, 200, 40, 40 };

    Camera2D camera1 = { 0 };
    camera1.target = (Vector2){ inset.x, inset.y };
    camera1.offset = (Vector2){ 200.0f, 200.0f };
    camera1.rotation = 0.0f;
    camera1.zoom = 1.0f;

    RenderTexture screenCamera1 = LoadRenderTexture(screenWidth / 2, screenHeight);
    Rectangle splitScreenRect = { 0.0f, 0.0f, (float)screenCamera1.texture.width, (float)-screenCamera1.texture.height / 1.0 };


    std::vector<std::pair<int, float>> altitude_buffer;

    // Main game loop
    while (!WindowShouldClose())        // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        UpdateCamera(&camera, CAMERA_FREE);
        //UpdateCamera(&camera, CAMERA_FIRST_PERSON);

        if (IsKeyDown('Z')) camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
        //----------------------------------------------------------------------------------
        BeginTextureMode(screenCamera1);
          ClearBackground(RAYWHITE);
          BeginMode2D(camera1);
            DrawRectangleRec(inset, RED);
          EndMode2D();
        EndTextureMode();

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(RAYWHITE);

            BeginMode3D(camera);
                DrawGrid(10, 1.0f);
                DrawModel(terrain, terrainPosition, 1.0f, GREEN);
                DrawModel(tomahawk, terrainPosition, 1.0f, RED);
                System s;
                DrawCoordinateSystem(s);
                DrawTrajectory(trajectory);
                update_tomahawk_position(tomahawk, trajectory, counter++);
                const auto altitude = measure_altitude(tomahawk, terrain);
                altitude_buffer.push_back(std::make_pair(counter, altitude));
            EndMode3D();

            DrawTextureRec(screenCamera1.texture, splitScreenRect, (Vector2){ 0, 0 }, WHITE);
            DrawRectangle( 10, 10, 320, 133, Fade(SKYBLUE, 0.5f));
            draw_altitude(altitude_buffer);

        EndDrawing();
    }

    CloseWindow();        // Close window and OpenGL context

    return EXIT_SUCCESS;
}

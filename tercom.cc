#include <cmath>
#include <iostream>
#include <vector>
#include "raylib.h"

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
    camera.position = (Vector3){ 00.0f, 10.0f, 1.0f }; // Camera position
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

    // Main game loop
    while (!WindowShouldClose())        // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        UpdateCamera(&camera, CAMERA_FREE);
        //UpdateCamera(&camera, CAMERA_FIRST_PERSON);

        if (IsKeyDown('Z')) camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
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

            EndMode3D();

            DrawRectangle( 10, 10, 320, 133, Fade(SKYBLUE, 0.5f));
            DrawRectangleLines( 10, 10, 320, 133, BLUE);

            DrawText("Free camera default controls:", 20, 20, 10, BLACK);
            DrawText("- Mouse Wheel to Zoom in-out", 40, 40, 10, DARKGRAY);
            DrawText("- Mouse Wheel Pressed to Pan", 40, 60, 10, DARKGRAY);
            DrawText("- Alt + Mouse Wheel Pressed to Rotate", 40, 80, 10, DARKGRAY);
            //DrawText("- Alt + Ctrl + Mouse Wheel Pressed for Smooth Zoom", 40, 100, 10, DARKGRAY);
            DrawText("- Z to zoom to (0, 0, 0)", 40, 120, 10, DARKGRAY);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}

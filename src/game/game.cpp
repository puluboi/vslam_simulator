#include "game.hpp"

Game::Game() : player({0.0f, 2.0f, 2.0f})
{
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "VSLAM Simulator");
    SetWindowPosition(100, 100); // Force position on primary monitor

    // DisableCursor();
    SetTargetFPS(60);
    createWorld();
}

Game::~Game()
{
    UnloadModel(map); // Free model memory
    CloseWindow();
}

void Game::createWorld()
{
    const char *assetPath = "assets/destroyed_warehouse_in_kaarina_finland";

    // Store original working directory
    const char *originalDir = GetWorkingDirectory();
    char savedDir[512];
    strncpy(savedDir, originalDir, sizeof(savedDir) - 1);
    savedDir[sizeof(savedDir) - 1] = '\0';

    // Change to asset directory so GLTF can find textures
    ChangeDirectory(assetPath);
    map = LoadModel("scene.gltf");
    ChangeDirectory(savedDir);

    TraceLog(LOG_INFO, "Model loaded - meshes: %d, materials: %d", map.meshCount, map.materialCount);

    // Load the base color texture
    const char *texturePath = TextFormat("%s/textures/Destroyed_warehouse_in_Kaarina_finland_baseColor.jpeg", assetPath);
    if (FileExists(texturePath))
    {
        Texture2D texture = LoadTexture(texturePath);
        // Apply texture to all materials
        for (int i = 0; i < map.materialCount; i++)
        {
            map.materials[i].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
            TraceLog(LOG_INFO, "Applied texture to material %d", i);
        }
    }
    else
    {
        TraceLog(LOG_WARNING, "Texture not found: %s", texturePath);
    }
}
void Game::run()
{
    while (!WindowShouldClose())
    {
        update();
        draw();
    }
}

void Game::update()
{
    player.update();
}

void Game::draw()
{
    Camera3D camera = player.getCamera();
    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode3D(camera);

    DrawModel(map, (Vector3){0.0f, -8.f, 0.0f}, 3.0f, WHITE);
    //DrawPlane((Vector3){0.0f, 0.0f, 0.0f}, (Vector2){32.0f, 32.0f}, LIGHTGRAY);
    //  Draw player cube
    if (cameraMode == CAMERA_THIRD_PERSON)
    {
        DrawCube(camera.target, 0.5f, 0.5f, 0.5f, PURPLE);
        DrawCubeWires(camera.target, 0.5f, 0.5f, 0.5f, DARKPURPLE);
    }

    EndMode3D();

    // Draw info boxes
    DrawRectangle(5, 5, 330, 100, Fade(SKYBLUE, 0.5f));
    DrawRectangleLines(5, 5, 330, 100, BLUE);

    DrawText("Camera controls:", 15, 15, 10, BLACK);
    DrawText("- Move keys: W, A, S, D, Space, Left-Ctrl", 15, 30, 10, BLACK);
    DrawText("- Look around: arrow keys or mouse", 15, 45, 10, BLACK);
    DrawText("- Camera mode keys: 1, 2, 3, 4", 15, 60, 10, BLACK);
    DrawText("- Zoom keys: num-plus, num-minus or mouse scroll", 15, 75, 10, BLACK);
    DrawText("- Camera projection key: P", 15, 90, 10, BLACK);

    DrawRectangle(600, 5, 195, 100, Fade(SKYBLUE, 0.5f));
    DrawRectangleLines(600, 5, 195, 100, BLUE);

    DrawText("Camera status:", 610, 15, 10, BLACK);
    DrawText(TextFormat("- Mode: %s", (cameraMode == CAMERA_FREE) ? "FREE" : (cameraMode == CAMERA_FIRST_PERSON) ? "FIRST_PERSON"
                                                                         : (cameraMode == CAMERA_THIRD_PERSON)   ? "THIRD_PERSON"
                                                                         : (cameraMode == CAMERA_ORBITAL)        ? "ORBITAL"
                                                                                                                 : "CUSTOM"),
             610, 30, 10, BLACK);
    DrawText(TextFormat("- Projection: %s", (camera.projection == CAMERA_PERSPECTIVE) ? "PERSPECTIVE" : (camera.projection == CAMERA_ORTHOGRAPHIC) ? "ORTHOGRAPHIC"
                                                                                                                                                   : "CUSTOM"),
             610, 45, 10, BLACK);
    DrawText(TextFormat("- Position: (%06.3f, %06.3f, %06.3f)", camera.position.x, camera.position.y, camera.position.z), 610, 60, 10, BLACK);
    DrawText(TextFormat("- Target: (%06.3f, %06.3f, %06.3f)", camera.target.x, camera.target.y, camera.target.z), 610, 75, 10, BLACK);
    DrawText(TextFormat("- Up: (%06.3f, %06.3f, %06.3f)", camera.up.x, camera.up.y, camera.up.z), 610, 90, 10, BLACK);

    // IMU data display
    DrawRectangle(600, 110, 195, 50, Fade(SKYBLUE, 0.5f));
    DrawRectangleLines(600, 110, 195, 50, BLUE);
    DrawText(TextFormat("- Accel: (%.1f, %.1f, %.1f)", player.imuLinearAcceleration().x, player.imuLinearAcceleration().y, player.imuLinearAcceleration().z), 610, 120, 10, BLACK);
    DrawText(TextFormat("- Gyro: (%.1f, %.1f, %.1f)", player.imuGyroAcceleration().x*10, player.imuGyroAcceleration().y*10, player.imuGyroAcceleration().z*10), 610, 135, 10, BLACK);

    EndDrawing();
}
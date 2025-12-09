#include "game.hpp"

Game::Game() : player({0.0f, 2.0f, 2.0f}), cameraPreview({0})
{
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "VSLAM Simulator");
    SetWindowPosition(100, 100); // Force position on primary monitor

    // DisableCursor();
    SetTargetFPS(60);
    createWorld();
    
    // Initialize ROS 2 publisher
    ros_publisher_ = std::make_unique<RosPublisher>();
}

Game::~Game()
{
    if (cameraPreview.id != 0)
    {
        UnloadTexture(cameraPreview);
    }
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
        ros_publisher_->spin();  // Process ROS callbacks
    }
}

void Game::update()
{
    player.update();
    
    // Toggle camera preview with P key
    if (IsKeyPressed(KEY_P))
    {
        showCameraPreview = !showCameraPreview;
        TraceLog(LOG_INFO, "Camera preview %s", showCameraPreview ? "enabled" : "disabled");
    }
    
    // Publish ground truth pose and twist
    ros_publisher_->publishPose(player.getPose());
    ros_publisher_->publishTwist(player.getTwist());
    
    // Publish IMU data
    ros_publisher_->publishIMU(player.imuLinearAcceleration(), player.imuGyroAcceleration());
}

void Game::draw()
{
    Camera3D camera = player.getCamera();
    
    // First render the 3D scene (this is what ROS will capture)
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
    
    // Capture frame for ROS right after 3D rendering, before UI overlay
    captureAndPublishFrame();

    // Draw info boxes
    DrawRectangle(5, 5, 330, 100, Fade(SKYBLUE, 0.5f));
    DrawRectangleLines(5, 5, 330, 100, BLUE);

    DrawText("Camera controls:", 15, 15, 10, BLACK);
    DrawText("- Move keys: W, A, S, D, Space, Left-Ctrl", 15, 30, 10, BLACK);
    DrawText("- Look around: arrow keys or mouse", 15, 45, 10, BLACK);
    DrawText("- Camera mode keys: 1, 2, 3, 4", 15, 60, 10, BLACK);
    DrawText("- Zoom keys: num-plus, num-minus or mouse scroll", 15, 75, 10, BLACK);
    DrawText("- Camera preview toggle: P", 15, 90, 10, BLACK);

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

    // Draw camera preview (what ROS will see) if enabled
    if (showCameraPreview && cameraPreview.id != 0)
    {
        int previewWidth = 160;
        int previewHeight = 90;
        DrawTexturePro(
            cameraPreview,
            (Rectangle){0, 0, (float)cameraPreview.width, (float)cameraPreview.height},
            (Rectangle){10, screenHeight - previewHeight - 10, (float)previewWidth, (float)previewHeight},
            (Vector2){0, 0},
            0.0f,
            WHITE
        );
        DrawRectangleLines(10, screenHeight - previewHeight - 10, previewWidth, previewHeight, RED);
        DrawText("ROS Camera", 15, screenHeight - previewHeight - 25, 10, RED);
    }

    EndDrawing();
}

void Game::captureAndPublishFrame()
{
    // Capture frame right after 3D rendering (before UI overlay)
    // This gives us a clean camera view for ROS
    Image screenshot = LoadImageFromScreen();
    
    // Update preview texture if preview is enabled
    if (showCameraPreview)
    {
        if (cameraPreview.id != 0)
        {
            UnloadTexture(cameraPreview);
        }
        cameraPreview = LoadTextureFromImage(screenshot);
    }
    
    // Publish to ROS 2, passing the player's camera
    ros_publisher_->publishImage(screenshot, player.getCamera());
    
    // Free the screenshot memory
    UnloadImage(screenshot);
}
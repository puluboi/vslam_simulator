#pragma once
#include "raylib.h"
#include "rcamera.h"
#include <string.h>
#include "player.hpp"

struct data{
    Image frame;
    int ax, yx, zx;
};

class Game{
public:
    Game();
    ~Game();
    void run();
    data getData();

private:

    void update();
    void draw();
    void createWorld();

    Player player;

    Model map;
    int cameraMode = CAMERA_FIRST_PERSON;
    int screenWidth = 1200;
    int screenHeight = 800;
    static constexpr unsigned int MAX_COLUMNS = 20;
    float heights[MAX_COLUMNS] = {0};
    Vector3 positions[MAX_COLUMNS] = {0};
    Color colors[MAX_COLUMNS] = {0};
};
// the raylib website https://www.raylib.com/index.html

#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include <string>
#include <cmath>
#include <vector>

using namespace std;  // Allow using string, vector, etc. without std::

// ------------------------------------------------------------
// Student Info
const string studentName = "Tyron Fajardo";
const string studentNumber = "123456789";

// ------------------------------------------------------------
// Simulation parameters
const unsigned int TARGET_FPS = 50;
float timeElapsed = 0;
float dt;

// adjustable parameters via sliders
float launchSpeed = 300;  // pixels per second
float launchAngle = 45;   // degrees
float gravity = 400; // downward acceleration

// ------------------------------------------------------------
// Projectile structure
struct Projectile {
    Vector2 position;
    Vector2 velocity;
    bool active;
};

// container for projectiles
vector<Projectile> projectiles;

// ------------------------------------------------------------
// Launch projectile (Week 2+)
void LaunchProjectile(float speed, float angleDeg)
{
    Projectile p;
    p.position = { 200, (float)GetScreenHeight() - 200 };
    p.velocity = {
        cos(angleDeg * DEG2RAD) * speed,
        -sin(angleDeg * DEG2RAD) * speed
    };
    p.active = true;
    projectiles.push_back(p);
}

// ------------------------------------------------------------
void update()
{
    dt = 1.0f / TARGET_FPS;
    timeElapsed += dt;

    // Fire new projectile on SPACE
    if (IsKeyPressed(KEY_SPACE))
    {
        LaunchProjectile(launchSpeed, launchAngle);
    }

    // Update all projectiles
    for (auto& p : projectiles)
    {
        if (!p.active) continue;

        p.position += p.velocity * dt;
        p.velocity.y += gravity * dt;

        // Week 4: ground collision
        if (p.position.y >= GetScreenHeight() - 50)
        {
            p.position.y = GetScreenHeight() - 50;
            p.velocity.y *= -0.5f; // bounce with damping
            if (fabs(p.velocity.y) < 20)
                p.active = false;
        }
    }
}

// ------------------------------------------------------------
void draw()
{
    BeginDrawing();
    ClearBackground(BLACK);

    // Student info (Week 1)
    DrawText(("Name: " + studentName).c_str(), 10, GetScreenHeight() - 40, 20, LIGHTGRAY);
    DrawText(("Student Number: " + studentNumber).c_str(), 10, GetScreenHeight() - 20, 20, LIGHTGRAY);

    // Elapsed time (Week 1)
    DrawText(TextFormat("Time: %.2f", timeElapsed), GetScreenWidth() - 150, 20, 20, LIGHTGRAY);

    // GUI sliders 
    GuiSliderBar(Rectangle{ 60, 40, 200, 20 }, "Speed", TextFormat("%.0f", launchSpeed), &launchSpeed, 50, 600);
    GuiSliderBar(Rectangle{ 60, 70, 200, 20 }, "Angle", TextFormat("%.0f", launchAngle), &launchAngle, 0, 90);
    GuiSliderBar(Rectangle{ 60, 100, 200, 20 }, "Gravity", TextFormat("%.0f", gravity), &gravity, -600, 600);

    // Launch guide (Week 2)
    Vector2 startPos = { 200, (float)GetScreenHeight() - 200 };
    Vector2 guide = {
        cos(launchAngle * DEG2RAD) * launchSpeed,
        -sin(launchAngle * DEG2RAD) * launchSpeed
    };
    DrawLineEx(startPos, startPos + guide * 0.2f, 3, RED);

    // Projectiles (Week 2–4)
    for (auto& p : projectiles)
    {
        if (p.active)
            DrawCircleV(p.position, 8, RED);
        else
            DrawCircleV(p.position, 8, DARKGRAY);
    }

    EndDrawing();
}

// ------------------------------------------------------------
int main()
{
    InitWindow(1200, 800, ("Game Physics - " + studentName + " " + studentNumber).c_str());
    SetTargetFPS(TARGET_FPS);

    while (!WindowShouldClose())
    {
        update();
        draw();
    }

    CloseWindow();
    return 0;
}
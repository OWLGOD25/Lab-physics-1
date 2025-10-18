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
const string studentNumber = "101542713";

// ------------------------------------------------------------
// Simulation parameters
const unsigned int TARGET_FPS = 50;
float timeElapsed = 0;
float dt;

// adjustable parameters via sliders
float launchSpeed = 300;  // pixels per second
float launchAngle = 45;   // degrees
float gravity = 400; // downward acceleration
float groundY = 700; // y position of the ground

// ------------------------------------------------------------
// Projectile structure
struct Projectile {
    Vector2 position;
    Vector2 velocity;
    bool active;
    Color color;
    float radius = 8.0f;
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
    p.color = RED;
    projectiles.push_back(p);
}

// ------------------------------------------------------------
// Detect Sphere-Sphere Overlap (Week 5 / Ex 3)
bool CheckSphereCollision(const Projectile& a, const Projectile& b)
{
    float dist = Vector2Distance(a.position, b.position);
    return dist < (a.radius + b.radius);
}

// ------------------------------------------------------------
// Detect Sphere–Halfspace Overlap (Ex 4)
bool CheckSphereHalfspace(const Projectile& p, float groundY)
{
    // Sphere overlaps if bottom of sphere goes below the ground
    return (p.position.y + p.radius) > groundY;
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

        // Week 4: ground collision(changed for lab4)
        if (p.position.y + p.radius >= groundY)
        {
            p.position.y = groundY - p.radius;
            p.velocity.y *= -0.5f;
            if (fabs(p.velocity.y) < 20)
                p.active = false;
        }
    }

  // Week 5: Balls Overlap Detection
  for (size_t i = 0; i < projectiles.size(); i++)
  {
      bool overlapping = false;
      for (size_t j = 0; j < projectiles.size(); j++)
      {
          if (i == j) continue;
          if (CheckSphereCollision(projectiles[i], projectiles[j]))
          {
              overlapping = true;
              break;
          }
      }
      projectiles[i].color = overlapping ? RED : LIGHTGRAY;


      // --------------------------------------------------------
      // Ex 4: Balls-Halfspace Overlap (which checks if the balls are touching or under the ground Y)
      bool halfOverlap = CheckSphereHalfspace(projectiles[i], groundY);

      if (overlapping || halfOverlap)
          projectiles[i].color = RED;
      else
          projectiles[i].color = LIGHTGRAY;
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
    DrawText(TextFormat("Time: %.2f  |  FPS: %i", timeElapsed, GetFPS()), GetScreenWidth() - 250, 10, 20, LIGHTGRAY);
   

    // GUI sliders 
    GuiSliderBar(Rectangle{ 60, 40, 200, 20 }, "Speed", TextFormat("%.0f", launchSpeed), &launchSpeed, 50, 600);
    GuiSliderBar(Rectangle{ 60, 70, 200, 20 }, "Angle", TextFormat("%.0f", launchAngle), &launchAngle, 0, 90);
    GuiSliderBar(Rectangle{ 60, 100, 200, 20 }, "Gravity", TextFormat("%.0f", gravity), &gravity, -600, 600);
    GuiSliderBar(Rectangle{ 60, 130, 200, 20 }, "Ground Y", TextFormat("%.0f", groundY), &groundY, 400, (float)GetScreenHeight());

    // Launch guide (Week 2)
    Vector2 startPos = { 200, (float)GetScreenHeight() - 200 };
    Vector2 guide = { cos(launchAngle * DEG2RAD) * launchSpeed, -sin(launchAngle * DEG2RAD) * launchSpeed};
    DrawLineEx(startPos, startPos + guide * 0.2f, 3, RED);

    // Draw Halfspace (ground)
    DrawLine(0, groundY, GetScreenWidth(), groundY, GREEN);

    // Projectiles (Week 2–5)
    for (auto& p : projectiles)
    {
        DrawCircleV(p.position, p.radius, p.color);
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
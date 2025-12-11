// the raylib website https://www.raylib.com/index.html
#define _CRT_SECURE_NO_WARNINGS

#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace std;  // makes life easier maybe? idk I just like coding with it in lol :)

// Section one
// ------------------------------------------------------------
// Student Info
const string studentName = "Tyron Fajardo";
const string studentNumber = "101542713";

// ------------------------------------------------------------
// Simulation parameters
const unsigned int TARGET_FPS = 50;
float timeElapsed = 0.0f;
float dt = 0.0f;

// World constants
const float POS_CORRECT_PERCENT = 0.80f;  // positional correction
const float POS_CORRECT_SLOP = 0.01f;
const float STATIC_VEL_EPS = 0.05f;  // tiny velocity ~ stopped

// ------------------ Adjustable via GUI ------------------
float gravityAcc = 600.0f;   // px/s^2 (down)
float globalRestitution = 0.25f;    // bounciness (0..1)
float globalFrictionCoeff = 0.60f;    // dynamic friction (0..1)
float pigToughness = 250.0f;   // how hard pigs are to kill

float maxSlingshotPower = 900.0f;   // max launch speed
float powerScale = 6.0f;     // power per pixel of drag

// ------------------------------------------------------------
// Slingshot / bird selection

enum ShapeType {
    SHAPE_CIRCLE,
    SHAPE_AABB
};

enum ObjectType {
    OBJ_BIRD,
    OBJ_BLOCK,
    OBJ_PIG,
    OBJ_STATIC_TERRAIN
};

struct Body {
    // physics state
    Vector2 position{ 0.0f, 0.0f };
    Vector2 velocity{ 0.0f, 0.0f };

    // geometry
    float   radius = 8.0f;        // for circles
    Vector2 halfExtents{ 10.0f, 10.0f }; // for AABBs

    // physics properties
    float   mass = 1.0f;
    float   invMass = 1.0f;
    float   restitution = 0.25f;
    float   friction = 0.5f;

    // game properties
    ShapeType  shape = SHAPE_CIRCLE;
    ObjectType type = OBJ_BLOCK;
    Color      color = LIGHTGRAY;
    bool       active = true;   // if false, skip update/draw
    bool       alive = true;   // for pigs

    // pig-specific
    float toughness = 0.0f;
};

vector<Body> bodies;

// Slingshot state
Vector2 slingAnchor{ 200.0f, 550.0f };    // fixed slingshot pivot
bool    isDragging = false;
Vector2 dragStart{ 0.0f, 0.0f };
Vector2 dragEnd{ 0.0f, 0.0f };

int currentBirdType = 0; // 0 = circle, 1 = square

// Ground (static)
float groundY = 700.0f;

// ------------------------------------------------------------
// Math helpers

static inline Vector2 SafeNormalize(const Vector2& v, const Vector2& fallback = { 1.0f, 0.0f }) {
    float len = Vector2Length(v);
    return (len > 1e-6f) ? Vector2Scale(v, 1.0f / len) : fallback;
}

// Clamp alias (from raymath)
static inline float ClampFloat(float x, float minV, float maxV) {
    return Clamp(x, minV, maxV);
}

// Section two
// ------------------------------------------------------------
// Body creation helpers

Body MakeCircle(ObjectType type, Vector2 pos, float radius, float mass, Color color) {
    Body b;
    b.position = pos;
    b.velocity = { 0.0f, 0.0f };
    b.radius = radius;
    b.halfExtents = { radius, radius }; // just for convenience
    b.mass = mass;
    b.invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
    b.restitution = globalRestitution;
    b.friction = globalFrictionCoeff;
    b.shape = SHAPE_CIRCLE;
    b.type = type;
    b.color = color;
    b.active = true;
    b.alive = true;
    b.toughness = (type == OBJ_PIG) ? pigToughness : 0.0f;
    return b;
}

Body MakeAABB(ObjectType type, Vector2 pos, Vector2 halfExtents, float mass, Color color) {
    Body b;
    b.position = pos;
    b.velocity = { 0.0f, 0.0f };
    b.radius = max(halfExtents.x, halfExtents.y); // handy for debug
    b.halfExtents = halfExtents;
    b.mass = mass;
    b.invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
    b.restitution = globalRestitution;
    b.friction = globalFrictionCoeff;
    b.shape = SHAPE_AABB;
    b.type = type;
    b.color = color;
    b.active = true;
    b.alive = true;
    b.toughness = (type == OBJ_PIG) ? pigToughness : 0.0f;
    return b;
}

// Section six
// ------------------------------------------------------------
// Geometry overlap tests (return penetration + contact normal)

// Circle–Circle
bool CircleCircleOverlap(const Body& a, const Body& b,
    float& penetration, Vector2& normal) {
    Vector2 ab = Vector2Subtract(b.position, a.position);
    float dist = Vector2Length(ab);
    float target = a.radius + b.radius;

    if (dist <= 0.0001f) {
        // overlapped almost exactly; choose any normal
        normal = { 1.0f, 0.0f };
        penetration = target;
        return true;
    }
    if (dist >= target) return false;

    normal = Vector2Scale(ab, 1.0f / dist);
    penetration = target - dist;
    return true;
}

// AABB–AABB (axis-aligned, centers at position, halfExtents)
bool AABBAABBOverlap(const Body& a, const Body& b,
    float& penetration, Vector2& normal) {
    Vector2 diff = Vector2Subtract(b.position, a.position);
    float overlapX = a.halfExtents.x + b.halfExtents.x - fabsf(diff.x);
    float overlapY = a.halfExtents.y + b.halfExtents.y - fabsf(diff.y);

    if (overlapX <= 0.0f || overlapY <= 0.0f) return false;

    // collision along axis of least penetration
    if (overlapX < overlapY) {
        penetration = overlapX;
        normal = { (diff.x > 0.0f) ? 1.0f : -1.0f, 0.0f };
    }
    else {
        penetration = overlapY;
        normal = { 0.0f, (diff.y > 0.0f) ? 1.0f : -1.0f };
    }
    return true;
}

// Circle–AABB
bool CircleAABBOverlap(const Body& circle, const Body& box,
    float& penetration, Vector2& normal) {
    // closest point on AABB to circle center
    Vector2 diff = Vector2Subtract(circle.position, box.position);
    Vector2 clamped = {
        Clamp(diff.x, -box.halfExtents.x, box.halfExtents.x),
        Clamp(diff.y, -box.halfExtents.y, box.halfExtents.y)
    };
    Vector2 closest = Vector2Add(box.position, clamped);

    // IMPORTANT: vector from circle -> closest point on box
    Vector2 v = Vector2Subtract(closest, circle.position);
    float dist2 = Vector2LengthSqr(v);
    float r = circle.radius;

    if (dist2 > r * r) return false;

    float dist = sqrtf(dist2);

    if (dist <= 0.0001f) {
        // Circle center is inside/on the box – pick a normal from circle to box center
        Vector2 fallback = Vector2Subtract(box.position, circle.position);
        normal = SafeNormalize(fallback, { 0.0f, 1.0f });
        penetration = r;  // approximate
        return true;
    }

    // normal points from circle -> box (matches ResolveContact assumption)
    normal = Vector2Scale(v, 1.0f / dist);
    penetration = r - dist;
    return true;
}

// Section seven
// ------------------------------------------------------------
// Generic collision resolution (impulse + friction + pig toughness)

void ResolveContact(Body& a, Body& b, float penetration, const Vector2& normal) {
    if (!a.active || !b.active) return;
    if (!a.alive || !b.alive)   return;

    float invA = a.invMass;
    float invB = b.invMass;
    float invSum = invA + invB;
    if (invSum <= 0.0f) return; // two static objects

	// Section eight
    // --- (1) Pig toughness check (use pre-collision momenta) ---
    // approximate "total momentum magnitude" as |m1 v1 - m2 v2|
    Vector2 p1 = Vector2Scale(a.velocity, a.mass);
    Vector2 p2 = Vector2Scale(b.velocity, b.mass);
    float relMomMag = Vector2Length(Vector2Subtract(p1, p2));

    if (a.type == OBJ_PIG && a.alive && relMomMag > a.toughness) {
        a.alive = false;
        a.active = false;
    }
    if (b.type == OBJ_PIG && b.alive && relMomMag > b.toughness) {
        b.alive = false;
        b.active = false;
    }

    // If pig died, still allow their last interaction to push things
    // ---------------------------------------------------------------

    // --- (2) Positional correction ---
    float remove = max(penetration - POS_CORRECT_SLOP, 0.0f) * POS_CORRECT_PERCENT / invSum;
    Vector2 corr = Vector2Scale(normal, remove);
    a.position = Vector2Subtract(a.position, Vector2Scale(corr, invA));
    b.position = Vector2Add(b.position, Vector2Scale(corr, invB));

    // --- (3) Velocity impulse (normal) ---
    Vector2 rv = Vector2Subtract(b.velocity, a.velocity);
    float velAlongNormal = Vector2DotProduct(rv, normal);
    if (velAlongNormal > 0.0f) return; // already separating

    float e = min(a.restitution, b.restitution);
    float j = -(1.0f + e) * velAlongNormal;
    j /= invSum;

    Vector2 impulse = Vector2Scale(normal, j);
    a.velocity = Vector2Subtract(a.velocity, Vector2Scale(impulse, invA));
    b.velocity = Vector2Add(b.velocity, Vector2Scale(impulse, invB));

    // --- (4) Friction impulse (Coulomb, dynamic only for simplicity) ---
    rv = Vector2Subtract(b.velocity, a.velocity);
    Vector2 tangent = SafeNormalize(Vector2Subtract(rv, Vector2Scale(normal, Vector2DotProduct(rv, normal))),
        { -normal.y, normal.x });
    float vt = Vector2DotProduct(rv, tangent);
    if (fabsf(vt) < STATIC_VEL_EPS) return; // almost no tangential motion

    float mu = 0.5f * (a.friction + b.friction);
    float jt = -vt / invSum;
    float maxFriction = mu * j;

    jt = ClampFloat(jt, -maxFriction, maxFriction);
    Vector2 frictionImpulse = Vector2Scale(tangent, jt);
    a.velocity = Vector2Subtract(a.velocity, Vector2Scale(frictionImpulse, invA));
    b.velocity = Vector2Add(b.velocity, Vector2Scale(frictionImpulse, invB));
}

// Section three
// ------------------------------------------------------------
// World setup

void BuildWorld() {
    bodies.clear();

    // Ground (big static AABB)
    {
        Vector2 pos = { (float)GetScreenWidth() * 0.5f, groundY + 20.0f };
        Vector2 half = { (float)GetScreenWidth(), 40.0f };
        Body ground = MakeAABB(OBJ_STATIC_TERRAIN, pos, half, 0.0f, DARKGREEN);
        ground.restitution = 0.2f;
        ground.friction = 0.9f;
        bodies.push_back(ground);
    }

    // Fort blocks (3+ blocks high)
    // Simple tower near right side
    Vector2 basePos = { 850.0f, groundY - 25.0f };
    Vector2 halfBlock{ 25.0f, 25.0f };
    float blockMass = 4.0f;

    int cols = 3;
    int rows = 4;

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            Vector2 pos = {
                basePos.x + (x - (cols / 2)) * (halfBlock.x * 2.2f),
                basePos.y - y * (halfBlock.y * 2.05f)
            };
            Body block = MakeAABB(OBJ_BLOCK, pos, halfBlock, blockMass, BROWN);
            bodies.push_back(block);
        }
    }

    // Pigs (circles) on top and inside fort
    {
        // on top
        Vector2 pigPosTop = { basePos.x, basePos.y - rows * (halfBlock.y * 2.1f) - 20.0f };
        Body pigTop = MakeCircle(OBJ_PIG, pigPosTop, 15.0f, 1.5f, GREEN);
        bodies.push_back(pigTop);

        // inside fort (middle row)
        Vector2 pigInside = { basePos.x, basePos.y - 1.5f * (halfBlock.y * 2.0f) };
        Body pigIn = MakeCircle(OBJ_PIG, pigInside, 15.0f, 1.5f, GREEN);
        bodies.push_back(pigIn);
    }
}

// Create a bird at the slingshot anchor
void SpawnBird(const Vector2& velocity) {
    Body bird;
    if (currentBirdType == 0) {
        // Circular light bird
        bird = MakeCircle(OBJ_BIRD, slingAnchor, 12.0f, 1.0f, YELLOW);
    }
    else {
        // Square heavy bird
        bird = MakeAABB(OBJ_BIRD, slingAnchor, { 14.0f, 14.0f }, 4.0f, RED);
    }
    bird.velocity = velocity;
    bodies.push_back(bird);
}

// Section four
// ------------------------------------------------------------
// Input handling (slingshot + bird switching)

void HandleSlingshotInput() {
    Vector2 mouse = GetMousePosition();

    // Switch bird type (TAB)
    if (IsKeyPressed(KEY_TAB)) {
        currentBirdType = 1 - currentBirdType;
    }

    // Reset world (R)
    if (IsKeyPressed(KEY_R)) {
        BuildWorld();
    }

    // Start drag near slingshot anchor
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        float dist = Vector2Distance(mouse, slingAnchor);
        if (dist < 60.0f) {
            isDragging = true;
            dragStart = slingAnchor;
            dragEnd = mouse;
        }
    }

    if (isDragging) {
        dragEnd = mouse;

        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
            // Compute launch velocity from drag
            Vector2 dragVec = Vector2Subtract(dragStart, dragEnd); // pull back from anchor
            float dragLen = Vector2Length(dragVec);
            if (dragLen > 5.0f) {
                float clampedLen = ClampFloat(dragLen, 0.0f, maxSlingshotPower / powerScale);
                Vector2 dir = Vector2Scale(dragVec, 1.0f / dragLen);
                float speed = clampedLen * powerScale;
                Vector2 vel = Vector2Scale(dir, speed);
                SpawnBird(vel);
            }
            isDragging = false;
        }
    }
}

// Section five
// ------------------------------------------------------------
// Physics update

void UpdatePhysics() {
    // Integrate velocities & positions
    for (auto& b : bodies) {
        if (!b.active) continue;
        if (b.invMass == 0.0f) continue; // static

        // gravity
        b.velocity.y += gravityAcc * dt;

        // integrate
        b.position.x += b.velocity.x * dt;
        b.position.y += b.velocity.y * dt;
    }

    // Collision detection & response (all pairs)
    const size_t n = bodies.size();
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            Body& a = bodies[i];
            Body& b = bodies[j];
            if (!a.active || !b.active) continue;

            float penetration = 0.0f;
            Vector2 normal{ 0.0f, 0.0f };
            bool overlapped = false;

            if (a.shape == SHAPE_CIRCLE && b.shape == SHAPE_CIRCLE) {
                overlapped = CircleCircleOverlap(a, b, penetration, normal);
            }
            else if (a.shape == SHAPE_AABB && b.shape == SHAPE_AABB) {
                overlapped = AABBAABBOverlap(a, b, penetration, normal);
            }
            else if (a.shape == SHAPE_CIRCLE && b.shape == SHAPE_AABB) {
                overlapped = CircleAABBOverlap(a, b, penetration, normal);
            }
            else if (a.shape == SHAPE_AABB && b.shape == SHAPE_CIRCLE) {
                overlapped = CircleAABBOverlap(b, a, penetration, normal);
                normal = Vector2Scale(normal, -1.0f); // flip to a->b
            }

            if (overlapped) {
                ResolveContact(a, b, penetration, normal);
            }
        }
    }

    // Small damping for sleeping objects
    for (auto& b : bodies) {
        if (!b.active || b.invMass == 0.0f) continue;
        if (fabsf(b.velocity.x) < 0.02f && fabsf(b.velocity.y) < 0.02f) {
            b.velocity = { 0.0f, 0.0f };
        }
    }
}

// ------------------------------------------------------------
void update() {
    dt = 1.0f / TARGET_FPS;
    timeElapsed += dt;

    // Update pig toughness & friction/restitution into new bodies too
    // (for simplicity, some properties are applied when building world or spawning birds)

    HandleSlingshotInput();
    UpdatePhysics();
}

// ------------------------------------------------------------
// Drawing helpers

void DrawBody(const Body& b) {
    if (!b.active) return;

    if (b.shape == SHAPE_CIRCLE) {
        Color c = b.color;
        if (b.type == OBJ_PIG) {
            c = b.alive ? GREEN : DARKGREEN;
        }
        DrawCircleV(b.position, b.radius, c);
    }
    else {
        // AABB
        Rectangle rect;
        rect.width = b.halfExtents.x * 2.0f;
        rect.height = b.halfExtents.y * 2.0f;
        rect.x = b.position.x - b.halfExtents.x;
        rect.y = b.position.y - b.halfExtents.y;

        DrawRectangleRec(rect, b.color);
    }
}

void DrawSlingshot() {
    // Stand
    DrawCircleV(slingAnchor, 8.0f, DARKBROWN);
    DrawRectangle(slingAnchor.x - 6, slingAnchor.y, 12, 80, DARKBROWN);

    // Current bird preview
    if (currentBirdType == 0) {
        DrawCircleV(slingAnchor, 12.0f, YELLOW);
    }
    else {
        Rectangle r{
            slingAnchor.x - 14.0f,
            slingAnchor.y - 14.0f,
            28.0f, 28.0f
        };
        DrawRectangleRec(r, RED);
    }

    // Drag rubber band
    if (isDragging) {
        DrawLineEx(slingAnchor, dragEnd, 3.0f, DARKGRAY);
        DrawCircleV(dragEnd, 6.0f, GRAY);
    }
}

// ------------------------------------------------------------
void draw() {
    BeginDrawing();
    ClearBackground(BLACK);

    // Student info
    DrawText(("Name: " + studentName).c_str(), 10, GetScreenHeight() - 40, 20, LIGHTGRAY);
    DrawText(("Student Number: " + studentNumber).c_str(), 10, GetScreenHeight() - 20, 20, LIGHTGRAY);

    // Time/FPS
    DrawText(TextFormat("Time: %.2f  |  FPS: %i", timeElapsed, GetFPS()),
        GetScreenWidth() - 260, 10, 20, LIGHTGRAY);

    // ----------------- GUI (two columns) -----------------
    const float colWidth = 240.0f;
    const float colGap = 80.0f;
    const float col1X = 140.0f;
    const float col2X = col1X + colWidth + colGap;
    const float topY = 40.0f;
    const float DY = 30.0f;

    float y1 = topY;
    float y2 = topY;

    // ------- Column 1: core physics -------
    GuiSliderBar({ col1X, y1, colWidth, 20 }, "Gravity",
        TextFormat("%.0f", gravityAcc), &gravityAcc, 100.0f, 1200.0f); y1 += DY;

    GuiSliderBar({ col1X, y1, colWidth, 20 }, "Restitution",
        TextFormat("%.2f", globalRestitution), &globalRestitution, 0.0f, 1.0f); y1 += DY;

    GuiSliderBar({ col1X, y1, colWidth, 20 }, "Friction (mu)",
        TextFormat("%.2f", globalFrictionCoeff), &globalFrictionCoeff, 0.0f, 1.5f); y1 += DY + 10;

    GuiSliderBar({ col1X, y1, colWidth, 20 }, "Pig Toughness",
        TextFormat("%.0f", pigToughness), &pigToughness, 50.0f, 800.0f); y1 += DY + 10;

    // ------- Column 2: slingshot tuning -------
    DrawText("Slingshot", col2X, y2 - 6, 18, LIGHTGRAY); y2 += DY;
    GuiSliderBar({ col2X, y2, colWidth, 20 }, "Max Power",
        TextFormat("%.0f", maxSlingshotPower), &maxSlingshotPower, 200.0f, 1500.0f); y2 += DY;

    GuiSliderBar({ col2X, y2, colWidth, 20 }, "Power Scale",
        TextFormat("%.1f", powerScale), &powerScale, 2.0f, 10.0f); y2 += DY;

    // Bird type label
    const char* birdLabel = (currentBirdType == 0) ? "Bird: Circle (light)" : "Bird: Square (heavy)";
    DrawText(birdLabel, col2X, y2 + 4, 18, YELLOW); y2 += DY;

    // ----------------- Scene drawing -----------------

    // Slingshot
    DrawSlingshot();

    // Bodies
    for (auto& b : bodies) {
        DrawBody(b);
    }

    // Instructions
    DrawText(
        "Controls:\n"
        "  LMB near slingshot: click, drag, release to launch.\n"
        "  TAB: switch bird (circle vs square).\n"
        "  R: reset fort.\n"
        "Notes:\n"
        "  - Pigs (green) die when collision momentum exceeds their Toughness.\n"
        "  - Blocks are AABB, Birds can be Sphere or AABB.\n"
        "  - Collisions use impulses with restitution and friction.",
        20, GetScreenHeight() - 200, 18, GRAY);

    EndDrawing();
}

// ------------------------------------------------------------
int main() {
    InitWindow(1200, 800, ("Game Physics - " + studentName + " " + studentNumber).c_str());
    SetTargetFPS(TARGET_FPS);

    groundY = 700.0f;
    slingAnchor = { 200.0f, groundY - 150.0f };

    BuildWorld();

    while (!WindowShouldClose()) {
        update();
        draw();
    }

    CloseWindow();
    return 0;
}

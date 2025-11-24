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

using namespace std;  // std:: convenience

// ------------------------------------------------------------
// Student Info
const string studentName = "Tyron Fajardo";
const string studentNumber = "101542713";

// ------------------------------------------------------------
// Simulation parameters
const unsigned int TARGET_FPS = 50;
float timeElapsed = 0.0f;
float dt = 0.0f;

// adjustable parameters via sliders
float launchSpeed = 300.0f;   // px/s
float launchAngle = 45.0f;    // degrees
float gravityAcc = 400.0f;   // px/s^2 (down)
float restitutionSS = 0.40f;    // sphere-sphere restitution (0..1)
float restitutionPlane = 0.60f;    // sphere-plane restitution (0..1)

// NEW (Lab 6): Coulomb friction coefficients
// Plane contact friction
float mu_s_plane = 0.6f;  // static friction coefficient for sphere-plane
float mu_k_plane = 0.5f;  // kinetic friction coefficient for sphere-plane
// Sphere-sphere friction
float mu_s_ss = 0.4f;  // static friction coefficient for sphere-sphere
float mu_k_ss = 0.3f;  // kinetic friction coefficient for sphere-sphere

// sphere-sphere positional correction
const float POS_CORRECT_PERCENT = 0.80f; // 80% of penetration per frame
const float POS_CORRECT_SLOP = 0.01f; // tolerance

// thresholds
const float STATIC_VEL_EPS = 0.05f;  // ~stopped along tangent counts as static

// ------------------------------------------------------------
// Projectile (sphere)
struct Projectile {
    Vector2 position{ 0.0f, 0.0f };
    Vector2 velocity{ 0.0f, 0.0f };
    bool    active = false;
    Color   color = LIGHTGRAY;
    float   radius = 8.0f;
    float   mass = 1.0f;
};

// 2D Halfspace (infinite plane) defined by a pivot point and a unit normal.
// We control pivotX/pivotY (translate) and angleDeg (rotate around pivot).
struct Plane2D {
    float angleDeg = 0.0f;  // rotation about pivot
    float pivotX = 0.0f;  // center X
    float pivotY = 700.0f;// center Y
    Color color = GREEN;
};

vector<Projectile> projectiles;

// Two adjustable planes
Plane2D planeA{ 0.0f, 600.0f, 700.0f, GREEN };       // near bottom, flat
Plane2D planeB{ 25.0f, 900.0f, 520.0f, DARKGREEN };  // a tilted ramp

// ------------------------------------------------------------
// Math helpers

// Plane normal from angle: 0° => (0, -1) (upwards; screen Y+ is down)
static inline Vector2 PlaneNormal(float angleDeg) {
    float r = angleDeg * DEG2RAD;
    return Vector2Normalize({ sinf(r), -cosf(r) });
}

// Signed distance from point p to plane (pivot p0, unit normal n):
// d = dot(p - p0, n). Sphere overlaps if (radius - d) > 0.
static inline float SignedDistanceToPlane(const Vector2& p, const Vector2& p0, const Vector2& n) {
    return Vector2DotProduct(Vector2Subtract(p, p0), n);
}

// Sphere-sphere overlap test (returns penetration & normal a->b)
static inline bool SphereSphereOverlap(const Projectile& a, const Projectile& b,
    float& penetration, Vector2& normal) {
    Vector2 ab = Vector2Subtract(b.position, a.position);
    float dist = Vector2Length(ab);
    float target = a.radius + b.radius;

    if (dist <= 0.0001f) { normal = { 1.0f, 0.0f }; penetration = target; return true; }
    if (dist >= target)   return false;

    normal = Vector2Scale(ab, 1.0f / dist); // from a to b
    penetration = target - dist;
    return true;
}

// Safe normalize helper
static inline Vector2 SafeNormalize(const Vector2& v, const Vector2& fallback = { 1.0f, 0.0f }) {
    float len = Vector2Length(v);
    return (len > 1e-6f) ? Vector2Scale(v, 1.0f / len) : fallback;
}

// ------------------------------------------------------------
// Collision response

// -------- Sphere–Plane (with restitution + Coulomb friction) --------
// 1) Separate out of plane along normal by penetration
// 2) Reflect normal component with restitution (bounce)
// 3) Apply friction impulse along tangent (static/kinetic Coulomb)
bool ResolveSphereHalfspace(Projectile& p, const Plane2D& plane) {
    Vector2 n = PlaneNormal(plane.angleDeg);
    Vector2 p0 = { plane.pivotX, plane.pivotY };
    float d = SignedDistanceToPlane(p.position, p0, n);
    float penetration = p.radius - d;

    if (penetration <= 0.0f) return false;

    // (1) Separate out of plane
    p.position = Vector2Add(p.position, Vector2Scale(n, penetration));

    // Decompose velocity into normal/tangent
    float vn = Vector2DotProduct(p.velocity, n);              // normal component (scalar)
    Vector2 t = { n.y, -n.x };                                // tangent unit vector
    float vt = Vector2DotProduct(p.velocity, t);              // tangential scalar

    // (2) Normal bounce (only if going into plane)
    float jn_mag = 0.0f;
    if (vn < 0.0f) {
        // v' = v - (1 + e) * vn * n
        float delta = (1.0f + restitutionPlane) * vn;         // vn is negative
        p.velocity = Vector2Subtract(p.velocity, Vector2Scale(n, delta));
        jn_mag = -(1.0f + restitutionPlane) * vn * p.mass;    // normal impulse magnitude (positive)
    }

    // (3) Coulomb friction impulse along tangent
    if (jn_mag > 0.0f) {
        // desired tangential impulse to kill vt
        float jt_req = -vt * p.mass; // plane is static -> effective mass = m
        float jt_cap_static = mu_s_plane * jn_mag;
        float jt_cap_kinetic = mu_k_plane * jn_mag;

        float jt = 0.0f;
        if (fabsf(vt) < STATIC_VEL_EPS && fabsf(jt_req) <= jt_cap_static) {
            // Static friction can fully stop tangential motion
            jt = jt_req;
        }
        else {
            // Kinetic friction clamps to mu_k * jn
            jt = Clamp(jt_req, -jt_cap_kinetic, jt_cap_kinetic);
        }
        // Apply tangential impulse
        p.velocity = Vector2Add(p.velocity, Vector2Scale(t, jt / p.mass));
    }

    return true;
}

// -------- Sphere–Sphere (impulse + positional correction + friction) --------
void ResolveSphereSphere(Projectile& a, Projectile& b, float e /*restitutionSS*/) {
    float penetration; Vector2 n; // normal from a->b
    if (!SphereSphereOverlap(a, b, penetration, n)) return;

    float invA = (a.mass > 0.0f) ? 1.0f / a.mass : 0.0f;
    float invB = (b.mass > 0.0f) ? 1.0f / b.mass : 0.0f;
    float invSum = invA + invB;
    if (invSum <= 0.0f) return;

    // (A) Positional correction (to prevent sinking)
    float remove = max(penetration - POS_CORRECT_SLOP, 0.0f) * POS_CORRECT_PERCENT / invSum;
    Vector2 corr = Vector2Scale(n, remove);
    a.position = Vector2Subtract(a.position, Vector2Scale(corr, invA));
    b.position = Vector2Add(b.position, Vector2Scale(corr, invB));

    // Relative velocity at contact
    Vector2 rv = Vector2Subtract(b.velocity, a.velocity);

    // (B) Normal impulse (bounce)
    float velAlongNormal = Vector2DotProduct(rv, n);
    if (velAlongNormal < 0.0f) {
        float jn = -(1.0f + e) * velAlongNormal;
        jn /= invSum;

        Vector2 impulseN = Vector2Scale(n, jn);
        a.velocity = Vector2Subtract(a.velocity, Vector2Scale(impulseN, invA));
        b.velocity = Vector2Add(b.velocity, Vector2Scale(impulseN, invB));

        // (C) Friction impulse along tangent
        // Recompute relative velocity after normal impulse
        rv = Vector2Subtract(b.velocity, a.velocity);
        Vector2 t = SafeNormalize(Vector2Subtract(rv, Vector2Scale(n, Vector2DotProduct(rv, n))), { n.y, -n.x });
        float vt = Vector2DotProduct(rv, t);

        // desired friction impulse to kill vt
        float jt_req = -vt / invSum; // effective mass = 1/(invA+invB)
        float jt_cap_static = mu_s_ss * jn;
        float jt_cap_kinetic = mu_k_ss * jn;

        float jt = 0.0f;
        if (fabsf(vt) < STATIC_VEL_EPS && fabsf(jt_req) <= jt_cap_static) {
            jt = jt_req; // static friction
        }
        else {
            jt = Clamp(jt_req, -jt_cap_kinetic, jt_cap_kinetic); // kinetic friction
        }

        Vector2 impulseT = Vector2Scale(t, jt);
        a.velocity = Vector2Subtract(a.velocity, Vector2Scale(impulseT, invA));
        b.velocity = Vector2Add(b.velocity, Vector2Scale(impulseT, invB));
    }
}

// ------------------------------------------------------------
// Launch projectile
void LaunchProjectile(float speed, float angleDeg) {
    Projectile p;
    p.position = { 200.0f, (float)GetScreenHeight() - 200.0f };
    p.velocity = { cosf(angleDeg * DEG2RAD) * speed, -sinf(angleDeg * DEG2RAD) * speed };
    p.active = true;
    p.color = LIGHTGRAY;
    p.mass = 1.0f;
    projectiles.push_back(p);
}

// ------------------------------------------------------------
static void DrawPlane(const Plane2D& pl) {
    Vector2 n = PlaneNormal(pl.angleDeg);
    Vector2 t = { n.y, -n.x };  // tangent
    Vector2 p0 = { pl.pivotX, pl.pivotY };

    const float span = 4000.0f;
    const float thickness = 3.0f;
    const float normalLen = 45.0f;

    Vector2 a = Vector2Add(p0, Vector2Scale(t, -span));
    Vector2 b = Vector2Add(p0, Vector2Scale(t, span));
    DrawLineEx(a, b, thickness, pl.color);

    // Normal arrow
    Vector2 mid = Vector2Lerp(a, b, 0.5f);
    Vector2 tip = Vector2Add(mid, Vector2Scale(n, normalLen));
    DrawLineEx(mid, tip, thickness * 0.7f, Fade(pl.color, 0.85f));
    DrawCircleV(tip, thickness * 0.7f, pl.color);

    // Pivot dot
    DrawCircleV(p0, thickness, pl.color);
}

// ------------------------------------------------------------
void update() {
    dt = 1.0f / TARGET_FPS;
    timeElapsed += dt;

    if (IsKeyPressed(KEY_SPACE))
        LaunchProjectile(launchSpeed, launchAngle);

    // Optional: spawn at mouse
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        Projectile p;
        p.position = GetMousePosition();
        p.velocity = { 0.0f, 0.0f };
        p.active = true;
        p.color = LIGHTGRAY;
        p.mass = 1.0f;
        projectiles.push_back(p);
    }

    // Integrate
    for (auto& p : projectiles) {
        if (!p.active) continue;
        p.velocity.y += gravityAcc * dt;
        p.position.x += p.velocity.x * dt;
        p.position.y += p.velocity.y * dt;
    }

    // Base color each frame
    for (auto& p : projectiles) p.color = LIGHTGRAY;

    // Sphere–Sphere response (do all pairs)
    for (size_t i = 0; i < projectiles.size(); ++i) {
        for (size_t j = i + 1; j < projectiles.size(); ++j) {
            ResolveSphereSphere(projectiles[i], projectiles[j], restitutionSS);
        }
    }
    // Color pairs still overlapping (feedback)
    for (size_t i = 0; i < projectiles.size(); ++i) {
        for (size_t j = i + 1; j < projectiles.size(); ++j) {
            float pen; Vector2 n;
            if (SphereSphereOverlap(projectiles[i], projectiles[j], pen, n)) {
                projectiles[i].color = RED;
                projectiles[j].color = RED;
            }
        }
    }

    // Sphere–Plane response (both planes)
    for (auto& p : projectiles) {
        bool hitA = ResolveSphereHalfspace(p, planeA);
        bool hitB = ResolveSphereHalfspace(p, planeB);
        if (hitA || hitB) p.color = RED;

        // settle threshold to kill micro-jitter
        if (fabsf(p.velocity.x) < 0.02f && fabsf(p.velocity.y) < 0.02f)
            p.velocity = { 0.0f, 0.0f };
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
    const float colGap = 100.0f;
    const float col1X = 150.0f;
    const float col2X = col1X + colWidth + colGap;
    const float topY = 40.0f;
    const float DY = 30.0f;

    float y1 = topY;
    float y2 = topY;

    // ------- Column 1: core projectile & plane friction -------

    GuiSliderBar({ col1X, y1, colWidth, 20 }, "Speed",
        TextFormat("%.0f", launchSpeed), &launchSpeed, 50, 600); y1 += DY;
    GuiSliderBar({ col1X, y1, colWidth, 20 }, "Angle",
        TextFormat("%.0f", launchAngle), &launchAngle, 0, 90); y1 += DY;
    GuiSliderBar({ col1X, y1, colWidth, 20 }, "Gravity",
        TextFormat("%.0f", gravityAcc), &gravityAcc, -1200, 1200); y1 += DY + 6;

    GuiSliderBar({ col1X, y1, colWidth, 20 }, "Restitution (S-S)",
        TextFormat("%.2f", restitutionSS), &restitutionSS, 0.0f, 1.0f); y1 += DY;
    GuiSliderBar({ col1X, y1, colWidth, 20 }, "Restitution (S-Plane)",
        TextFormat("%.2f", restitutionPlane), &restitutionPlane, 0.0f, 1.0f); y1 += DY + 10;

    // Plane friction checkbox + sliders
    DrawText("Friction (Plane)", col1X, y1 - 6, 18, LIGHTGRAY); y1 += DY;
    GuiSliderBar({ col1X, y1, colWidth, 20 }, "mu_s (static)",
        TextFormat("%.2f", mu_s_plane), &mu_s_plane, 0.0f, 1.5f); y1 += DY;
    GuiSliderBar({ col1X, y1, colWidth, 20 }, "mu_k (kinetic)",
        TextFormat("%.2f", mu_k_plane), &mu_k_plane, 0.0f, 1.5f); y1 += DY + 10;

    // Plane A controls
    DrawText("Plane A (Ground)", col1X, y1 - 6, 18, LIGHTGRAY); y1 += DY;
    GuiSliderBar({ col1X, y1, colWidth, 20 }, "Pos X",
        TextFormat("%.0f", planeA.pivotX), &planeA.pivotX, 0, (float)GetScreenWidth()); y1 += DY;
    GuiSliderBar({ col1X, y1, colWidth, 20 }, "Pos Y",
        TextFormat("%.0f", planeA.pivotY), &planeA.pivotY, 0, (float)GetScreenHeight()); y1 += DY;
    GuiSliderBar({ col1X, y1, colWidth, 20 }, "Angle",
        TextFormat("%.0f", planeA.angleDeg), &planeA.angleDeg, -90.0f, 90.0f); y1 += DY;

    // ------- Column 2: sphere–sphere friction & Plane B -------


    DrawText("Friction (Sphere-Sphere)", col2X, y2 - 6, 18, LIGHTGRAY); y2 += DY;
    GuiSliderBar({ col2X, y2, colWidth, 20 }, "mu_s (static)",
        TextFormat("%.2f", mu_s_ss), &mu_s_ss, 0.0f, 1.5f); y2 += DY;
    GuiSliderBar({ col2X, y2, colWidth, 20 }, "mu_k (kinetic)",
        TextFormat("%.2f", mu_k_ss), &mu_k_ss, 0.0f, 1.5f); y2 += DY + 10;

    // Plane B controls
    DrawText("Plane B (Ramp)", col2X, y2 - 6, 18, LIGHTGRAY); y2 += DY;
    GuiSliderBar({ col2X, y2, colWidth, 20 }, "Pos X",
        TextFormat("%.0f", planeB.pivotX), &planeB.pivotX, 0, (float)GetScreenWidth()); y2 += DY;
    GuiSliderBar({ col2X, y2, colWidth, 20 }, "Pos Y",
        TextFormat("%.0f", planeB.pivotY), &planeB.pivotY, 0, (float)GetScreenHeight()); y2 += DY;
    GuiSliderBar({ col2X, y2, colWidth, 20 }, "Angle",
        TextFormat("%.0f", planeB.angleDeg), &planeB.angleDeg, -90.0f, 90.0f); y2 += DY;

    // ----------------- Scene drawing -----------------

    // Launch guide
    Vector2 startPos = { 200.0f, (float)GetScreenHeight() - 200.0f };
    Vector2 guide = { cosf(launchAngle * DEG2RAD) * launchSpeed,
                         -sinf(launchAngle * DEG2RAD) * launchSpeed };
    DrawLineEx(startPos, Vector2Add(startPos, Vector2Scale(guide, 0.2f)), 3, RED);

    // Draw planes
    DrawPlane(planeA);
    DrawPlane(planeB);

    // Draw spheres
    for (auto& p : projectiles)
        DrawCircleV(p.position, p.radius, p.color);

    // Instructions text at bottom
    DrawText("SPACE: launch | LMB: spawn at mouse\n"
        "Lab 6: Toggle friction ON/OFF for Plane and Sphere–Sphere to A/B compare.\n"
        "Adjust mu_s/mu_k and restitution to see sliding vs sticking and bounce.",
        20, GetScreenHeight() - 140, 18, GRAY);

    EndDrawing();
}


// ------------------------------------------------------------
int main() {
    InitWindow(1200, 800, ("Game Physics - " + studentName + " " + studentNumber).c_str());
    SetTargetFPS(TARGET_FPS);

    // Nice starting pivots
    planeA.pivotX = GetScreenWidth() * 0.50f;
    planeB.pivotX = GetScreenWidth() * 0.75f;

    while (!WindowShouldClose()) {
        update();
        draw();
    }
    CloseWindow();
    return 0;
}

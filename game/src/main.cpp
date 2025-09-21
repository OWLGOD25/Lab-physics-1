// the raylib website https://www.raylib.com/index.html

#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include <string>
#include <cmath>


using namespace std;  // Allow using string, cout, etc. without std::


// Student Info
const string studentName = "Tyron Fajardo";
const string studentNumber = "123456789";



// Simulation parameters
const unsigned int TARGET_FPS = 50; //1/50 = 0.02
float time = 0;
float dt;
float x = 500;
float y = 500;
float amplitude = 70;
float frequency = 1.0f; // in Hz

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void update()
{
    dt = 1.0 / TARGET_FPS;
	time += dt;
	x = x + (-sin(time * frequency)) * frequency * amplitude * dt;
    y = y + (cos(time * frequency)) * frequency * amplitude * dt;

}

void draw()
{
    BeginDrawing();
    ClearBackground(BLACK);
    DrawText("Hello world!", 10, 10, 20, LIGHTGRAY);
    GuiSliderBar(Rectangle{ 60, (float)GetScreenHeight() -10, 1000, 10 }, "Time", TextFormat("%.2f", time), &time, 0, 240);
    DrawText(TextFormat("FPS: %i, Time: %0.2f", TARGET_FPS, time), GetScreenWidth() - 200, 40, 20, LIGHTGRAY);
	DrawCircle(x, y, 60, RED);
    DrawCircle(GetScreenWidth() / 2 + cos(time * frequency) * amplitude, GetScreenHeight()/ 2 + sin (time * frequency) * amplitude, 60, BLUE);
    DrawText((studentName + " " + studentNumber).c_str(), 10, GetScreenHeight() - 30, 20, LIGHTGRAY);
    EndDrawing();

}

int main()
{
	// alternatively you can use the game.h file to declare constants
	// like this InitWindow(InitialWidth, InitialHeight, "Lab-Physics-1");
	InitWindow(1200, 800, ("Game Physics - " + studentName + " " + studentNumber).c_str());// you can add the game.h file and use it to declare functions and variables
	//which is the numbers in the brackets
    SetTargetFPS(TARGET_FPS);

    while (!WindowShouldClose())
    {
        update();
		draw();
    }

    CloseWindow();
    return 0;
}

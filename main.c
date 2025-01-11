#include <immintrin.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "PhysicsObjects.h"
#include "raylib.h"

#define OBJECT_COUNT (50*50)
#define NUM_THREADS 4
#define OBJECT_RADIUS 20

void MoveBall(PhysicsObjects *objects) {
    Vector2 speed = {0.0f, 0.0f};

    if (IsKeyDown(KEY_W)) {
        speed.y += 5.0f;
    }
    if (IsKeyDown(KEY_S)) {
        speed.y += -5.0f;
    }
    if (IsKeyDown(KEY_A)) {
        speed.x += -5.0f;
    }
    if (IsKeyDown(KEY_D)) {
        speed.x += 5.0f;
    }

    float length = sqrtf(speed.x * speed.x + speed.y * speed.y);
    if (length != 0) {
        speed.x /= length;
        speed.y /= length;
    }

    objects->xPositions[0] += speed.x;
    objects->yPositions[0] += speed.y;
}

int main() {
    srand(time(NULL));
    PhysicsObjects *objects = GridSpawn(OBJECT_COUNT, 1600, 900);
    objects->masses[0] = 100.0f;
    objects->radii[0] = 25;

    InitWindow(1600, 1200, "Vectorization Practice 1");
    SetTargetFPS(1000);

    while (!WindowShouldClose()) {
        MoveBall(objects);

        UpdatePhysicsObjects2(objects, GetFrameTime());

        BeginDrawing();
        ClearBackground(BLACK);

        DrawCircle((int) objects->xPositions[0], 1200 - (int) objects->yPositions[0], objects->radii[0], BLUE);
        for (int i = 1; i < objects->size; i++) {
            DrawCircle((int) objects->xPositions[i], 1200 - (int) objects->yPositions[i], objects->radii[i],RED);
        }

        EndDrawing();
    }


    return 0;
}

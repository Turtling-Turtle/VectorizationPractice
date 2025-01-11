#include <immintrin.h>
#include "PhysicsObjects.h"
#include <stdlib.h>
#include <math.h>

#define GRAVITY 0
#define FRICTION_COEFFICIENT 0.8f
#define RESTITUTION 0.8f
#define DAMPING 0.98f

PhysicsObjects *CreatePhysicsObjects(int size) {
    if (size <= 0) return NULL;

    PhysicsObjects *objects = malloc(sizeof(PhysicsObjects));
    if (!objects) {
        free(objects);
        return NULL;
    }
    objects->size = size;
    objects->xPositions = (float *) _aligned_malloc(size * sizeof(float), 32);
    objects->yPositions = (float *) _aligned_malloc(size * sizeof(float), 32);
    objects->xVelocities = (float *) _aligned_malloc(size * sizeof(float), 32);
    objects->yVelocities = (float *) _aligned_malloc(size * sizeof(float), 32);
    objects->xAccelerations = (float *) _aligned_malloc(size * sizeof(float), 32);
    objects->yAccelerations = (float *) _aligned_malloc(size * sizeof(float), 32);
    objects->masses = (float *) _aligned_malloc(size * sizeof(float), 32);
    objects->radii = (float *) _aligned_malloc(size * sizeof(float), 32);
    if (!objects->xPositions || !objects->yPositions || !objects->xVelocities || !objects->yVelocities || !objects->
        masses || !objects->radii) {
        DestroyPhysicsObjects(objects);
        return NULL;
    }

    for (int i = 0; i < size; i++) {
        objects->xPositions[i] = 0;
        objects->yPositions[i] = 0;
        objects->xVelocities[i] = 0;
        objects->yVelocities[i] = 0;
        objects->xAccelerations[i] = 0;
        objects->yAccelerations[i] = 0;
        objects->masses[i] = 1;
        objects->radii[i] = 20;
    }

    return objects;
}

void DestroyPhysicsObjects(PhysicsObjects *objects) {
    if (!objects) return;

    free(objects->xPositions);
    free(objects->yPositions);

    free(objects->xVelocities);
    free(objects->yVelocities);

    free(objects->xAccelerations);
    free(objects->yAccelerations);

    free(objects->radii);
    free(objects->masses);

    free(objects);
}

void UpdatePhysicsObjects(PhysicsObjects *objects, float deltaTime) {
    int i;
    __m256 deltaT = _mm256_set1_ps(deltaTime);
    for (i = 0; i + 8 <= objects->size; i += 8) {
        //Update Accelerations


        //Update Velocities
        __m256 xVelocities = _mm256_load_ps(&objects->xVelocities[i]);
        __m256 xAccelerations = _mm256_load_ps(&objects->xAccelerations[i]);
        xVelocities = _mm256_add_ps(xVelocities, _mm256_mul_ps(xAccelerations, deltaT));

        __m256 yVelocities = _mm256_load_ps(&objects->yVelocities[i]);
        __m256 yAccelerations = _mm256_load_ps(&objects->yAccelerations[i]);
        yVelocities = _mm256_add_ps(yVelocities, _mm256_mul_ps(yAccelerations, deltaT));

        _mm256_store_ps(&objects->xVelocities[i], xVelocities);
        _mm256_store_ps(&objects->yVelocities[i], yVelocities);

        //Update positions
        __m256 xPositions = _mm256_load_ps(&objects->xPositions[i]);
        xPositions = _mm256_add_ps(xPositions, _mm256_mul_ps(xVelocities, deltaT));

        __m256 yPositions = _mm256_load_ps(&objects->xPositions[i]);
        yPositions = _mm256_add_ps(yPositions, _mm256_mul_ps(yVelocities, deltaT));

        _mm256_store_ps(&objects->xPositions[i], xPositions);
        _mm256_store_ps(&objects->yPositions[i], yPositions);
    }
}

float ComputeDistance(float x1, float y1, float x2, float y2) {
    return sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

float ComputeMagnitude(float x1, float y1) {
    return sqrtf(x1 * x1 + y1 * y1);
}

void NormalizeVector2(float *x, float *y) {
    float magnitude = ComputeMagnitude(*x, *y);
    if (magnitude != 0) {
        *x /= magnitude;
        *y /= magnitude;
    }
}

void SolveCollisions(PhysicsObjects *objects, int obj1, int obj2) {
    if (objects->masses[obj1] <= 0 && objects->masses[obj2] <= 0) return; //mass of 0 means its static/doesn't move.

    float x1 = objects->xPositions[obj1], y1 = objects->yPositions[obj1];
    float x2 = objects->xPositions[obj2], y2 = objects->yPositions[obj2];
    float normalX = x1 - x2;
    float normalY = y1 - y2;
    NormalizeVector2(&normalX, &normalY);
    float rad = objects->radii[obj1] + objects->radii[obj2];
    float depth = rad - ComputeDistance(x1, y1, x2, y2);


    float totalMass = objects->masses[obj1] + objects->masses[obj2];
    if (objects->masses[obj1] > 0) {
        objects->xPositions[obj1] += normalX * (depth * (objects->masses[obj2] / (totalMass)));
        objects->yPositions[obj1] += normalY * (depth * (objects->masses[obj2] / (totalMass)));
    }

    if (objects->masses[obj2] > 0) {
        objects->xPositions[obj2] -= normalX * (depth * (objects->masses[obj1] / (totalMass)));
        objects->yPositions[obj2] -= normalY * (depth * (objects->masses[obj1] / (totalMass)));
    }
}

int Intersects(PhysicsObjects *objects, int i, int j) {
    float distance = ComputeDistance(objects->xPositions[i], objects->yPositions[i], objects->xPositions[j], objects->yPositions[j]);
    return distance <= objects->radii[i] + objects->radii[j];
}

PhysicsObjects *GridSpawn(int count, int width, int height) {
    PhysicsObjects *objects = CreatePhysicsObjects(count);
    if (!objects) {
        DestroyPhysicsObjects(objects);
        return NULL;
    }

    int rows = (int) sqrt(count);
    int cols = (count + rows - 1) / rows;
    float spacingX = (float) width / cols;
    float spacingY = (float) height / rows;
    float radius = 12.0f;

    for (int i = 0; i < count; i++) {
        int row = i / cols;
        int col = i % cols;

        objects->xPositions[i] = (col + 0.5f) * spacingX;
        objects->yPositions[i] = (row + 0.5f) * spacingY;

        objects->xVelocities[i] = 0.0f;
        objects->yVelocities[i] = 0.0f;

        objects->xAccelerations[i] = 0.0f;
        objects->yAccelerations[i] = 0.0f;

        objects->radii[i] = radius;
    }

    return objects;
}

//Compiler loves this version
void UpdatePhysicsObjects2(PhysicsObjects *objects, float deltaTime) {
    for (int i = 0; i < objects->size; ++i) {
        if (objects->masses[i] <= 1) {
            continue;
        }
        objects->yAccelerations[i] += GRAVITY * (1 / objects->masses[i]);

        float dragX = 0, dragY = 0;
        if (objects->xVelocities[i] != 0 || objects->yVelocities[i] != 0) {
            float drag = 0.47f;
            float velocityMag = ComputeMagnitude(objects->xVelocities[i], objects->yVelocities[i]);
            float unitVelocityX = objects->xVelocities[i], unitVelocityY = objects->yVelocities[i];
            NormalizeVector2(&unitVelocityX, &unitVelocityY);
            float dragForceX = unitVelocityX * (-drag * (velocityMag * velocityMag));
            float dragForceY = unitVelocityY * (-drag * (velocityMag * velocityMag));
            dragX += dragForceX * (1 / objects->masses[i]);
            dragY += dragForceY * (1 / objects->masses[i]);
        }

        objects->xVelocities[i] += (objects->xAccelerations[i] + dragX) * deltaTime;
        objects->yVelocities[i] += (objects->yAccelerations[i] + dragY) * deltaTime;

        objects->xPositions[i] += objects->xVelocities[i] * deltaTime;
        objects->yPositions[i] += objects->yVelocities[i] * deltaTime;
    }

    for (int i = 0; i < objects->size; i++) {
        for (int j = i + 1; j < objects->size; j++) {
            if (Intersects(objects, i, j)) {
                SolveCollisions(objects, i, j);
            }
        }
    }

}


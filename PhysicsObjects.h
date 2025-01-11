#ifndef PHYSICS_OBJECTS_H
#define PHYSICS_OBJECTS_H

typedef struct {
    float *xPositions;
    float *yPositions;

    float *xVelocities;
    float *yVelocities;

    float *xAccelerations;
    float *yAccelerations;

    float *masses;

    float *radii;

    int size;
} PhysicsObjects;

PhysicsObjects *CreatePhysicsObjects(int size);

void DestroyPhysicsObjects(PhysicsObjects *objects);

void UpdatePhysicsObjects(PhysicsObjects *objects, float deltaTime);

void UpdatePhysicsObjects2(PhysicsObjects *objects, float deltaTime);

PhysicsObjects *GridSpawn(int count, int width, int height);

#endif // PHYSICS_OBJECTS_H

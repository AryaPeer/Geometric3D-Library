#ifndef RAY_TRIANGLE_INTERSECTION_H
#define RAY_TRIANGLE_INTERSECTION_H

#include <cuda_runtime.h>

struct Ray {
    float3 origin;
    float3 direction;
};

struct Triangle {
    float3 v0;
    float3 v1;
    float3 v2;
};

struct Intersection {
    bool hit;
    float t;
    float u;
    float v;
};


/**
 * @brief Performs ray-triangle intersection tests using CUDA.
 *
 * @param rays Array of rays.
 * @param triangles Array of triangles.
 * @param intersections Output array of intersections.
 * @param numTests Number of ray-triangle pairs.
 */
void performRayTriangleIntersections(
    Ray* rays,
    Triangle* triangles,
    Intersection* intersections,
    int numTests);

#endif // RAY_TRIANGLE_INTERSECTION_H

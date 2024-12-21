#include "RayTriangleIntersection.h"
#include <iostream>

#define BLOCK_SIZE 256

// CUDA kernel for ray-triangle intersection
__global__ void rayTriangleIntersectKernel(
    Ray* rays,
    Triangle* triangles,
    Intersection* intersections,
    int numTests)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < numTests) {
        const float EPSILON = 1e-8f;
        Ray ray = rays[idx];
        Triangle tri = triangles[idx];

        float3 edge1 = make_float3(
            tri.v1.x - tri.v0.x,
            tri.v1.y - tri.v0.y,
            tri.v1.z - tri.v0.z);

        float3 edge2 = make_float3(
            tri.v2.x - tri.v0.x,
            tri.v2.y - tri.v0.y,
            tri.v2.z - tri.v0.z);

        float3 h = cross(ray.direction, edge2);
        float a = dot(edge1, h);

        if (fabs(a) < EPSILON) {
            intersections[idx].hit = false; 
            return;
        }

        float f = 1.0f / a;
        float3 s = make_float3(
            ray.origin.x - tri.v0.x,
            ray.origin.y - tri.v0.y,
            ray.origin.z - tri.v0.z);

        float u = f * dot(s, h);
        if (u < 0.0f || u > 1.0f) {
            intersections[idx].hit = false;
            return;
        }

        float3 q = cross(s, edge1);
        float v = f * dot(ray.direction, q);
        if (v < 0.0f || u + v > 1.0f) {
            intersections[idx].hit = false;
            return;
        }

        float t = f * dot(edge2, q);
        if (t > EPSILON) { 
            intersections[idx].hit = true;
            intersections[idx].t = t;
            intersections[idx].u = u;
            intersections[idx].v = v;
        } else {
            intersections[idx].hit = false; 
        }
    }
}

void checkCudaError(cudaError_t err, const char* msg) {
    if (err != cudaSuccess) {
        std::cerr << "CUDA Error: " << msg << " - " << cudaGetErrorString(err) << "\n";
        exit(EXIT_FAILURE);
    }
}

void performRayTriangleIntersections(
    Ray* rays,
    Triangle* triangles,
    Intersection* intersections,
    int numTests)
{
    Ray* d_rays = nullptr;
    Triangle* d_triangles = nullptr;
    Intersection* d_intersections = nullptr;

    size_t sizeRays = numTests * sizeof(Ray);
    size_t sizeTriangles = numTests * sizeof(Triangle);
    size_t sizeIntersections = numTests * sizeof(Intersection);

    checkCudaError(cudaMalloc(&d_rays, sizeRays), "Allocating device memory for rays");
    checkCudaError(cudaMalloc(&d_triangles, sizeTriangles), "Allocating device memory for triangles");
    checkCudaError(cudaMalloc(&d_intersections, sizeIntersections), "Allocating device memory for intersections");

    checkCudaError(cudaMemcpy(d_rays, rays, sizeRays, cudaMemcpyHostToDevice), "Copying rays to device");
    checkCudaError(cudaMemcpy(d_triangles, triangles, sizeTriangles, cudaMemcpyHostToDevice), "Copying triangles to device");

    int numBlocks = (numTests + BLOCK_SIZE - 1) / BLOCK_SIZE;
    rayTriangleIntersectKernel<<<numBlocks, BLOCK_SIZE>>>(d_rays, d_triangles, d_intersections, numTests);
    checkCudaError(cudaGetLastError(), "Launching kernel");

    checkCudaError(cudaMemcpy(intersections, d_intersections, sizeIntersections, cudaMemcpyDeviceToHost), "Copying intersections to host");

    cudaFree(d_rays);
    cudaFree(d_triangles);
    cudaFree(d_intersections);
}

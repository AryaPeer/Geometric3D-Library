#ifndef RAY_H
#define RAY_H

#include "Point3D.h"

template <typename T>
class Ray {
public:
    Point3D<T> origin;
    Point3D<T> direction;

    Ray();
    Ray(const Point3D<T>& orig, const Point3D<T>& dir);
};

template <typename T>
bool rayTriangleIntersect(
    const Ray<T>& ray,
    const Point3D<T>& v0,
    const Point3D<T>& v1,
    const Point3D<T>& v2,
    T& t, T& u, T& v);

#endif // RAY_H

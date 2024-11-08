#include "includes/Ray.h"
#include <cmath>

template <typename T>
Ray<T>::Ray()
    : origin(0, 0, 0), direction(0, 0, 1) {}

template <typename T>
Ray<T>::Ray(const Point3D<T>& orig, const Point3D<T>& dir)
    : origin(orig), direction(dir) {}

template <typename T>
bool rayTriangleIntersect(
    const Ray<T>& ray,
    const Point3D<T>& v0,
    const Point3D<T>& v1,
    const Point3D<T>& v2,
    T& t, T& u, T& v)
{
    const T EPSILON = static_cast<T>(1e-8);
    Point3D<T> edge1 = v1 - v0;
    Point3D<T> edge2 = v2 - v0;

    Point3D<T> h = ray.direction.cross(edge2);
    T a = edge1.dot(h);
    if (std::abs(a) < EPSILON)
        return false;

    T f = static_cast<T>(1.0) / a;
    Point3D<T> s = ray.origin - v0;
    u = f * s.dot(h);
    if (u < static_cast<T>(0.0) || u > static_cast<T>(1.0))
        return false;

    Point3D<T> q = s.cross(edge1);
    v = f * ray.direction.dot(q);
    if (v < static_cast<T>(0.0) || u + v > static_cast<T>(1.0))
        return false;

    t = f * edge2.dot(q);
    if (t > EPSILON) 
        return true;
    else 
        return false;
}

template class Ray<float>;
template class Ray<double>;

template bool rayTriangleIntersect<float>(
    const Ray<float>& ray,
    const Point3D<float>& v0,
    const Point3D<float>& v1,
    const Point3D<float>& v2,
    float& t, float& u, float& v);

template bool rayTriangleIntersect<double>(
    const Ray<double>& ray,
    const Point3D<double>& v0,
    const Point3D<double>& v1,
    const Point3D<double>& v2,
    double& t, double& u, double& v);
    
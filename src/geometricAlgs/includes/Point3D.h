#ifndef POINT3D_H
#define POINT3D_H

#include <cmath>

template <typename T>
class Point3D {
public:
    T x, y, z;

    Point3D();
    Point3D(T x, T y, T z);

    Point3D(const Point3D& other);

    Point3D& operator=(const Point3D& other);

    ~Point3D();

    T distanceTo(const Point3D& other) const;

    Point3D operator+(const Point3D& other) const;
    Point3D operator-(const Point3D& other) const;
    Point3D operator*(T scalar) const;
    Point3D operator/(T scalar) const;

    const T* data() const;
};

#endif
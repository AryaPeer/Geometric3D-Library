#ifndef POINT3D_H
#define POINT3D_H

template <typename T>
class Point3D {
public:
    T x, y, z;

    // Constructors
    Point3D();
    Point3D(T xVal, T yVal, T zVal);

    // Operators
    Point3D<T> operator+(const Point3D<T>& other) const;
    Point3D<T> operator-(const Point3D<T>& other) const;
    Point3D<T> operator*(T scalar) const;
    Point3D<T> operator/(T scalar) const;

    T dot(const Point3D<T>& other) const;
    Point3D<T> cross(const Point3D<T>& other) const;

    T magnitude() const;
    Point3D<T> normalized() const;
};

#include "Point3D.inl" 

#endif // POINT3D_H

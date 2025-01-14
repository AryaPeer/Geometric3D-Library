#ifndef POINT3D_H
#define POINT3D_H

#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <iostream>

template <typename T>
class Point3D {
    static_assert(std::is_arithmetic<T>::value, "Point3D requires an arithmetic type.");

public:
    T x, y, z;

    Point3D();
    Point3D(T x, T y, T z);
    Point3D(const Point3D& other);

    Point3D& operator=(const Point3D& other);

    ~Point3D();

    T distanceTo(const Point3D& other) const; // Euclidean distance
    T dot(const Point3D& other) const;       // Dot product
    Point3D cross(const Point3D& other) const; // Cross product
    T norm() const;                          // Magnitude
    Point3D normalized() const;              // Unit vector
    T normSquared() const;                   // Squared magnitude

    Point3D operator+(const Point3D& other) const;
    Point3D& operator+=(const Point3D& other);
    Point3D operator-(const Point3D& other) const;
    Point3D& operator-=(const Point3D& other);
    Point3D operator*(T scalar) const;
    Point3D& operator*=(T scalar);
    Point3D operator/(T scalar) const;
    Point3D& operator/=(T scalar);
    bool operator==(const Point3D& other) const;
    bool operator!=(const Point3D& other) const;
    bool operator<(const Point3D& other) const;

    const T* data() const;

    template <typename U>
    friend std::ostream& operator<<(std::ostream& os, const Point3D<U>& point);
};

#endif // POINT3D_H

#include <cmath>

// Default constructor
template <typename T>
Point3D<T>::Point3D() : x(0), y(0), z(0) {}

// Parameterized constructor
template <typename T>
Point3D<T>::Point3D(T xVal, T yVal, T zVal) : x(xVal), y(yVal), z(zVal) {}

// Addition
template <typename T>
Point3D<T> Point3D<T>::operator+(const Point3D<T>& other) const {
    return Point3D<T>(x + other.x, y + other.y, z + other.z);
}

// Subtraction
template <typename T>
Point3D<T> Point3D<T>::operator-(const Point3D<T>& other) const {
    return Point3D<T>(x - other.x, y - other.y, z - other.z);
}

// Scalar multiplication
template <typename T>
Point3D<T> Point3D<T>::operator*(T scalar) const {
    return Point3D<T>(x * scalar, y * scalar, z * scalar);
}

// Scalar division
template <typename T>
Point3D<T> Point3D<T>::operator/(T scalar) const {
    return Point3D<T>(x / scalar, y / scalar, z / scalar);
}

// Dot product
template <typename T>
T Point3D<T>::dot(const Point3D<T>& other) const {
    return x * other.x + y * other.y + z * other.z;
}

// Cross product
template <typename T>
Point3D<T> Point3D<T>::cross(const Point3D<T>& other) const {
    return Point3D<T>(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x);
}

// Magnitude
template <typename T>
T Point3D<T>::magnitude() const {
    return std::sqrt(x * x + y * y + z * z);
}

// Normalization
template <typename T>
Point3D<T> Point3D<T>::normalized() const {
    T mag = magnitude();
    return Point3D<T>(x / mag, y / mag, z / mag);
}


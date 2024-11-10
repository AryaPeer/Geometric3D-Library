#include "includes/Point3D.h"
#include <stdexcept>

template <typename T>
Point3D<T>::Point3D() : x(T(0)), y(T(0)), z(T(0)) {}

template <typename T>
Point3D<T>::Point3D(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

template <typename T>
Point3D<T>::Point3D(const Point3D<T>& other) : x(other.x), y(other.y), z(other.z) {}

template <typename T>
Point3D<T>& Point3D<T>::operator=(const Point3D<T>& other) {
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
}

// Destructor
template <typename T>
Point3D<T>::~Point3D() {}

// Member Functions
template <typename T>
T Point3D<T>::distanceTo(const Point3D<T>& other) const {
    return (*this - other).norm();
}

template <typename T>
T Point3D<T>::dot(const Point3D<T>& other) const {
    return x * other.x + y * other.y + z * other.z;
}

template <typename T>
Point3D<T> Point3D<T>::cross(const Point3D<T>& other) const {
    return Point3D<T>(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

template <typename T>
T Point3D<T>::norm() const {
    return std::sqrt(x * x + y * y + z * z);
}

template <typename T>
Point3D<T> Point3D<T>::normalized() const {
    T length = norm();
    if (length == T(0)) {
        throw std::runtime_error("Cannot normalize a zero-length vector.");
    }
    return *this / length;
}

// Operator Overloads
template <typename T>
Point3D<T> Point3D<T>::operator+(const Point3D<T>& other) const {
    return Point3D<T>(x + other.x, y + other.y, z + other.z);
}

template <typename T>
Point3D<T>& Point3D<T>::operator+=(const Point3D<T>& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

template <typename T>
Point3D<T> Point3D<T>::operator-(const Point3D<T>& other) const {
    return Point3D<T>(x - other.x, y - other.y, z - other.z);
}

template <typename T>
Point3D<T>& Point3D<T>::operator-=(const Point3D<T>& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

template <typename T>
Point3D<T> Point3D<T>::operator*(T scalar) const {
    return Point3D<T>(x * scalar, y * scalar, z * scalar);
}

template <typename T>
Point3D<T>& Point3D<T>::operator*=(T scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

template <typename T>
Point3D<T> Point3D<T>::operator/(T scalar) const {
    if (scalar == T(0)) {
        throw std::runtime_error("Division by zero.");
    }
    return Point3D<T>(x / scalar, y / scalar, z / scalar);
}

template <typename T>
Point3D<T>& Point3D<T>::operator/=(T scalar) {
    if (scalar == T(0)) {
        throw std::runtime_error("Division by zero.");
    }
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
}

template <typename T>
bool Point3D<T>::operator==(const Point3D<T>& other) const {
    return x == other.x && y == other.y && z == other.z;
}

template <typename T>
bool Point3D<T>::operator!=(const Point3D<T>& other) const {
    return !(*this == other);
}

template <typename T>
const T* Point3D<T>::data() const {
    return &x;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Point3D<T>& point) {
    os << "(" << point.x << ", " << point.y << ", " << point.z << ")";
    return os;
}
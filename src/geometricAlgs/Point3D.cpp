#include "includes/Point3D.h"

template <typename T>
Point3D<T>::Point3D() : x(T(0)), y(T(0)), z(T(0)) {}

template <typename T>
Point3D<T>::Point3D(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

template <typename T>
Point3D<T>::Point3D(const Point3D<T>& other) : x(other.x), y(other.y), z(other.z) {}

template <typename T>
Point3D<T>& Point3D<T>::operator=(const Point3D<T>& other) {
    if (this != &other) {
        x = other.x;
        y = other.y;
        z = other.z;
    }
    return *this;
}

template <typename T>
Point3D<T>::~Point3D() {}

template <typename T>
T Point3D<T>::distanceTo(const Point3D<T>& other) const {
    T dx = x - other.x;
    T dy = y - other.y;
    T dz = z - other.z;
    return static_cast<T>(std::sqrt(dx * dx + dy * dy + dz * dz));
}

template <typename T>
Point3D<T> Point3D<T>::operator+(const Point3D<T>& other) const {
    return Point3D<T>(x + other.x, y + other.y, z + other.z);
}

template <typename T>
Point3D<T> Point3D<T>::operator-(const Point3D<T>& other) const {
    return Point3D<T>(x - other.x, y - other.y, z - other.z);
}

template <typename T>
Point3D<T> Point3D<T>::operator*(T scalar) const {
    return Point3D<T>(x * scalar, y * scalar, z * scalar);
}

template <typename T>
Point3D<T> Point3D<T>::operator/(T scalar) const {
    return Point3D<T>(x / scalar, y / scalar, z / scalar);
}

template <typename T>
const T* Point3D<T>::data() const {
    return &x;
}

template class Point3D<float>;
template class Point3D<double>;

#include "includes/Point3D.h"

// Default Constructor
template <typename T>
Point3D<T>::Point3D() : x(T(0)), y(T(0)), z(T(0)) {}

// Parameterized Constructor
template <typename T>
Point3D<T>::Point3D(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

// Copy Constructor
template <typename T>
Point3D<T>::Point3D(const Point3D<T>& other) : x(other.x), y(other.y), z(other.z) {}

// Assignment Operator
template <typename T>
Point3D<T>& Point3D<T>::operator=(const Point3D<T>& other) {
    if (this != &other) {
        this->x = other.x;
        this->y = other.y;
        this->z = other.z;
    }
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
    return this->x * other.x + this->y * other.y + this->z * other.z;
}

template <typename T>
Point3D<T> Point3D<T>::cross(const Point3D<T>& other) const {
    return Point3D<T>(
        this->y * other.z - this->z * other.y,
        this->z * other.x - this->x * other.z,
        this->x * other.y - this->y * other.x
    );
}

template <typename T>
T Point3D<T>::normSquared() const {
    return x * x + y * y + z * z;
}


template <typename T>
T Point3D<T>::norm() const {
    return std::sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
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
    return Point3D<T>(this->x + other.x, this->y + other.y, this->z + other.z);
}

template <typename T>
Point3D<T>& Point3D<T>::operator+=(const Point3D<T>& other) {
    this->x += other.x;
    this->y += other.y;
    this->z += other.z;
    return *this;
}

template <typename T>
Point3D<T> Point3D<T>::operator-(const Point3D<T>& other) const {
    return Point3D<T>(this->x - other.x, this->y - other.y, this->z - other.z);
}

template <typename T>
Point3D<T>& Point3D<T>::operator-=(const Point3D<T>& other) {
    this->x -= other.x;
    this->y -= other.y;
    this->z -= other.z;
    return *this;
}

template <typename T>
Point3D<T> Point3D<T>::operator*(T scalar) const {
    return Point3D<T>(this->x * scalar, this->y * scalar, this->z * scalar);
}

template <typename T>
Point3D<T>& Point3D<T>::operator*=(T scalar) {
    this->x *= scalar;
    this->y *= scalar;
    this->z *= scalar;
    return *this;
}

template <typename T>
Point3D<T> Point3D<T>::operator/(T scalar) const {
    if (scalar == T(0)) {
        throw std::runtime_error("Division by zero.");
    }
    return Point3D<T>(this->x / scalar, this->y / scalar, this->z / scalar);
}

template <typename T>
Point3D<T>& Point3D<T>::operator/=(T scalar) {
    if (scalar == T(0)) {
        throw std::runtime_error("Division by zero.");
    }
    this->x /= scalar;
    this->y /= scalar;
    this->z /= scalar;
    return *this;
}

template <typename T>
bool Point3D<T>::operator==(const Point3D<T>& other) const {
    return this->x == other.x && this->y == other.y && this->z == other.z;
}

template <typename T>
bool Point3D<T>::operator<(const Point3D<T>& other) const {
    if (x != other.x) return x < other.x;
    if (y != other.y) return y < other.y;
    return z < other.z;
}

template <typename T>
bool Point3D<T>::operator!=(const Point3D<T>& other) const {
    return !(*this == other);
}

template <typename T>
const T* Point3D<T>::data() const {
    return &this->x;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Point3D<T>& point) {
    os << "(" << point.x << ", " << point.y << ", " << point.z << ")";
    return os;
}

// Explicit Instantiation
template class Point3D<float>;
template class Point3D<double>;
#include "AABB.h"
// Axis-Aligned Bounding Box (AABB)
template <typename T>
AABB<T>::AABB()
    : min(std::numeric_limits<T>::max(), std::numeric_limits<T>::max(), std::numeric_limits<T>::max()),
      max(std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest()) {}

template <typename T>
AABB<T>::AABB(const Point3D<T>& minPoint, const Point3D<T>& maxPoint)
    : min(minPoint), max(maxPoint) {}

template <typename T>
void AABB<T>::expand(const Point3D<T>& point) {
    min.x = std::min(min.x, point.x);
    min.y = std::min(min.y, point.y);
    min.z = std::min(min.z, point.z);

    max.x = std::max(max.x, point.x);
    max.y = std::max(max.y, point.y);
    max.z = std::max(max.z, point.z);
}

template <typename T>
void AABB<T>::expand(const AABB<T>& box) {
    expand(box.min);
    expand(box.max);
}

template <typename T>
bool AABB<T>::intersects(const AABB<T>& box) const {
    return (min.x <= box.max.x && max.x >= box.min.x) &&
           (min.y <= box.max.y && max.y >= box.min.y) &&
           (min.z <= box.max.z && max.z >= box.min.z);
}

template <typename T>
bool AABB<T>::contains(const Point3D<T>& point) const {
    return (point.x >= min.x && point.x <= max.x) &&
           (point.y >= min.y && point.y <= max.y) &&
           (point.z >= min.z && point.z <= max.z);
}

template <typename T>
Point3D<T> AABB<T>::center() const {
    return Point3D<T>((min.x + max.x) / 2, (min.y + max.y) / 2, (min.z + max.z) / 2);
}

template <typename T>
Point3D<T> AABB<T>::size() const {
    return Point3D<T>(max.x - min.x, max.y - min.y, max.z - min.z);
}

template class AABB<float>;
template class AABB<double>;

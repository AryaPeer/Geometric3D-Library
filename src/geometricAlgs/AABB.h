#ifndef AABB_H
#define AABB_H

#include "Point3D.h"
#include <limits>
#include <algorithm>

// Axis-Aligned Bounding Box (AABB)
template <typename T>
class AABB {
public:
    Point3D<T> min;
    Point3D<T> max;

    AABB();
    AABB(const Point3D<T>& minPoint, const Point3D<T>& maxPoint);

    void expand(const Point3D<T>& point);
    void expand(const AABB<T>& box);

    bool intersects(const AABB<T>& box) const;
    bool contains(const Point3D<T>& point) const;

    Point3D<T> center() const;
    Point3D<T> size() const;
};

#endif // AABB_H

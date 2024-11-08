#ifndef OBB_H
#define OBB_H

#include "Point3D.h"
#include <array>

template <typename T>
class OBB {
public:
    Point3D<T> center;

    std::array<Point3D<T>, 3> axis;

    std::array<T, 3> extent;

    OBB();
    OBB(const Point3D<T>& centerPoint, const std::array<Point3D<T>, 3>& axes, const std::array<T, 3>& extents);

    bool contains(const Point3D<T>& point) const;

    void transform(const std::array<std::array<T, 3>, 3>& rotationMatrix, const Point3D<T>& translation);

    static OBB<T> computeFromPoints(const std::vector<Point3D<T>>& points);
    
    std::array<Point3D<T>, 8> getCorners() const;
};

#endif // OBB_H
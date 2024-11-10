#ifndef OBB_H
#define OBB_H

#include "Point3D.h"
#include <array>
#include <vector>
#include <Eigen/Dense>

template <typename T>
class OBB {
public:
    Point3D<T> center;
    std::array<Point3D<T>, 3> axes; 
    std::array<T, 3> extents;       

    OBB();
    OBB(const Point3D<T>& centerPoint, const std::array<Point3D<T>, 3>& axes, const std::array<T, 3>& extents);

    bool contains(const Point3D<T>& point) const;

    void transform(const Eigen::Transform<T, 3, Eigen::Affine>& transform);

    static OBB<T> computeFromPoints(const std::vector<Point3D<T>>& points);
    
    std::array<Point3D<T>, 8> getCorners() const;
};

#endif // OBB_H

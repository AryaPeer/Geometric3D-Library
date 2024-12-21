#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include <vector>
#include "Point3D.h"

template <typename T>
class ConvexHull {
public:
    static void compute(const std::vector<Point3D<T>>& points, std::vector<Point3D<T>>& hullVertices, std::vector<std::vector<int>>& hullFaces);
};

#include "ConvexHull.inl"

#endif // CONVEXHULL_H

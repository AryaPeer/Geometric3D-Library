#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include <vector>
#include <mutex>
#include "Point3D.h"

// Forward declaration of ConvexHull class
template <typename T>
class ConvexHull {
public:
    ConvexHull(const std::vector<Point3D<T>>& points);

    ~ConvexHull();

    void compute();

    const std::vector<Point3D<T>>& getVertices() const;

    const std::vector<std::vector<int>>& getFaces() const;

private:
    std::vector<Point3D<T>> inputPoints;
    std::vector<Point3D<T>> hullVertices;
    std::vector<std::vector<int>> hullFaces;

    mutable std::mutex mutex;

    struct Face {
        int indices[3];
        Point3D<T> normal;
    };

    void initializeHull();
    void addPointToHull(int pointIndex);
    T distanceToFace(const Face& face, const Point3D<T>& point) const;
    Point3D<T> computeNormal(const Point3D<T>& a, const Point3D<T>& b, const Point3D<T>& c) const;
};

#endif // CONVEXHULL_H

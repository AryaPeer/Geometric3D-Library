#ifndef OCTREE_H
#define OCTREE_H

#include <vector>
#include <array>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include "Point3D.h"

template <typename T>
class OctreeNode;

template <typename T>
class Octree {
public:
    Octree(const Point3D<T>& origin, T halfDimension, int maxPointsPerNode = 1);

    ~Octree();

    void insert(const Point3D<T>& point);

    bool remove(const Point3D<T>& point, T epsilon = std::numeric_limits<T>::epsilon());

    void queryRange(const Point3D<T>& min, const Point3D<T>& max, std::vector<Point3D<T>>& results) const;

    void clear();

private:
    std::unique_ptr<OctreeNode<T>> root;
    int maxPointsPerNode;

    Octree(const Octree&) = delete;
    Octree& operator=(const Octree&) = delete;
};

template <typename T>
class OctreeNode {
public:
    Point3D<T> origin;  
    T halfDimension;
    std::array<std::unique_ptr<OctreeNode<T>>, 8> children; 
    std::vector<Point3D<T>> points;

    mutable std::shared_mutex nodeMutex;

    OctreeNode(const Point3D<T>& origin, T halfDimension);

    ~OctreeNode();

    void insert(const Point3D<T>& point, int maxPointsPerNode);

    bool remove(const Point3D<T>& point, T epsilon, int maxPointsPerNode);

    void queryRange(const Point3D<T>& min, const Point3D<T>& max, std::vector<Point3D<T>>& results) const;

    void clear();

private:
    int getOctantContainingPoint(const Point3D<T>& point) const;

    void subdivide();

    bool intersects(const Point3D<T>& min, const Point3D<T>& max) const;

    bool arePointsEqual(const Point3D<T>& p1, const Point3D<T>& p2, T epsilon) const;
};

#endif // OCTREE_H

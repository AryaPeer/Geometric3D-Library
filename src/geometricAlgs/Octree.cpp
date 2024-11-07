#include "Octree.h"
#include <cmath>

template <typename T>
Octree<T>::Octree(const Point3D<T>& origin, T halfDimension, int maxPointsPerNode)
    : root(std::make_unique<OctreeNode<T>>(origin, halfDimension)), maxPointsPerNode(maxPointsPerNode) {}

template <typename T>
Octree<T>::~Octree() {
    clear();
}

template <typename T>
void Octree<T>::insert(const Point3D<T>& point) {
    root->insert(point, maxPointsPerNode);
}

template <typename T>
bool Octree<T>::remove(const Point3D<T>& point, T epsilon) {
    return root->remove(point, epsilon, maxPointsPerNode);
}

template <typename T>
void Octree<T>::queryRange(const Point3D<T>& min, const Point3D<T>& max, std::vector<Point3D<T>>& results) const {
    root->queryRange(min, max, results);
}

template <typename T>
void Octree<T>::clear() {
    root->clear();
}


template <typename T>
OctreeNode<T>::OctreeNode(const Point3D<T>& origin, T halfDimension)
    : origin(origin), halfDimension(halfDimension) {
    children.fill(nullptr);
}

template <typename T>
OctreeNode<T>::~OctreeNode() {
    clear();
}

template <typename T>
void OctreeNode<T>::insert(const Point3D<T>& point, int maxPointsPerNode) {
    std::unique_lock<std::shared_mutex> lock(nodeMutex);

    if (children[0] == nullptr) {
        points.push_back(point);

        if (points.size() > static_cast<size_t>(maxPointsPerNode)) {
            subdivide();

            for (const auto& p : points) {
                int octant = getOctantContainingPoint(p);
                children[octant]->insert(p, maxPointsPerNode);
            }
            points.clear();
        }
    } else {
        int octant = getOctantContainingPoint(point);
        lock.unlock();
        children[octant]->insert(point, maxPointsPerNode);
    }
}

template <typename T>
bool OctreeNode<T>::remove(const Point3D<T>& point, T epsilon, int maxPointsPerNode) {
    std::unique_lock<std::shared_mutex> lock(nodeMutex);

    if (!intersects(point, point)) {
        return false;
    }

    if (children[0] == nullptr) {
        for (auto it = points.begin(); it != points.end(); ++it) {
            if (arePointsEqual(*it, point, epsilon)) {
                points.erase(it);
                return true;
            }
        }
        return false;
    } else {
        int octant = getOctantContainingPoint(point);
        lock.unlock();
        bool removed = children[octant]->remove(point, epsilon, maxPointsPerNode);

        lock.lock();
        bool allChildrenEmpty = true;
        for (int i = 0; i < 8; ++i) {
            std::shared_lock<std::shared_mutex> childLock(children[i]->nodeMutex);
            if (children[i]->children[0] != nullptr || !children[i]->points.empty()) {
                allChildrenEmpty = false;
                break;
            }
        }

        if (allChildrenEmpty) {
            for (int i = 0; i < 8; ++i) {
                children[i].reset();
            }
        }

        return removed;
    }
}

template <typename T>
void OctreeNode<T>::queryRange(const Point3D<T>& min, const Point3D<T>& max, std::vector<Point3D<T>>& results) const {
    std::shared_lock<std::shared_mutex> lock(nodeMutex);

    if (!intersects(min, max))
        return;

    if (children[0] == nullptr) {
        for (const auto& p : points) {
            if (p.x >= min.x && p.x <= max.x &&
                p.y >= min.y && p.y <= max.y &&
                p.z >= min.z && p.z <= max.z) {
                results.push_back(p);
            }
        }
    } else {
        lock.unlock(); 
        for (int i = 0; i < 8; ++i) {
            children[i]->queryRange(min, max, results);
        }
    }
}

template <typename T>
void OctreeNode<T>::clear() {
    std::unique_lock<std::shared_mutex> lock(nodeMutex);
    points.clear();

    for (int i = 0; i < 8; ++i) {
        if (children[i]) {
            lock.unlock();
            children[i]->clear();
            lock.lock();
            children[i].reset();
        }
    }
}

template <typename T>
int OctreeNode<T>::getOctantContainingPoint(const Point3D<T>& point) const {
    int octant = 0;
    if (point.x >= origin.x) octant |= 4;
    if (point.y >= origin.y) octant |= 2;
    if (point.z >= origin.z) octant |= 1;
    return octant;
}

template <typename T>
void OctreeNode<T>::subdivide() {
    T childHalf = halfDimension / static_cast<T>(2);
    for (int i = 0; i < 8; ++i) {
        Point3D<T> childOrigin = origin;
        childOrigin.x += childHalf * ((i & 4) ? static_cast<T>(0.5) : static_cast<T>(-0.5));
        childOrigin.y += childHalf * ((i & 2) ? static_cast<T>(0.5) : static_cast<T>(-0.5));
        childOrigin.z += childHalf * ((i & 1) ? static_cast<T>(0.5) : static_cast<T>(-0.5));
        children[i] = std::make_unique<OctreeNode<T>>(childOrigin, childHalf);
    }
}

template <typename T>
bool OctreeNode<T>::intersects(const Point3D<T>& min, const Point3D<T>& max) const {
    return !(origin.x + halfDimension < min.x || origin.x - halfDimension > max.x ||
             origin.y + halfDimension < min.y || origin.y - halfDimension > max.y ||
             origin.z + halfDimension < min.z || origin.z - halfDimension > max.z);
}

template <typename T>
bool OctreeNode<T>::arePointsEqual(const Point3D<T>& p1, const Point3D<T>& p2, T epsilon) const {
    return std::abs(p1.x - p2.x) <= epsilon &&
           std::abs(p1.y - p2.y) <= epsilon &&
           std::abs(p1.z - p2.z) <= epsilon;
}

template class Octree<float>;
template class Octree<double>;

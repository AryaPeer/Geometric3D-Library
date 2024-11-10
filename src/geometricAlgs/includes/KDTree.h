#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include <mutex>
#include <shared_mutex>
#include <algorithm>
#include "Point3D.h"

template <typename T>
class KDTreeNode;

template <typename T>
class KDTree {
public:
    KDTree();
    ~KDTree();

    void build(const std::vector<Point3D<T>>& points);

    void insert(const Point3D<T>& point);
    bool remove(const Point3D<T>& point, T epsilon = std::numeric_limits<T>::epsilon());

    bool nearestNeighbor(const Point3D<T>& queryPoint, Point3D<T>& nearest, T& minDistSquared) const;
    void kNearestNeighbors(const Point3D<T>& queryPoint, int k, std::vector<Point3D<T>>& neighbors) const;
    void radiusSearch(const Point3D<T>& queryPoint, T radius, std::vector<Point3D<T>>& neighbors) const;

    void rangeQuery(const Point3D<T>& min, const Point3D<T>& max, std::vector<Point3D<T>>& results) const;
    void clear();

private:
    KDTreeNode<T>* root;

    mutable std::shared_mutex treeMutex;

    KDTreeNode<T>* buildRec(std::vector<Point3D<T>>& points, int depth);
    KDTreeNode<T>* insertRec(KDTreeNode<T>* node, const Point3D<T>& point, int depth);
    KDTreeNode<T>* removeRec(KDTreeNode<T>* node, const Point3D<T>& point, int depth, bool& deleted);
    KDTreeNode<T>* findMin(KDTreeNode<T>* node, int axis, int depth);
    void nearestNeighborRec(KDTreeNode<T>* node, const Point3D<T>& queryPoint, int depth, Point3D<T>& nearest, T& minDistSquared) const;
    void kNearestNeighborsRec(KDTreeNode<T>* node, const Point3D<T>& queryPoint, int depth, int k, std::vector<std::pair<T, Point3D<T>>>& heap) const;
    void radiusSearchRec(KDTreeNode<T>* node, const Point3D<T>& queryPoint, T radiusSquared, int depth, std::vector<Point3D<T>>& neighbors) const;
    void rangeQueryRec(KDTreeNode<T>* node, const Point3D<T>& min, const Point3D<T>& max, int depth, std::vector<Point3D<T>>& results) const;
    void clearRec(KDTreeNode<T>* node);

    KDTree(const KDTree&) = delete;
    KDTree& operator=(const KDTree&) = delete;
};

template <typename T>
class KDTreeNode {
public:
    Point3D<T> point;
    KDTreeNode<T>* left;
    KDTreeNode<T>* right;

    mutable std::shared_mutex nodeMutex;

    KDTreeNode(const Point3D<T>& pt)
        : point(pt), left(nullptr), right(nullptr) {}
};

#endif // KDTREE_H
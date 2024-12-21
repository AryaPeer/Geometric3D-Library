#include "includes/KDTree.h"
#include <limits>
#include <cmath>
#include <queue>

// Constructor
template <typename T>
KDTree<T>::KDTree() : root(nullptr) {}

// Destructor
template <typename T>
KDTree<T>::~KDTree() {
    clear();
}

template <typename T>
void KDTree<T>::build(const std::vector<Point3D<T>>& points) {
    std::unique_lock<std::shared_mutex> lock(treeMutex);
    std::vector<Point3D<T>> pts = points;
    root = buildRec(pts, 0);
}

template <typename T>
KDTreeNode<T>* KDTree<T>::buildRec(std::vector<Point3D<T>>& points, int depth) {
    if (points.empty()) {
        return nullptr;
    }

    int axis = depth % 3;
    auto comparator = [axis](const Point3D<T>& a, const Point3D<T>& b) {
        if (axis == 0)
            return a.x < b.x;
        else if (axis == 1)
            return a.y < b.y;
        else
            return a.z < b.z;
    };

    size_t medianIndex = points.size() / 2;
    std::nth_element(points.begin(), points.begin() + medianIndex, points.end(), comparator);

    Point3D<T> medianPoint = points[medianIndex];
    std::vector<Point3D<T>> leftPoints(points.begin(), points.begin() + medianIndex);
    std::vector<Point3D<T>> rightPoints(points.begin() + medianIndex + 1, points.end());

    KDTreeNode<T>* node = new KDTreeNode<T>(medianPoint);
    node->left = buildRec(leftPoints, depth + 1);
    node->right = buildRec(rightPoints, depth + 1);

    return node;
}

template <typename T>
void KDTree<T>::insert(const Point3D<T>& point) {
    std::unique_lock<std::shared_mutex> lock(treeMutex);
    root = insertRec(root, point, 0);
}

template <typename T>
KDTreeNode<T>* KDTree<T>::insertRec(KDTreeNode<T>* node, const Point3D<T>& point, int depth) {
    if (node == nullptr) {
        return new KDTreeNode<T>(point);
    }

    int axis = depth % 3;

    {
        std::unique_lock<std::shared_mutex> nodeLock(node->nodeMutex);

        if ((axis == 0 && point.x < node->point.x) ||
            (axis == 1 && point.y < node->point.y) ||
            (axis == 2 && point.z < node->point.z)) {
            node->left = insertRec(node->left, point, depth + 1);
        } else {
            node->right = insertRec(node->right, point, depth + 1);
        }
    }

    return node;
}

template <typename T>
bool KDTree<T>::remove(const Point3D<T>& point, T epsilon) {
    std::unique_lock<std::shared_mutex> lock(treeMutex);
    bool deleted = false;
    root = removeRec(root, point, 0, deleted, epsilon);
    return deleted;
}

template <typename T>
KDTreeNode<T>* KDTree<T>::removeRec(KDTreeNode<T>* node, const Point3D<T>& point, int depth, bool& deleted, T epsilon) {
    if (node == nullptr) {
        return nullptr;
    }

    int axis = depth % 3;

    {
        std::unique_lock<std::shared_mutex> nodeLock(node->nodeMutex);

        if (node->point.distanceTo(point) <= epsilon) {
            deleted = true;

            if (node->right != nullptr) {
                KDTreeNode<T>* minNode = findMin(node->right, axis, depth + 1);
                {
                    std::unique_lock<std::shared_mutex> minNodeLock(minNode->nodeMutex);
                    node->point = minNode->point;
                }
                node->right = removeRec(node->right, minNode->point, depth + 1, deleted, epsilon);
            } else if (node->left != nullptr) {
                KDTreeNode<T>* minNode = findMin(node->left, axis, depth + 1);
                {
                    std::unique_lock<std::shared_mutex> minNodeLock(minNode->nodeMutex);
                    node->point = minNode->point;
                }
                node->right = node->left;
                node->left = nullptr;
                node->right = removeRec(node->right, minNode->point, depth + 1, deleted, epsilon);
            } else {
                delete node;
                return nullptr;
            }
            return node;
        }

        if ((axis == 0 && point.x < node->point.x) ||
            (axis == 1 && point.y < node->point.y) ||
            (axis == 2 && point.z < node->point.z)) {
            node->left = removeRec(node->left, point, depth + 1, deleted, epsilon);
        } else {
            node->right = removeRec(node->right, point, depth + 1, deleted, epsilon);
        }
    }

    return node;
}

template <typename T>
KDTreeNode<T>* KDTree<T>::findMin(KDTreeNode<T>* node, int axis, int depth) {
    if (node == nullptr) {
        return nullptr;
    }

    int currentAxis = depth % 3;

    std::shared_lock<std::shared_mutex> nodeLock(node->nodeMutex);

    if (currentAxis == axis) {
        if (node->left == nullptr) {
            return node;
        }
        return findMin(node->left, axis, depth + 1);
    }

    KDTreeNode<T>* leftMin = findMin(node->left, axis, depth + 1);
    KDTreeNode<T>* rightMin = findMin(node->right, axis, depth + 1);
    KDTreeNode<T>* minNode = node;

    auto compare = [axis](const Point3D<T>& a, const Point3D<T>& b) {
        if (axis == 0)
            return a.x < b.x;
        else if (axis == 1)
            return a.y < b.y;
        else
            return a.z < b.z;
    };

    if (leftMin != nullptr && compare(leftMin->point, minNode->point)) {
        minNode = leftMin;
    }

    if (rightMin != nullptr && compare(rightMin->point, minNode->point)) {
        minNode = rightMin;
    }

    return minNode;
}

template <typename T>
bool KDTree<T>::nearestNeighbor(const Point3D<T>& queryPoint, Point3D<T>& nearest, T& minDistSquared) const {
    std::shared_lock<std::shared_mutex> lock(treeMutex);
    if (root == nullptr) {
        return false;
    }

    minDistSquared = std::numeric_limits<T>::max();
    nearestNeighborRec(root, queryPoint, 0, nearest, minDistSquared);
    return true;
}

template <typename T>
void KDTree<T>::nearestNeighborRec(KDTreeNode<T>* node, const Point3D<T>& queryPoint, int depth, Point3D<T>& nearest, T& minDistSquared) const {
    if (node == nullptr) {
        return;
    }

    std::shared_lock<std::shared_mutex> nodeLock(node->nodeMutex);

    T distSquared = (node->point - queryPoint).normSquared();

    if (distSquared < minDistSquared) {
        minDistSquared = distSquared;
        nearest = node->point;
    }

    int axis = depth % 3;
    KDTreeNode<T>* nearNode = nullptr;
    KDTreeNode<T>* farNode = nullptr;

    if ((axis == 0 && queryPoint.x < node->point.x) ||
        (axis == 1 && queryPoint.y < node->point.y) ||
        (axis == 2 && queryPoint.z < node->point.z)) {
        nearNode = node->left;
        farNode = node->right;
    } else {
        nearNode = node->right;
        farNode = node->left;
    }

    nearestNeighborRec(nearNode, queryPoint, depth + 1, nearest, minDistSquared);

    T diff = 0;
    if (axis == 0) diff = queryPoint.x - node->point.x;
    else if (axis == 1) diff = queryPoint.y - node->point.y;
    else diff = queryPoint.z - node->point.z;

    if (diff * diff < minDistSquared) {
        nearestNeighborRec(farNode, queryPoint, depth + 1, nearest, minDistSquared);
    }
}

template <typename T>
void KDTree<T>::kNearestNeighbors(const Point3D<T>& queryPoint, int k, std::vector<Point3D<T>>& neighbors) const {
    std::shared_lock<std::shared_mutex> lock(treeMutex);
    if (root == nullptr || k <= 0) {
        return;
    }

    std::priority_queue<std::pair<T, Point3D<T>>, 
                       std::vector<std::pair<T, Point3D<T>>>,
                       PairComparator> heap;

    kNearestNeighborsRec(root, queryPoint, 0, k, heap);

    neighbors.clear();
    while (!heap.empty()) {
        neighbors.push_back(heap.top().second);
        heap.pop();
    }
    std::reverse(neighbors.begin(), neighbors.end());
}

template <typename T>
void KDTree<T>::kNearestNeighborsRec(KDTreeNode<T>* node, const Point3D<T>& queryPoint, 
    int depth, int k, std::priority_queue<std::pair<T, Point3D<T>>, 
                                        std::vector<std::pair<T, Point3D<T>>>,
                                        PairComparator>& heap) const {
    
    if (node == nullptr) return;

    const int axis = depth % 3;
    T distance = queryPoint.distanceTo(node->point);

    if (heap.size() < static_cast<size_t>(k) || distance < heap.top().first) {
        if (heap.size() == static_cast<size_t>(k)) heap.pop();
        heap.push(std::make_pair(distance, node->point));
    }

    T delta;
    switch (axis) {
        case 0: delta = queryPoint.x - node->point.x; break;
        case 1: delta = queryPoint.y - node->point.y; break;
        default: delta = queryPoint.z - node->point.z; break;
    }

    KDTreeNode<T>* firstSearch = (delta < 0) ? node->left : node->right;
    KDTreeNode<T>* secondSearch = (delta < 0) ? node->right : node->left;

    kNearestNeighborsRec(firstSearch, queryPoint, depth + 1, k, heap);

    if (heap.size() < static_cast<size_t>(k) || std::abs(delta) < heap.top().first) {
        kNearestNeighborsRec(secondSearch, queryPoint, depth + 1, k, heap);
    }
}


template <typename T>
void KDTree<T>::radiusSearch(const Point3D<T>& queryPoint, T radius, std::vector<Point3D<T>>& neighbors) const {
    std::shared_lock<std::shared_mutex> lock(treeMutex);
    if (root == nullptr || radius <= 0) {
        return;
    }

    T radiusSquared = radius * radius;
    radiusSearchRec(root, queryPoint, radiusSquared, 0, neighbors);
}

template <typename T>
void KDTree<T>::radiusSearchRec(KDTreeNode<T>* node, const Point3D<T>& queryPoint, T radiusSquared, int depth, std::vector<Point3D<T>>& neighbors) const {
    if (node == nullptr) {
        return;
    }

    std::shared_lock<std::shared_mutex> nodeLock(node->nodeMutex);

    T distSquared = (node->point - queryPoint).normSquared();

    if (distSquared <= radiusSquared) {
        neighbors.push_back(node->point);
    }

    int axis = depth % 3;
    KDTreeNode<T>* nearNode = nullptr;
    KDTreeNode<T>* farNode = nullptr;

    if ((axis == 0 && queryPoint.x < node->point.x) ||
        (axis == 1 && queryPoint.y < node->point.y) ||
        (axis == 2 && queryPoint.z < node->point.z)) {
        nearNode = node->left;
        farNode = node->right;
    } else {
        nearNode = node->right;
        farNode = node->left;
    }

    radiusSearchRec(nearNode, queryPoint, radiusSquared, depth + 1, neighbors);

    T diff = 0;
    if (axis == 0) diff = queryPoint.x - node->point.x;
    else if (axis == 1) diff = queryPoint.y - node->point.y;
    else diff = queryPoint.z - node->point.z;

    if (diff * diff <= radiusSquared) {
        radiusSearchRec(farNode, queryPoint, radiusSquared, depth + 1, neighbors);
    }
}

template <typename T>
void KDTree<T>::rangeQuery(const Point3D<T>& min, const Point3D<T>& max, std::vector<Point3D<T>>& results) const {
    std::shared_lock<std::shared_mutex> lock(treeMutex);
    rangeQueryRec(root, min, max, 0, results);
}

template <typename T>
void KDTree<T>::rangeQueryRec(KDTreeNode<T>* node, const Point3D<T>& min, const Point3D<T>& max, int depth, std::vector<Point3D<T>>& results) const {
    if (node == nullptr) {
        return;
    }

    std::shared_lock<std::shared_mutex> nodeLock(node->nodeMutex);

    if (node->point.x >= min.x && node->point.x <= max.x &&
        node->point.y >= min.y && node->point.y <= max.y &&
        node->point.z >= min.z && node->point.z <= max.z) {
        results.push_back(node->point);
    }

    int axis = depth % 3;

    if ((axis == 0 && min.x <= node->point.x) ||
        (axis == 1 && min.y <= node->point.y) ||
        (axis == 2 && min.z <= node->point.z)) {
        rangeQueryRec(node->left, min, max, depth + 1, results);
    }

    if ((axis == 0 && max.x >= node->point.x) ||
        (axis == 1 && max.y >= node->point.y) ||
        (axis == 2 && max.z >= node->point.z)) {
        rangeQueryRec(node->right, min, max, depth + 1, results);
    }
}

template <typename T>
void KDTree<T>::clear() {
    std::unique_lock<std::shared_mutex> lock(treeMutex);
    clearRec(root);
    root = nullptr;
}

template <typename T>
void KDTree<T>::clearRec(KDTreeNode<T>* node) {
    if (node != nullptr) {
        clearRec(node->left);
        clearRec(node->right);
        delete node;
    }
}

template class KDTree<float>;
template class KDTree<double>;

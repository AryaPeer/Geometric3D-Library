#include "KDTree.h"
#include <limits>
#include <cmath>

// Constructor
template <typename T>
KDTree<T>::KDTree() : root(nullptr) {}

// Destructor
template <typename T>
KDTree<T>::~KDTree() {
    clear();
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
bool KDTree<T>::remove(const Point3D<T>& point) {
    std::unique_lock<std::shared_mutex> lock(treeMutex);
    bool deleted = false;
    root = removeRec(root, point, 0, deleted);
    return deleted;
}

template <typename T>
KDTreeNode<T>* KDTree<T>::removeRec(KDTreeNode<T>* node, const Point3D<T>& point, int depth, bool& deleted) {
    if (node == nullptr) {
        return nullptr;
    }

    int axis = depth % 3;

    {
        std::unique_lock<std::shared_mutex> nodeLock(node->nodeMutex);

        if (node->point == point) {
            deleted = true;

            if (node->right != nullptr) {
                KDTreeNode<T>* minNode = findMin(node->right, axis, depth + 1);

                {
                    std::unique_lock<std::shared_mutex> minNodeLock(minNode->nodeMutex);
                    node->point = minNode->point;
                }

                node->right = removeRec(node->right, minNode->point, depth + 1, deleted);
            } else if (node->left != nullptr) {
                KDTreeNode<T>* minNode = findMin(node->left, axis, depth + 1);

                {
                    std::unique_lock<std::shared_mutex> minNodeLock(minNode->nodeMutex);
                    node->point = minNode->point;
                }

                node->right = node->left;
                node->left = nullptr;

                node->right = removeRec(node->right, minNode->point, depth + 1, deleted);
            } else {
                delete node;
                return nullptr;
            }
            return node;
        }

        if ((axis == 0 && point.x < node->point.x) ||
            (axis == 1 && point.y < node->point.y) ||
            (axis == 2 && point.z < node->point.z)) {
            node->left = removeRec(node->left, point, depth + 1, deleted);
        } else {
            node->right = removeRec(node->right, point, depth + 1, deleted);
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

    T distSquared = (node->point.x - queryPoint.x) * (node->point.x - queryPoint.x) +
                    (node->point.y - queryPoint.y) * (node->point.y - queryPoint.y) +
                    (node->point.z - queryPoint.z) * (node->point.z - queryPoint.z);

    if (distSquared < minDistSquared) {
        minDistSquared = distSquared;
        nearest = node->point;
    }

    int axis = depth % 3;
    KDTreeNode<T>* nextNode = nullptr;
    KDTreeNode<T>* otherNode = nullptr;

    if ((axis == 0 && queryPoint.x < node->point.x) ||
        (axis == 1 && queryPoint.y < node->point.y) ||
        (axis == 2 && queryPoint.z < node->point.z)) {
        nextNode = node->left;
        otherNode = node->right;
    } else {
        nextNode = node->right;
        otherNode = node->left;
    }

    nearestNeighborRec(nextNode, queryPoint, depth + 1, nearest, minDistSquared);

    T diff = 0;
    if (axis == 0) diff = queryPoint.x - node->point.x;
    else if (axis == 1) diff = queryPoint.y - node->point.y;
    else diff = queryPoint.z - node->point.z;

    if (diff * diff < minDistSquared) {
        nearestNeighborRec(otherNode, queryPoint, depth + 1, nearest, minDistSquared);
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

#ifndef BVH_H
#define BVH_H

#include <vector>
#include <memory>
#include <mutex>
#include <algorithm>
#include "AABB.h"

template <typename T>
class BVHNode {
public:
    AABB<T> boundingBox;
    std::unique_ptr<BVHNode<T>> left;
    std::unique_ptr<BVHNode<T>> right;
    std::vector<T> objects; // Only for leaf nodes

    // Constructor
    BVHNode();

    // Build the BVH node
    void build(std::vector<T>& objects, int maxObjectsPerLeaf = 1);

    // Mutex for thread safety during construction
    std::mutex mutex;
};

template <typename T>
class BVH {
public:
    BVH(std::vector<T>& objects, int maxObjectsPerLeaf = 1);

    ~BVH();

    const BVHNode<T>* getRoot() const;

private:
    std::unique_ptr<BVHNode<T>> root;
};

#endif
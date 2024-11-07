#include "BVH.h"
#include <thread>
#include <future>

// BVHNode

template <typename T>
BVHNode<T>::BVHNode() {}

template <typename T>
void BVHNode<T>::build(std::vector<T>& objs, int maxObjectsPerLeaf) {
    boundingBox = AABB<typename T::value_type>();

    for (const auto& obj : objs) {
        boundingBox.expand(obj.getBoundingBox());
    }

    if (objs.size() <= maxObjectsPerLeaf) {
        objects = objs;
        return;
    }

    Point3D<typename T::value_type> extent = boundingBox.size();
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent.y && extent.z > extent.x) axis = 2;

    std::sort(objs.begin(), objs.end(), [axis](const T& a, const T& b) {
        return a.getCenter()[axis] < b.getCenter()[axis];
    });

    size_t mid = objs.size() / 2;
    std::vector<T> leftObjects(objs.begin(), objs.begin() + mid);
    std::vector<T> rightObjects(objs.begin() + mid, objs.end());

    auto buildChild = [](std::unique_ptr<BVHNode<T>>& childNode, std::vector<T>& childObjects, int maxObjectsPerLeaf) {
        childNode = std::make_unique<BVHNode<T>>();
        childNode->build(childObjects, maxObjectsPerLeaf);
    };

    std::future<void> leftFuture = std::async(std::launch::async, buildChild, std::ref(left), std::ref(leftObjects), maxObjectsPerLeaf);
    std::future<void> rightFuture = std::async(std::launch::async, buildChild, std::ref(right), std::ref(rightObjects), maxObjectsPerLeaf);

    leftFuture.get();
    rightFuture.get();
}

template <typename T>
BVH<T>::BVH(std::vector<T>& objects, int maxObjectsPerLeaf) {
    root = std::make_unique<BVHNode<T>>();
    root->build(objects, maxObjectsPerLeaf);
}

template <typename T>
BVH<T>::~BVH() {}

template <typename T>
const BVHNode<T>* BVH<T>::getRoot() const {
    return root.get();
}


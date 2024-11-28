#include "BVH.h"
#include <algorithm>
#include <limits>
#include <thread>
#include <future>

template <typename T>
BVHNode<T>::BVHNode() {}

template <typename T>
void BVHNode<T>::build(std::vector<T>& objs, int maxObjectsPerLeaf) {
    boundingBox.reset();
    for (const auto& obj : objs) {
        boundingBox.expand(obj.getBoundingBox());
    }
    if (objs.size() <= maxObjectsPerLeaf) {
        objects = std::move(objs);
        return;
    }
    AABB<typename T::value_type> centroidBounds;
    centroidBounds.reset();
    for (const auto& obj : objs) {
        centroidBounds.expand(obj.getBoundingBox().center());
    }
    Point3D<typename T::value_type> extent = centroidBounds.size();
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;
    if (extent[axis] == 0) {
        objects = std::move(objs);
        return;
    }
    typename T::value_type midPoint = centroidBounds.min[axis] + extent[axis] / 2;
    auto midIter = std::partition(objs.begin(), objs.end(), [axis, midPoint](const T& obj) {
        return obj.getBoundingBox().center()[axis] < midPoint;
    });
    if (midIter == objs.begin() || midIter == objs.end()) {
        midIter = objs.begin() + objs.size() / 2;
        std::nth_element(objs.begin(), midIter, objs.end(), [axis](const T& a, const T& b) {
            return a.getBoundingBox().center()[axis] < b.getBoundingBox().center()[axis];
        });
    }
    std::vector<T> leftObjects(objs.begin(), midIter);
    std::vector<T> rightObjects(midIter, objs.end());
    left = std::make_unique<BVHNode<T>>();
    right = std::make_unique<BVHNode<T>>();
    auto leftFuture = std::async(std::launch::async, [&]() {
        left->build(leftObjects, maxObjectsPerLeaf);
    });
    right->build(rightObjects, maxObjectsPerLeaf);
    leftFuture.get();
}

template <typename T>
void BVHNode<T>::refit() {
    if (isLeaf()) {
        boundingBox.reset();
        for (const auto& obj : objects) {
            boundingBox.expand(obj.getBoundingBox());
        }
    } else {
        left->refit();
        right->refit();
        boundingBox = left->boundingBox;
        boundingBox.expand(right->boundingBox);
    }
}

template <typename T>
bool BVHNode<T>::isLeaf() const {
    return !left && !right;
}

template <typename T>
template <typename Ray>
bool BVHNode<T>::intersect(const Ray& ray, float& tMin, T& hitObject) const {
    if (!boundingBox.intersect(ray, tMin))
        return false;
    bool hit = false;
    if (isLeaf()) {
        for (const auto& obj : objects) {
            float t;
            if (obj.intersect(ray, t) && t < tMin) {
                tMin = t;
                hitObject = obj;
                hit = true;
            }
        }
    } else {
        T tempHitObjectLeft, tempHitObjectRight;
        float tLeft = tMin, tRight = tMin;
        bool hitLeft = left && left->intersect(ray, tLeft, tempHitObjectLeft);
        bool hitRight = right && right->intersect(ray, tRight, tempHitObjectRight);
        if (hitLeft && (!hitRight || tLeft < tRight)) {
            tMin = tLeft;
            hitObject = tempHitObjectLeft;
            hit = true;
        } else if (hitRight) {
            tMin = tRight;
            hitObject = tempHitObjectRight;
            hit = true;
        }
    }
    return hit;
}

template <typename T>
void BVHNode<T>::insert(const T& object) {
    boundingBox.expand(object.getBoundingBox());

    if (isLeaf()) {
        objects.push_back(object);
        if (objects.size() > maxObjectsPerLeaf) {
            std::vector<T> objs = std::move(objects);
            build(objs, maxObjectsPerLeaf);
        }
    } else {
        bool inserted = false;
        if (left->boundingBox.contains(object.getBoundingBox())) {
            left->insert(object);
            inserted = true;
        }
        if (right->boundingBox.contains(object.getBoundingBox())) {
            right->insert(object);
            inserted = true;
        }
        if (!inserted) {
            objects.push_back(object);
        }
    }
}

template <typename T>
void BVHNode<T>::remove(const T& object) {
    if (isLeaf()) {
        auto it = std::find(objects.begin(), objects.end(), object);
        if (it != objects.end()) {
            objects.erase(it);
            refit();
        }
    } else {
        if (left->boundingBox.intersects(object.getBoundingBox())) {
            left->remove(object);
        }
        if (right->boundingBox.intersects(object.getBoundingBox())) {
            right->remove(object);
        }
        refit();
    }
}

template <typename T>
BVH<T>::BVH(int maxObjectsPerLeaf) : maxObjectsPerLeaf(maxObjectsPerLeaf) {}

template <typename T>
void BVH<T>::build(const std::vector<T>& objects) {
    std::vector<T> objs = objects;
    root = std::make_unique<BVHNode<T>>();
    root->build(objs, maxObjectsPerLeaf);
}

template <typename T>
void BVH<T>::refit() {
    if (root)
        root->refit();
}

template <typename T>
void BVH<T>::insert(const T& object) {
    if (!root) {
        root = std::make_unique<BVHNode<T>>();
        root->objects.push_back(object);
        root->boundingBox = object.getBoundingBox();
    } else {
        root->insert(object);
    }
}

template <typename T>
void BVH<T>::remove(const T& object) {
    if (root) {
        root->remove(object);
    }
}

template <typename T>
template <typename Ray>
bool BVH<T>::intersect(const Ray& ray, T& hitObject) const {
    float tMin = std::numeric_limits<float>::max();
    if (!root)
        return false;
    return root->intersect(ray, tMin, hitObject);
}

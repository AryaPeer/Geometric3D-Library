#ifndef BVH_H
#define BVH_H

#include <vector>
#include <memory>
#include "AABB.h"

template <typename T>
class BVHNode {
public:
    AABB<typename T::value_type> boundingBox;
    std::unique_ptr<BVHNode<T>> left;
    std::unique_ptr<BVHNode<T>> right;
    std::vector<T> objects;

    BVHNode();

    void build(std::vector<T>& objects, int maxObjectsPerLeaf = 1);
    void refit();
    bool isLeaf() const;

    template <typename Ray>
    bool intersect(const Ray& ray, float& tMin, T& hitObject) const;

    void insert(const T& object);
    void remove(const T& object);
};

template <typename T>
class BVH {
public:
    BVH(int maxObjectsPerLeaf = 1);

    void build(const std::vector<T>& objects);
    void refit();
    void insert(const T& object);
    void remove(const T& object);

    template <typename Ray>
    bool intersect(const Ray& ray, T& hitObject) const;

private:
    std::unique_ptr<BVHNode<T>> root;
    int maxObjectsPerLeaf;
};

#endif // BVH_H

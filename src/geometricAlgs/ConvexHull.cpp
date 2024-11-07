#include "ConvexHull.h"
#include <thread>
#include <future>
#include <set>
#include <algorithm>
#include <cmath>

template <typename T>
ConvexHull<T>::ConvexHull(const std::vector<Point3D<T>>& points) : inputPoints(points) {}


template <typename T>
ConvexHull<T>::~ConvexHull() {}

template <typename T>
void ConvexHull<T>::compute() {
    if (inputPoints.size() < 4) {
        std::lock_guard<std::mutex> lock(mutex);
        hullVertices = inputPoints;
        return;
    }

    initializeHull();

    std::vector<std::future<void>> futures;
    size_t numThreads = std::thread::hardware_concurrency();
    size_t pointsPerThread = inputPoints.size() / numThreads;

    for (size_t i = 0; i < numThreads; ++i) {
        futures.emplace_back(std::async(std::launch::async, [this, i, numThreads, pointsPerThread]() {
            size_t startIdx = i * pointsPerThread;
            size_t endIdx = (i == numThreads - 1) ? inputPoints.size() : startIdx + pointsPerThread;

            for (size_t j = startIdx; j < endIdx; ++j) {
                addPointToHull(static_cast<int>(j));
            }
        }));
    }

    // Wait for all threads to complete
    for (auto& f : futures) {
        f.get();
    }


}

template <typename T>
void ConvexHull<T>::initializeHull() {
    int i0 = 0, i1 = 1, i2 = 2, i3 = 3;

    Point3D<T> normal = computeNormal(inputPoints[i0], inputPoints[i1], inputPoints[i2]);

    T dist = distanceToFace({{i0, i1, i2}, normal}, inputPoints[i3]);
    if (std::abs(dist) < std::numeric_limits<T>::epsilon()) {
        // Find another point that's not coplanar
        for (size_t i = 4; i < inputPoints.size(); ++i) {
            dist = distanceToFace({{i0, i1, i2}, normal}, inputPoints[i]);
            if (std::abs(dist) > std::numeric_limits<T>::epsilon()) {
                i3 = static_cast<int>(i);
                break;
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(mutex);
        hullVertices = {inputPoints[i0], inputPoints[i1], inputPoints[i2], inputPoints[i3]};
        hullFaces = {
            {0, 1, 2},
            {0, 1, 3},
            {1, 2, 3},
            {2, 0, 3}
        };
    }
}

template <typename T>
void ConvexHull<T>::addPointToHull(int pointIndex) {
    const Point3D<T>& point = inputPoints[pointIndex];

    std::vector<int> visibleFaces;
    std::map<std::pair<int, int>, int> edgeCounts;

    {
        std::lock_guard<std::mutex> lock(mutex);
        for (size_t faceIdx = 0; faceIdx < hullFaces.size(); ++faceIdx) {
            const auto& faceIndices = hullFaces[faceIdx];

            Face face;
            face.indices[0] = faceIndices[0];
            face.indices[1] = faceIndices[1];
            face.indices[2] = faceIndices[2];
            face.normal = computeNormal(
                hullVertices[face.indices[0]],
                hullVertices[face.indices[1]],
                hullVertices[face.indices[2]]
            );

            T dist = distanceToFace(face, point);
            if (dist > std::numeric_limits<T>::epsilon()) {
                visibleFaces.push_back(static_cast<int>(faceIdx));

                for (int i = 0; i < 3; ++i) {
                    int idx1 = face.indices[i];
                    int idx2 = face.indices[(i + 1) % 3];
                    int minIdx = std::min(idx1, idx2);
                    int maxIdx = std::max(idx1, idx2);
                    std::pair<int, int> edge(minIdx, maxIdx);
                    edgeCounts[edge]++;
                }
            }
        }

        if (!visibleFaces.empty()) {
            std::vector<std::pair<int, int>> horizonEdges;
            for (const auto& edgeCountPair : edgeCounts) {
                if (edgeCountPair.second == 1) {
                    horizonEdges.push_back(edgeCountPair.first);
                }
            }

            std::set<int> facesToRemove(visibleFaces.begin(), visibleFaces.end());
            std::vector<std::vector<int>> newHullFaces;
            for (size_t faceIdx = 0; faceIdx < hullFaces.size(); ++faceIdx) {
                if (facesToRemove.find(static_cast<int>(faceIdx)) == facesToRemove.end()) {
                    newHullFaces.push_back(hullFaces[faceIdx]);
                }
            }
            hullFaces.swap(newHullFaces);

            int newVertexIndex = static_cast<int>(hullVertices.size());
            hullVertices.push_back(point);

            for (const auto& edge : horizonEdges) {
                hullFaces.push_back({edge.first, edge.second, newVertexIndex});
            }
        }
    }
}


template <typename T>
T ConvexHull<T>::distanceToFace(const Face& face, const Point3D<T>& point) const {
    const Point3D<T>& a = hullVertices[face.indices[0]];
    Point3D<T> vec = point - a;
    T dist = vec.x * face.normal.x + vec.y * face.normal.y + vec.z * face.normal.z;
    return dist;
}

template <typename T>
Point3D<T> ConvexHull<T>::computeNormal(const Point3D<T>& a, const Point3D<T>& b, const Point3D<T>& c) const {
    Point3D<T> ab = b - a;
    Point3D<T> ac = c - a;

    Point3D<T> normal(
        ab.y * ac.z - ab.z * ac.y,
        ab.z * ac.x - ab.x * ac.z,
        ab.x * ac.y - ab.y * ac.x
    );

    T length = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    if (length > std::numeric_limits<T>::epsilon()) {
        normal.x /= length;
        normal.y /= length;
        normal.z /= length;
    }
    return normal;
}

template <typename T>
const std::vector<Point3D<T>>& ConvexHull<T>::getVertices() const {
    return hullVertices;
}

template <typename T>
const std::vector<std::vector<int>>& ConvexHull<T>::getFaces() const {
    return hullFaces;
}

template class ConvexHull<float>;
template class ConvexHull<double>;

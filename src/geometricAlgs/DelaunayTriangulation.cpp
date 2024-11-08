#include "includes/DelaunayTriangulation.h"
#include <cmath>
#include <algorithm>
#include <mutex>
#include <map>
#include <set>

template <typename T>
DelaunayTriangulation<T>::DelaunayTriangulation() {}

template <typename T>
DelaunayTriangulation<T>::~DelaunayTriangulation() {}

template <typename T>
void DelaunayTriangulation<T>::insertPoint(const Point3D<T>& point) {
    std::lock_guard<std::mutex> lock(mutex);

    int pointIndex = static_cast<int>(vertices.size());
    vertices.push_back(point);

    if (tetrahedra.empty()) {
        initialize();
    } else {
        updateTriangulation(pointIndex);
    }
}

template <typename T>
void DelaunayTriangulation<T>::initialize() {
    T bigValue = static_cast<T>(1e6);
    vertices.push_back(Point3D<T>(-bigValue, -bigValue, -bigValue));
    vertices.push_back(Point3D<T>(bigValue, -bigValue, -bigValue));
    vertices.push_back(Point3D<T>(0, bigValue, -bigValue));
    vertices.push_back(Point3D<T>(0, 0, bigValue));

    tetrahedra.push_back({0, 1, 2, 3});
}

template <typename T>
void DelaunayTriangulation<T>::updateTriangulation(int pointIndex) {
    std::vector<int> badTetrahedra;
    std::map<std::pair<int, int>, int> edgeCounts;

    for (size_t i = 0; i < tetrahedra.size(); ++i) {
        if (isPointInCircumsphere(tetrahedra[i], vertices[pointIndex])) {
            badTetrahedra.push_back(static_cast<int>(i));

            for (int j = 0; j < 4; ++j) {
                for (int k = j + 1; k < 4; ++k) {
                    int idx1 = tetrahedra[i][j];
                    int idx2 = tetrahedra[i][k];
                    std::pair<int, int> edge(std::min(idx1, idx2), std::max(idx1, idx2));
                    edgeCounts[edge]++;
                }
            }
        }
    }

    std::vector<std::array<int, 3>> boundaryFaces;
    for (const auto& edge : edgeCounts) {
        if (edge.second == 1) {
            boundaryFaces.push_back({edge.first.first, edge.first.second, pointIndex});
        }
    }

    for (int i = static_cast<int>(badTetrahedra.size()) - 1; i >= 0; --i) {
        tetrahedra.erase(tetrahedra.begin() + badTetrahedra[i]);
    }

    for (const auto& face : boundaryFaces) {
        tetrahedra.push_back({face[0], face[1], pointIndex, face[2]});
    }
}

template <typename T>
bool DelaunayTriangulation<T>::isPointInCircumsphere(const std::array<int, 4>& tetIndices, const Point3D<T>& point) const {
    const auto& p0 = vertices[tetIndices[0]];
    const auto& p1 = vertices[tetIndices[1]];
    const auto& p2 = vertices[tetIndices[2]];
    const auto& p3 = vertices[tetIndices[3]];

    T a[5][5] = {
        {p0.x, p0.y, p0.z, p0.x * p0.x + p0.y * p0.y + p0.z * p0.z, 1},
        {p1.x, p1.y, p1.z, p1.x * p1.x + p1.y * p1.y + p1.z * p1.z, 1},
        {p2.x, p2.y, p2.z, p2.x * p2.x + p2.y * p2.y + p2.z * p2.z, 1},
        {p3.x, p3.y, p3.z, p3.x * p3.x + p3.y * p3.y + p3.z * p3.z, 1},
        {point.x, point.y, point.z, point.x * point.x + point.y * point.y + point.z * point.z, 1}
    };

    T determinant = computeDeterminant(a);

    return determinant > 0;
}

template <typename T>
T DelaunayTriangulation<T>::computeDeterminant(T matrix[5][5]) const {
    T det = 0;
    int sign = 1;

    for (int i = 0; i < 5; ++i) {
        T subMatrix[4][4];
        for (int j = 1; j < 5; ++j) {
            int colIndex = 0;
            for (int k = 0; k < 5; ++k) {
                if (k == i) continue;
                subMatrix[j - 1][colIndex] = matrix[j][k];
                colIndex++;
            }
        }
        det += sign * matrix[0][i] * compute4x4Determinant(subMatrix);
        sign = -sign;
    }
    return det;
}

template <typename T>
T DelaunayTriangulation<T>::compute4x4Determinant(T matrix[4][4]) const {
    T det = 0;
    int sign = 1;

    for (int i = 0; i < 4; ++i) {
        T subMatrix[3][3];
        for (int j = 1; j < 4; ++j) {
            int colIndex = 0;
            for (int k = 0; k < 4; ++k) {
                if (k == i) continue;
                subMatrix[j - 1][colIndex] = matrix[j][k];
                colIndex++;
            }
        }
        det += sign * matrix[0][i] * compute3x3Determinant(subMatrix);
        sign = -sign;
    }
    return det;
}

template <typename T>
T DelaunayTriangulation<T>::compute3x3Determinant(T matrix[3][3]) const {
    return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
           matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
           matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}

template <typename T>
const std::vector<std::array<int, 4>>& DelaunayTriangulation<T>::getTetrahedra() const {
    return tetrahedra;
}

template <typename T>
const std::vector<Point3D<T>>& DelaunayTriangulation<T>::getVertices() const {
    return vertices;
}

template class DelaunayTriangulation<float>;
template class DelaunayTriangulation<double>;

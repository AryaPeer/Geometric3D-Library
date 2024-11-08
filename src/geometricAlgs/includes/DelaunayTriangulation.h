#ifndef DELAUNAYTRIANGULATION_H
#define DELAUNAYTRIANGULATION_H

#include <vector>
#include <array>
#include <mutex>
#include "Point3D.h"

// DelaunayTriangulation class template
template <typename T>
class DelaunayTriangulation {
public:
    DelaunayTriangulation();
    ~DelaunayTriangulation();
    void insertPoint(const Point3D<T>& point);
    const std::vector<std::array<int, 4>>& getTetrahedra() const;
    const std::vector<Point3D<T>>& getVertices() const;

private:
    std::vector<Point3D<T>> vertices;
    std::vector<std::array<int, 4>> tetrahedra;

    mutable std::mutex mutex;

    void initialize();

    void updateTriangulation(int pointIndex);

    bool isPointInCircumsphere(const std::array<int, 4>& tetIndices, const Point3D<T>& point) const;

    T computeDeterminant(T matrix[5][5]) const;

    T compute4x4Determinant(T matrix[4][4]) const;

    T compute3x3Determinant(T matrix[3][3]) const;
};

#endif // DELAUNAYTRIANGULATION_H

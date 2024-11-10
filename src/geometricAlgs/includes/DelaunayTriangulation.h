#ifndef DELAUNAYTRIANGULATION_H
#define DELAUNAYTRIANGULATION_H

#include <vector>
#include <array>
#include <unordered_set>
#include "Point3D.h"

template <typename T>
class DelaunayTriangulation {
public:
    DelaunayTriangulation();
    ~DelaunayTriangulation();

    void insertPoint(const Point3D<T>& point);
    void insertConstraint(const Point3D<T>& p1, const Point3D<T>& p2);

    const std::vector<std::array<int, 3>>& getTriangles() const;
    const std::vector<Point3D<T>>& getVertices() const;

private:
    struct Edge {
        int p1, p2;
        Edge(int a, int b) : p1(a < b ? a : b), p2(a < b ? b : a) {}
        bool operator==(const Edge& other) const {
            return p1 == other.p1 && p2 == other.p2;
        }
    };

    struct EdgeHash {
        size_t operator()(const Edge& edge) const {
            return std::hash<int>()(edge.p1) ^ std::hash<int>()(edge.p2);
        }
    };

    std::vector<Point3D<T>> vertices;
    std::vector<std::array<int, 3>> triangles;

    std::unordered_set<Edge, EdgeHash> constraints;

    void initialize();
    void legalizeEdge(int a, int b, int c, int triangleIndex);
    bool isEdgeIllegal(int a, int b, int c, int d) const;
    bool isPointInCircumcircle(int a, int b, int c, int d) const;
    T orient2D(const Point3D<T>& a, const Point3D<T>& b, const Point3D<T>& c) const;
};

#endif // DELAUNAYTRIANGULATION_H

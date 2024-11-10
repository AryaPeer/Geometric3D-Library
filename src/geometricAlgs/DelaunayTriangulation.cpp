#include "DelaunayTriangulation.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stack>

template <typename T>
DelaunayTriangulation<T>::DelaunayTriangulation() {
    initialize();
}

template <typename T>
DelaunayTriangulation<T>::~DelaunayTriangulation() {}

template <typename T>
void DelaunayTriangulation<T>::initialize() {
    T bigValue = std::numeric_limits<T>::max() / static_cast<T>(10);

    vertices.push_back(Point3D<T>(-bigValue, -bigValue, 0));
    vertices.push_back(Point3D<T>(bigValue, -bigValue, 0));
    vertices.push_back(Point3D<T>(0, bigValue, 0));

    triangles.push_back({0, 1, 2});
}

template <typename T>
void DelaunayTriangulation<T>::insertPoint(const Point3D<T>& point) {
    int pointIndex = static_cast<int>(vertices.size());
    vertices.push_back(point);

    std::vector<int> badTriangles;
    std::vector<Edge> polygon;

    for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
        const auto& tri = triangles[i];
        if (isPointInCircumcircle(tri[0], tri[1], tri[2], pointIndex)) {
            badTriangles.push_back(i);

            Edge edges[3] = {Edge(tri[0], tri[1]), Edge(tri[1], tri[2]), Edge(tri[2], tri[0])};
            for (const auto& edge : edges) {
                auto it = std::find(polygon.begin(), polygon.end(), edge);
                if (it != polygon.end()) {
                    polygon.erase(it);
                } else {
                    polygon.push_back(edge);
                }
            }
        }
    }

    // Remove bad triangles
    for (int i = static_cast<int>(badTriangles.size()) - 1; i >= 0; --i) {
        triangles.erase(triangles.begin() + badTriangles[i]);
    }

    // Re-triangulate the hole
    for (const auto& edge : polygon) {
        triangles.push_back({edge.p1, edge.p2, pointIndex});
        int triangleIndex = static_cast<int>(triangles.size()) - 1;
        legalizeEdge(edge.p1, edge.p2, pointIndex, triangleIndex);
    }
}

template <typename T>
void DelaunayTriangulation<T>::insertConstraint(const Point3D<T>& p1, const Point3D<T>& p2) {
    int idx1 = -1, idx2 = -1;
    for (int i = 0; i < static_cast<int>(vertices.size()); ++i) {
        if (vertices[i] == p1) {
            idx1 = i;
        }
        if (vertices[i] == p2) {
            idx2 = i;
        }
        if (idx1 != -1 && idx2 != -1) {
            break;
        }
    }

    if (idx1 == -1) {
        idx1 = static_cast<int>(vertices.size());
        vertices.push_back(p1);
    }
    if (idx2 == -1) {
        idx2 = static_cast<int>(vertices.size());
        vertices.push_back(p2);
    }

    constraints.insert(Edge(idx1, idx2));

    insertPoint(p1);
    insertPoint(p2);

    // Enforce the constraint by edge flipping
    std::stack<Edge> edgeStack;
    edgeStack.push(Edge(idx1, idx2));

    while (!edgeStack.empty()) {
        Edge edge = edgeStack.top();
        edgeStack.pop();

        for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
            const auto& tri = triangles[i];

            // Check if the triangle contains the edge
            if ((tri[0] == edge.p1 && tri[1] == edge.p2) ||
                (tri[1] == edge.p1 && tri[2] == edge.p2) ||
                (tri[2] == edge.p1 && tri[0] == edge.p2) ||
                (tri[0] == edge.p2 && tri[1] == edge.p1) ||
                (tri[1] == edge.p2 && tri[2] == edge.p1) ||
                (tri[2] == edge.p2 && tri[0] == edge.p1)) {

                // Find the opposite point
                int opposite = -1;
                for (int j = 0; j < 3; ++j) {
                    if (tri[j] != edge.p1 && tri[j] != edge.p2) {
                        opposite = tri[j];
                        break;
                    }
                }

                // Check if the edge is illegal
                if (isEdgeIllegal(edge.p1, edge.p2, opposite, -1)) {
                    // Flip the edge
                    triangles.erase(triangles.begin() + i);
                    triangles.push_back({edge.p1, opposite, edge.p2});
                    triangles.push_back({edge.p2, opposite, edge.p1});

                    // Add new edges to the stack
                    edgeStack.push(Edge(edge.p1, opposite));
                    edgeStack.push(Edge(edge.p2, opposite));
                    break;
                }
            }
        }
    }
}

template <typename T>
void DelaunayTriangulation<T>::legalizeEdge(int a, int b, int c, int triangleIndex) {
    // Find the triangle adjacent to (a, b)
    for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
        if (i == triangleIndex) continue;

        const auto& tri = triangles[i];
        if ((tri[0] == b && tri[1] == a) ||
            (tri[1] == b && tri[2] == a) ||
            (tri[2] == b && tri[0] == a) ||
            (tri[0] == a && tri[1] == b) ||
            (tri[1] == a && tri[2] == b) ||
            (tri[2] == a && tri[0] == b)) {

            int d = -1;
            for (int j = 0; j < 3; ++j) {
                if (tri[j] != a && tri[j] != b) {
                    d = tri[j];
                    break;
                }
            }

            if (isEdgeIllegal(a, b, c, d)) {
                // Remove the two triangles sharing the edge (a, b)
                triangles.erase(triangles.begin() + std::max(i, triangleIndex));
                triangles.erase(triangles.begin() + std::min(i, triangleIndex));

                // Create new triangles by flipping the edge
                triangles.push_back({a, c, d});
                triangles.push_back({b, d, c});

                // Legalize new edges
                legalizeEdge(a, d, c, static_cast<int>(triangles.size()) - 2);
                legalizeEdge(b, c, d, static_cast<int>(triangles.size()) - 1);
            }
            break;
        }
    }
}

template <typename T>
bool DelaunayTriangulation<T>::isEdgeIllegal(int a, int b, int c, int d) const {
    if (constraints.find(Edge(a, b)) != constraints.end()) {
        return false;
    }
    if (d == -1) {
        return false;
    }
    return isPointInCircumcircle(a, b, c, d);
}

template <typename T>
bool DelaunayTriangulation<T>::isPointInCircumcircle(int a, int b, int c, int d) const {
    const auto& pa = vertices[a];
    const auto& pb = vertices[b];
    const auto& pc = vertices[c];
    const auto& pd = vertices[d];

    T ax = pa.x - pd.x;
    T ay = pa.y - pd.y;
    T bx = pb.x - pd.x;
    T by = pb.y - pd.y;
    T cx = pc.x - pd.x;
    T cy = pc.y - pd.y;

    T det = (ax * ax + ay * ay) * (bx * cy - cx * by)
          - (bx * bx + by * by) * (ax * cy - cx * ay)
          + (cx * cx + cy * cy) * (ax * by - bx * ay);

    return det > static_cast<T>(0);
}

template <typename T>
T DelaunayTriangulation<T>::orient2D(const Point3D<T>& a, const Point3D<T>& b, const Point3D<T>& c) const {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

template <typename T>
const std::vector<std::array<int, 3>>& DelaunayTriangulation<T>::getTriangles() const {
    return triangles;
}

template <typename T>
const std::vector<Point3D<T>>& DelaunayTriangulation<T>::getVertices() const {
    return vertices;
}

template class DelaunayTriangulation<float>;
template class DelaunayTriangulation<double>;

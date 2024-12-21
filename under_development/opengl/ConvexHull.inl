#include <algorithm>
#include <limits>
#include <set>
#include <stack>

template <typename T>
struct Face {
    int a, b, c; 
    Point3D<T> normal;

    Face(int v0, int v1, int v2, const Point3D<T>& n)
        : a(v0), b(v1), c(v2), normal(n) {}
};

template <typename T>
void ConvexHull<T>::compute(const std::vector<Point3D<T>>& points, std::vector<Point3D<T>>& hullVertices, std::vector<std::vector<int>>& hullFaces) {

    if (points.size() < 4) {
        hullVertices = points;
        hullFaces.clear();
        return;
    }

    int minX = 0, maxX = 0;
    int minY = 0, maxY = 0;
    int minZ = 0, maxZ = 0;

    for (size_t i = 1; i < points.size(); ++i) {
        if (points[i].x < points[minX].x) minX = i;
        if (points[i].x > points[maxX].x) maxX = i;
        if (points[i].y < points[minY].y) minY = i;
        if (points[i].y > points[maxY].y) maxY = i;
        if (points[i].z < points[minZ].z) minZ = i;
        if (points[i].z > points[maxZ].z) maxZ = i;
    }

    std::set<int> initialVertices = { minX, maxX, minY, maxY, minZ, maxZ };
    std::vector<int> vertices(initialVertices.begin(), initialVertices.end());

    if (vertices.size() < 4) {
        hullVertices = points;
        hullFaces.clear();
        return;
    }

    hullVertices = points;

    std::vector<Face<T>> faces;
    faces.emplace_back(vertices[0], vertices[1], vertices[2], Point3D<T>());
    faces.emplace_back(vertices[0], vertices[1], vertices[3], Point3D<T>());
    faces.emplace_back(vertices[0], vertices[2], vertices[3], Point3D<T>());
    faces.emplace_back(vertices[1], vertices[2], vertices[3], Point3D<T>());

    hullFaces.clear();
    for (const auto& face : faces) {
        hullFaces.push_back({ face.a, face.b, face.c });
    }
}


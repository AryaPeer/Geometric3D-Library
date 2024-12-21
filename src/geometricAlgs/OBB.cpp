#include "includes/OBB.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

template <typename T>
OBB<T>::OBB()
    : center(0, 0, 0), axes{Point3D<T>(1, 0, 0), Point3D<T>(0, 1, 0), Point3D<T>(0, 0, 1)}, extents{1, 1, 1} {}

template <typename T>
OBB<T>::OBB(const Point3D<T>& centerPoint, const std::array<Point3D<T>, 3>& axes, const std::array<T, 3>& extents)
    : center(centerPoint), axes(axes), extents(extents) {}

template <typename T>
bool OBB<T>::contains(const Point3D<T>& point) const {
    Point3D<T> d = point - center;
    for (int i = 0; i < 3; ++i) {
        T dist = d.dot(axes[i]);
        if (std::abs(dist) > extents[i]) {
            return false;
        }
    }
    return true;
}

template <typename T>
void OBB<T>::transform(const Eigen::Transform<T, 3, Eigen::Affine>& transform) {
    Eigen::Matrix<T, 3, 1> centerVec(center.x, center.y, center.z);
    centerVec = transform * centerVec;
    center = Point3D<T>(centerVec[0], centerVec[1], centerVec[2]);

    for (int i = 0; i < 3; ++i) {
        Eigen::Matrix<T, 3, 1> axisVec(axes[i].x, axes[i].y, axes[i].z);
        axisVec = transform.linear() * axisVec;
        axisVec.normalize();  // Ensure the axis remains a unit vector
        axes[i] = Point3D<T>(axisVec[0], axisVec[1], axisVec[2]);
    }
}

template <typename T>
OBB<T> OBB<T>::computeFromPoints(const std::vector<Point3D<T>>& points) {
    if (points.empty()) {
        throw std::invalid_argument("Point set is empty");
    }

    Eigen::Matrix<T, 3, 1> meanVec(0, 0, 0);
    for (const auto& p : points) {
        meanVec += Eigen::Matrix<T, 3, 1>(p.x, p.y, p.z);
    }
    meanVec /= static_cast<T>(points.size());

    Eigen::Matrix<T, 3, 3> covMatrix = Eigen::Matrix<T, 3, 3>::Zero();
    for (const auto& p : points) {
        Eigen::Matrix<T, 3, 1> d(p.x - meanVec[0], p.y - meanVec[1], p.z - meanVec[2]);
        covMatrix += d * d.transpose();
    }
    covMatrix /= static_cast<T>(points.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 3, 3>> solver(covMatrix);
    Eigen::Matrix<T, 3, 1> eigenvalues = solver.eigenvalues();
    Eigen::Matrix<T, 3, 3> eigenvectors = solver.eigenvectors();

    std::array<Point3D<T>, 3> axes = {
        Point3D<T>(eigenvectors(0, 2), eigenvectors(1, 2), eigenvectors(2, 2)),
        Point3D<T>(eigenvectors(0, 1), eigenvectors(1, 1), eigenvectors(2, 1)),
        Point3D<T>(eigenvectors(0, 0), eigenvectors(1, 0), eigenvectors(2, 0))
    };

    std::array<T, 3> minProj = {std::numeric_limits<T>::max(), std::numeric_limits<T>::max(), std::numeric_limits<T>::max()};
    std::array<T, 3> maxProj = {std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest()};

    for (const auto& p : points) {
        Eigen::Matrix<T, 3, 1> d(p.x - meanVec[0], p.y - meanVec[1], p.z - meanVec[2]);
        for (int i = 0; i < 3; ++i) {
            T proj = d.dot(eigenvectors.col(2 - i));
            minProj[i] = std::min(minProj[i], proj);
            maxProj[i] = std::max(maxProj[i], proj);
        }
    }

    std::array<T, 3> extents;
    Eigen::Matrix<T, 3, 1> obbCenterVec = meanVec;
    for (int i = 0; i < 3; ++i) {
        extents[i] = (maxProj[i] - minProj[i]) / 2;
        obbCenterVec += ((maxProj[i] + minProj[i]) / 2) * eigenvectors.col(2 - i);
    }

    Point3D<T> obbCenter(obbCenterVec[0], obbCenterVec[1], obbCenterVec[2]);

    return OBB<T>(obbCenter, axes, extents);
}

template <typename T>
std::array<Point3D<T>, 8> OBB<T>::getCorners() const {
    std::array<Point3D<T>, 8> corners;
    std::array<Point3D<T>, 3> axesScaled = {
        axes[0] * extents[0],
        axes[1] * extents[1],
        axes[2] * extents[2]
    };

    corners[0] = center - axesScaled[0] - axesScaled[1] - axesScaled[2];
    corners[1] = center + axesScaled[0] - axesScaled[1] - axesScaled[2];
    corners[2] = center + axesScaled[0] + axesScaled[1] - axesScaled[2];
    corners[3] = center - axesScaled[0] + axesScaled[1] - axesScaled[2];
    corners[4] = center - axesScaled[0] - axesScaled[1] + axesScaled[2];
    corners[5] = center + axesScaled[0] - axesScaled[1] + axesScaled[2];
    corners[6] = center + axesScaled[0] + axesScaled[1] + axesScaled[2];
    corners[7] = center - axesScaled[0] + axesScaled[1] + axesScaled[2];

    return corners;
}

template class OBB<float>;
template class OBB<double>;
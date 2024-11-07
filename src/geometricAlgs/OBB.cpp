#include "OBB.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <Eigen/Dense> // If you are using this library make sure to install eigen before compilation
#include <Eigen/Eigenvalues>


template <typename T>
OBB<T>::OBB()
    : center(0, 0, 0), axis{Point3D<T>(1, 0, 0), Point3D<T>(0, 1, 0), Point3D<T>(0, 0, 1)}, extent{1, 1, 1} {}

template <typename T>
OBB<T>::OBB(const Point3D<T>& centerPoint, const std::array<Point3D<T>, 3>& axes, const std::array<T, 3>& extents)
    : center(centerPoint), axis(axes), extent(extents) {}

template <typename T>
bool OBB<T>::contains(const Point3D<T>& point) const {
    Point3D<T> d = point - center;
    for (int i = 0; i < 3; ++i) {
        T dist = d.dot(axis[i]);
        if (std::abs(dist) > extent[i]) {
            return false;
        }
    }
    return true;
}

template <typename T>
void OBB<T>::transform(const std::array<std::array<T, 3>, 3>& rotationMatrix, const Point3D<T>& translation) {
    // Rotate the axes
    for (int i = 0; i < 3; ++i) {
        Point3D<T> rotatedAxis(
            axis[i].x * rotationMatrix[0][0] + axis[i].y * rotationMatrix[0][1] + axis[i].z * rotationMatrix[0][2],
            axis[i].x * rotationMatrix[1][0] + axis[i].y * rotationMatrix[1][1] + axis[i].z * rotationMatrix[1][2],
            axis[i].x * rotationMatrix[2][0] + axis[i].y * rotationMatrix[2][1] + axis[i].z * rotationMatrix[2][2]
        );
        axis[i] = rotatedAxis;
    }

    center = Point3D<T>(
        center.x * rotationMatrix[0][0] + center.y * rotationMatrix[0][1] + center.z * rotationMatrix[0][2],
        center.x * rotationMatrix[1][0] + center.y * rotationMatrix[1][1] + center.z * rotationMatrix[1][2],
        center.x * rotationMatrix[2][0] + center.y * rotationMatrix[2][1] + center.z * rotationMatrix[2][2]
    ) + translation;
}

template <typename T>
OBB<T> OBB<T>::computeFromPoints(const std::vector<Point3D<T>>& points) {
    Point3D<T> mean(0, 0, 0);
    for (const auto& p : points) {
        mean = mean + p;
    }
    mean = mean / static_cast<T>(points.size());

    T cov[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    for (const auto& p : points) {
        Point3D<T> d = p - mean;
        cov[0][0] += d.x * d.x;
        cov[0][1] += d.x * d.y;
        cov[0][2] += d.x * d.z;
        cov[1][0] += d.y * d.x;
        cov[1][1] += d.y * d.y;
        cov[1][2] += d.y * d.z;
        cov[2][0] += d.z * d.x;
        cov[2][1] += d.z * d.y;
        cov[2][2] += d.z * d.z;
    }

    Eigen::Matrix<T, 3, 3> covMatrix;
    covMatrix << cov[0][0], cov[0][1], cov[0][2],
                 cov[1][0], cov[1][1], cov[1][2],
                 cov[2][0], cov[2][1], cov[2][2];

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 3, 3>> solver(covMatrix);
    Eigen::Matrix<T, 3, 1> eigenvalues = solver.eigenvalues();
    Eigen::Matrix<T, 3, 3> eigenvectors = solver.eigenvectors();

    std::array<Point3D<T>, 3> axes;
    axes[0] = Point3D<T>(eigenvectors(0, 0), eigenvectors(1, 0), eigenvectors(2, 0));
    axes[1] = Point3D<T>(eigenvectors(0, 1), eigenvectors(1, 1), eigenvectors(2, 1));
    axes[2] = Point3D<T>(eigenvectors(0, 2), eigenvectors(1, 2), eigenvectors(2, 2));

    T minProj[3] = {std::numeric_limits<T>::max(), std::numeric_limits<T>::max(), std::numeric_limits<T>::max()};
    T maxProj[3] = {std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest()};

    for (const auto& p : points) {
        Point3D<T> d = p - mean;
        for (int i = 0; i < 3; ++i) {
            T proj = d.dot(axes[i]);
            minProj[i] = std::min(minProj[i], proj);
            maxProj[i] = std::max(maxProj[i], proj);
        }
    }

    std::array<T, 3> extents;
    for (int i = 0; i < 3; ++i) {
        extents[i] = (maxProj[i] - minProj[i]) / 2;
    }

    Point3D<T> obbCenter = mean;
    for (int i = 0; i < 3; ++i) {
        obbCenter = obbCenter + axes[i] * ((maxProj[i] + minProj[i]) / 2);
    }

    return OBB<T>(obbCenter, axes, extents);
}

template <typename T>
std::array<Point3D<T>, 8> OBB<T>::getCorners() const {
    std::array<Point3D<T>, 8> corners;
    std::array<Point3D<T>, 3> axesScaled = {
        axis[0] * extent[0],
        axis[1] * extent[1],
        axis[2] * extent[2]
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

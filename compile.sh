#!/bin/bash

EIGEN_PATH="/usr/local/include/eigen3"  

if [ ! -d "$EIGEN_PATH" ]; then
  echo "Eigen library not found. Please install Eigen to the path directory."
  exit 1
fi

mkdir -p bin/static
rm -f bin/static/*.o bin/static/*.a bin/main.o

g++ -c src/geometricAlgs/Point3D.cpp -o bin/static/Point3D.o
g++ -c src/geometricAlgs/KDTree.cpp -o bin/static/KDTree.o
g++ -c src/geometricAlgs/Octree.cpp -o bin/static/Octree.o
g++ -c src/geometricAlgs/AABB.cpp -o bin/static/AABB.o
g++ -c src/geometricAlgs/BVH.cpp -o bin/static/BVH.o
g++ -c src/geometricAlgs/ConvexHull.cpp -o bin/static/ConvexHull.o
g++ -c src/geometricAlgs/DelaunayTriangulation.cpp -o bin/static/DelaunayTriangulation.o
g++ -I "$EIGEN_PATH" -c src/geometricAlgs/OBB.cpp -o bin/static/OBB.o
g++ -c src/geometricAlgs/Ray.cpp -o bin/static/Ray.o

g++ -I "$EIGEN_PATH" -c src/main.cpp -o bin/main.o

ar rcs bin/static/geometricAlgsLib.a bin/static/*.o

g++ bin/main.o -Lbin/static -l:geometricAlgsLib.a -o bin/execute_program

echo "Compilation complete. Run ./bin/execute_program to execute the program."

# Geometric3D-Library

A comprehensive C++ library for 3D geometric algorithms and data structures, offering efficient implementations of common geometric operations and spatial data structures.

## Features

### Geometric Algorithms
- Convex Hull: Parallel 3D convex hull computation
- Delaunay Triangulation: 2D triangulation with constraint support
- Ray-Triangle Intersection: Fast geometry testing

### Spatial Data Structures:
- KD-Tree: For efficient spatial searches
- Octree: For spatial partitioning and LOD
- BVH (Bounding Volume Hierarchy): For collision detection and ray tracing
- AABB (Axis-Aligned Bounding Box): For basic collision detection

### Geometric Primitives:
- Point3D: Template-based 3D point/vector implementation
- Ray: For ray casting and intersection tests
- OBB (Oriented Bounding Box): For tight-fitting collision bounds

### Thread Safety:
- Concurrent access support
- Mutex-protected operations
- Thread-safe data structure modifications

### Under Development:
- OpenGL Visualization Tools
- CUDA-accelerated Ray-Triangle Intersection

## Dependencies

- C++11 or later
- Eigen3 Library (for matrix operations)
- OpenGL & GLFW (for visualization tools - optional(since still being developed))
- CUDA Toolkit (for GPU acceleration - optional(since still being developed))

## Installation

1. **Clone the Repository**
   ```bash
   git clone https://github.com/AryaPeer/Geometric3D-Library.git
   cd Geometric3D-Library
   ```

2. **Install Dependencies**
   - For Debian/Ubuntu:
     ```bash
     sudo apt-get install libeigen3-dev
     ```
   - For macOS:
     ```bash
     brew install eigen
     ```

3. **Build the Library**
   ```bash
   bash compile.sh
   ```

## Usage

1.**Navigate to main.cpp under src**

2. **Edit File To Include Required Headers**
   ```cpp
   #include "geometricAlgs/includes/Point3D.h"
   #include "geometricAlgs/includes/KDTree.h"
   // ... other headers as needed
   ```

3. **Implement Watever You Want (Basic Example Below)**
   ```cpp
   #include "geometricAlgs/includes/Point3D.h"
   #include "geometricAlgs/includes/KDTree.h"
   
   int main() {
       // Create points
       std::vector<Point3D<float>> points;
       points.push_back(Point3D<float>(1.0f, 2.0f, 3.0f));
       points.push_back(Point3D<float>(4.0f, 5.0f, 6.0f));
       
       // Create and build KD-tree
       KDTree<float> kdTree;
       kdTree.build(points);
       
       return 0;
   }
   ```

## Project Structure

```
Geometric3D-Library/
├── src/
│   ├── geometricAlgs/
│   │   ├── includes/      # Header files
│   │   └── ...           # Implementation files
│   └── main.cpp
├── under_development/
│   ├── cuda/            # CUDA implementations (still under development)
│   └── opengl/          # OpenGL visualizations (still under development)
├── bin/
│   └── static/         # Compiled objects and libraries
├── compile.sh          # Build script
└── README.md
```

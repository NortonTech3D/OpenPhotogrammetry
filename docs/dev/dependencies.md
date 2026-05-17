# Dependency Foundation

Dependency management is anchored on `vcpkg.json` in manifest mode with a pinned `builtin-baseline`.

## Core stack
- Build/toolchain: CMake, Ninja, ccache, vcpkg
- Core C++: Eigen, fmt, spdlog, CLI11, nlohmann-json
- Imaging/IO: OpenCV, OpenImageIO, libjpeg-turbo, libpng, libtiff, Exiv2
- Geometry/point cloud: CGAL, PCL
- Optimization: Ceres, SuiteSparse, glog, gflags
- GUI/visualization: Qt6 (`qtbase`), ImGui, VTK
- Testing: Catch2
- Python bridge: pybind11

## Optional or external integrations (initially non-vendored)
- Open3D (Python/C++ pipeline bridge)
- Interoperability adapters for COLMAP, OpenMVG, OpenMVS
- CUDA (NVIDIA) and Metal (Apple Silicon)

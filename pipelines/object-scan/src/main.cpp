#include <iostream>

#include "openphotogrammetry/camera/camera.hpp"
#include "openphotogrammetry/meshing/meshing.hpp"

int main() {
  const bool ok = op::camera::has_valid_default_calibration() && op::meshing::can_generate_mesh();
  std::cout << "pipeline.object_scan=" << (ok ? "ok" : "failed") << '\n';
  return ok ? 0 : 1;
}

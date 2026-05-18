#include <iostream>

#include "openphotogrammetry/camera/camera.hpp"
#include "openphotogrammetry/meshing/meshing.hpp"

int main() {
  const bool ready = op::camera::has_valid_default_calibration() && op::meshing::can_generate_mesh();
  std::cout << "gui.ready=" << (ready ? "true" : "false") << '\n';
  return ready ? 0 : 1;
}

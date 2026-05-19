#include <iostream>

#include "openphotogrammetry/sfm/sfm.hpp"
#include "openphotogrammetry/texturing/texturing.hpp"

int main() {
  const bool ready = op::sfm::can_initialize_reconstruction() && op::texturing::can_texture_mesh();
  std::cout << "cli.ready=" << (ready ? "true" : "false") << '\n';
  return ready ? 0 : 1;
}

#include <iostream>

#include "openphotogrammetry/sfm/sfm.hpp"
#include "openphotogrammetry/texturing/texturing.hpp"

int main() {
  const bool ok = op::sfm::can_initialize_reconstruction() && op::texturing::can_texture_mesh();
  std::cout << "pipeline.reconstruction=" << (ok ? "ok" : "failed") << '\n';
  return ok ? 0 : 1;
}

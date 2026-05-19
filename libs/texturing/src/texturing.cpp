#include "openphotogrammetry/texturing/texturing.hpp"

#include "openphotogrammetry/io/io.hpp"
#include "openphotogrammetry/meshing/meshing.hpp"

namespace op::texturing {

int texture_resolution() {
  if (!op::meshing::can_generate_mesh()) {
    return 0;
  }

  return op::io::can_read_project_manifest("dataset.json") ? 2048 : 0;
}

bool can_texture_mesh() {
  return texture_resolution() >= 1024;
}

}  // namespace op::texturing

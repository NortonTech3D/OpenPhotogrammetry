#include "openphotogrammetry/mvs/mvs.hpp"

#include "openphotogrammetry/sfm/sfm.hpp"

namespace op::mvs {

int depth_map_layers() {
  return op::sfm::can_initialize_reconstruction() ? 4 : 0;
}

bool can_dense_reconstruct() {
  return depth_map_layers() >= 1;
}

}  // namespace op::mvs

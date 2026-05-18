#include "openphotogrammetry/meshing/meshing.hpp"

#include "openphotogrammetry/mvs/mvs.hpp"

namespace op::meshing {

int target_face_budget() {
  return op::mvs::can_dense_reconstruct() ? 50000 : 0;
}

bool can_generate_mesh() {
  return target_face_budget() > 0;
}

}  // namespace op::meshing

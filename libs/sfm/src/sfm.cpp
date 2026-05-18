#include "openphotogrammetry/sfm/sfm.hpp"

#include "openphotogrammetry/features/features.hpp"

namespace op::sfm {

int minimum_view_count() {
  return op::features::has_viable_feature_budget() ? 2 : 0;
}

bool can_initialize_reconstruction() {
  return minimum_view_count() >= 2;
}

}  // namespace op::sfm

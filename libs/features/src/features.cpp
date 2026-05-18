#include "openphotogrammetry/features/features.hpp"

#include "openphotogrammetry/camera/camera.hpp"

namespace op::features {

int recommended_feature_count() {
  return op::camera::has_valid_default_calibration() ? 2048 : 0;
}

bool has_viable_feature_budget() {
  return recommended_feature_count() >= 512;
}

}  // namespace op::features

#include "openphotogrammetry/camera/camera.hpp"

#include "openphotogrammetry/core/core.hpp"

namespace op::camera {

double default_focal_length_px() {
  return 1000.0;
}

bool has_valid_default_calibration() {
  return default_focal_length_px() > 0.0 && op::core::is_runtime_ready();
}

}  // namespace op::camera

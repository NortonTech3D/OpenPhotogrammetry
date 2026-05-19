#include "openphotogrammetry/core/core.hpp"

namespace op::core {

int version_major() {
  return 1;
}

bool is_runtime_ready() {
  return version_major() == 1;
}

}  // namespace op::core

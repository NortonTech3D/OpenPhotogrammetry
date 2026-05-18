#include <cstdint>

#include "openphotogrammetry/features/features.hpp"

int main() {
  std::int64_t accumulator = 0;
  for (int i = 0; i < 1000; ++i) {
    accumulator += op::features::recommended_feature_count();
  }

  return accumulator > 0 ? 0 : 1;
}

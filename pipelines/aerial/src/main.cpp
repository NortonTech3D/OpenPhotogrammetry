#include <iostream>

#include "openphotogrammetry/features/features.hpp"
#include "openphotogrammetry/geo/geo.hpp"

int main() {
  const bool ok = op::features::has_viable_feature_budget() && op::geo::supports_wgs84();
  std::cout << "pipeline.aerial=" << (ok ? "ok" : "failed") << '\n';
  return ok ? 0 : 1;
}

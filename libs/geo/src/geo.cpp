#include "openphotogrammetry/geo/geo.hpp"

#include "openphotogrammetry/core/core.hpp"

namespace op::geo {

bool supports_wgs84() {
  return op::core::is_runtime_ready();
}

int epsg_code() {
  return supports_wgs84() ? 4326 : 0;
}

}  // namespace op::geo

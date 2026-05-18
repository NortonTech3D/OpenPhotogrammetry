#include "openphotogrammetry/io/io.hpp"

#include "openphotogrammetry/core/core.hpp"

namespace op::io {

bool can_read_project_manifest(std::string_view path) {
  return !path.empty() && op::core::is_runtime_ready();
}

}  // namespace op::io

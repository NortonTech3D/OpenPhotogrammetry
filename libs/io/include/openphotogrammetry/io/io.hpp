#pragma once

#include <string_view>

namespace op::io {

bool can_read_project_manifest(std::string_view path);

}  // namespace op::io

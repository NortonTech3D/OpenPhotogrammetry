#include <iostream>
#include <string_view>

#include "openphotogrammetry/camera/camera.hpp"
#include "openphotogrammetry/meshing/meshing.hpp"

int main(int argc, char** argv) {
  bool live_scan_enabled = false;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg{argv[i]};
    if (arg == "--live-scan") {
      live_scan_enabled = true;
      continue;
    }

    std::cerr << "unsupported option: " << arg << '\n';
    return 2;
  }

  const bool ok = op::camera::has_valid_default_calibration() && op::meshing::can_generate_mesh();
  std::cout << "pipeline.object_scan=" << (ok ? "ok" : "failed")
            << " mode=" << (live_scan_enabled ? "live" : "batch") << '\n';
  return ok ? 0 : 1;
}

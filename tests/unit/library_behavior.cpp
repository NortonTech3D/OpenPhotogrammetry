#include <iostream>
#include <string_view>

#include "openphotogrammetry/camera/camera.hpp"
#include "openphotogrammetry/core/core.hpp"
#include "openphotogrammetry/features/features.hpp"
#include "openphotogrammetry/geo/geo.hpp"
#include "openphotogrammetry/io/io.hpp"
#include "openphotogrammetry/meshing/meshing.hpp"
#include "openphotogrammetry/mvs/mvs.hpp"
#include "openphotogrammetry/sfm/sfm.hpp"
#include "openphotogrammetry/texturing/texturing.hpp"

namespace {

bool check_library(std::string_view name) {
  if (name == "core") {
    return op::core::version_major() == 1 && op::core::is_runtime_ready();
  }

  if (name == "io") {
    return op::io::can_read_project_manifest("manifest.json");
  }

  if (name == "camera") {
    return op::camera::default_focal_length_px() > 0.0 && op::camera::has_valid_default_calibration();
  }

  if (name == "features") {
    return op::features::recommended_feature_count() >= 512 && op::features::has_viable_feature_budget();
  }

  if (name == "sfm") {
    return op::sfm::minimum_view_count() >= 2 && op::sfm::can_initialize_reconstruction();
  }

  if (name == "mvs") {
    return op::mvs::depth_map_layers() > 0 && op::mvs::can_dense_reconstruct();
  }

  if (name == "meshing") {
    return op::meshing::target_face_budget() > 0 && op::meshing::can_generate_mesh();
  }

  if (name == "texturing") {
    return op::texturing::texture_resolution() >= 1024 && op::texturing::can_texture_mesh();
  }

  if (name == "geo") {
    return op::geo::epsg_code() == 4326 && op::geo::supports_wgs84();
  }

  return false;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "expected one library name argument\n";
    return 2;
  }

  const std::string_view name{argv[1]};
  if (!check_library(name)) {
    std::cerr << "behavior check failed for " << name << '\n';
    return 1;
  }

  return 0;
}

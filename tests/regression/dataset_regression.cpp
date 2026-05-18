#include <fstream>
#include <iostream>
#include <string>

#include "openphotogrammetry/io/io.hpp"
#include "openphotogrammetry/texturing/texturing.hpp"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "expected dataset path argument\n";
    return 2;
  }

  std::ifstream file(argv[1]);
  if (!file) {
    std::cerr << "unable to open dataset file\n";
    return 1;
  }

  std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  const bool has_schema = content.find("\"schema\": \"op-regression-dataset-v1\"") != std::string::npos;
  const bool has_expected_images = content.find("\"image_count\": 8") != std::string::npos;
  const bool has_expected_coordinate_system = content.find("\"coordinate_system\": \"EPSG:4326\"") != std::string::npos;

  const bool io_ready = op::io::can_read_project_manifest("fixed-mini-dataset.json");
  const bool texturing_ready = op::texturing::can_texture_mesh();

  if (!has_schema || !has_expected_images || !has_expected_coordinate_system || !io_ready || !texturing_ready) {
    std::cerr << "dataset regression contract failed\n";
    return 1;
  }

  return 0;
}

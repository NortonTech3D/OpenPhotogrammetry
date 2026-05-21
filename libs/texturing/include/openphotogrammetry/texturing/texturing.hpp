#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "openphotogrammetry/camera/camera.hpp"
#include "openphotogrammetry/core/core.hpp"
#include "openphotogrammetry/features/features.hpp"
#include "openphotogrammetry/meshing/meshing.hpp"
#include "openphotogrammetry/sfm/sfm.hpp"

namespace op::texturing {

// ── Existing API (retained for backward compatibility) ────────────────────────

int  texture_resolution();
bool can_texture_mesh();

// ── Texture types ─────────────────────────────────────────────────────────────

struct TextureAtlas {
  int                              width{0};
  int                              height{0};
  std::vector<std::array<uint8_t, 3>> pixels;  // RGB, row-major
};

struct TexturedMesh {
  op::meshing::Mesh              geometry;
  TextureAtlas                   atlas;
  std::vector<op::core::Vec2d>   uvs;  // per-vertex UV in [0,1]²
};

// ── Texturing pipeline ────────────────────────────────────────────────────────

// Project colour from the best-viewing image onto each mesh vertex.
// For each vertex the image whose optical axis most closely aligns with the
// surface normal (and which has a valid projection) is selected, then a simple
// atlas is built by packing UV coordinates.
//
//  images     – source images (RGB)
//  intrinsics – camera intrinsics per image
//  poses      – camera poses per image
//  resolution – output atlas resolution (width = height)
TexturedMesh project_texture_onto_mesh(const op::meshing::Mesh&                          mesh,
                                       const std::vector<op::features::RgbImage>&        images,
                                       const std::vector<op::camera::CameraIntrinsics>&  intrinsics,
                                       const std::vector<op::sfm::CameraPose>&           poses,
                                       int                                                resolution = 2048);

}  // namespace op::texturing

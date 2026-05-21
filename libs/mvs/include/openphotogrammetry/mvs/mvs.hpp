#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "openphotogrammetry/core/core.hpp"
#include "openphotogrammetry/camera/camera.hpp"
#include "openphotogrammetry/sfm/sfm.hpp"
#include "openphotogrammetry/features/features.hpp"

namespace op::mvs {

// ── Existing API (retained for backward compatibility) ────────────────────────

int  depth_map_layers();
bool can_dense_reconstruct();

// ── Depth map ─────────────────────────────────────────────────────────────────

struct DepthMap {
  int                 width{0};
  int                 height{0};
  std::vector<float>  depth;       // depth[y*width+x], 0 = invalid
  std::vector<float>  confidence;  // in [0, 1], 0 = invalid
};

// ── Dense point cloud ─────────────────────────────────────────────────────────

struct DensePointCloud {
  std::vector<op::core::Vec3d>               points;
  std::vector<op::core::Vec3d>               normals;   // may be empty
  std::vector<std::array<uint8_t, 3>>        colors;    // RGB, may be empty
};

// ── Depth estimation (block-matching) ────────────────────────────────────────

// Estimate a depth map for `reference` by searching for photo-consistent
// depths along epipolar lines in `neighbor`.
//
//  window_size – NCC correlation window half-width in pixels (total = 2*w+1)
//  depth_steps – number of depth hypotheses tested along each ray
DepthMap estimate_depth_map(const op::features::GrayscaleImage&   reference,
                            const op::camera::CameraIntrinsics&   ref_intrinsics,
                            const op::sfm::CameraPose&             ref_pose,
                            const op::features::GrayscaleImage&   neighbor,
                            const op::camera::CameraIntrinsics&   nbr_intrinsics,
                            const op::sfm::CameraPose&             nbr_pose,
                            int                                    window_size = 4,
                            float                                  min_depth   = 0.1f,
                            float                                  max_depth   = 100.0f,
                            int                                    depth_steps = 64);

// Fuse multiple depth maps into a single dense point cloud.
// Pixels are kept only if they are consistent across views.
DensePointCloud fuse_depth_maps(const std::vector<DepthMap>&                        depth_maps,
                                const std::vector<op::camera::CameraIntrinsics>&    intrinsics,
                                const std::vector<op::sfm::CameraPose>&             poses,
                                float                                               min_confidence = 0.3f);

}  // namespace op::mvs

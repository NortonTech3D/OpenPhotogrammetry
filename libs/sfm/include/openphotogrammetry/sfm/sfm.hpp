#pragma once

#include <string>
#include <utility>
#include <vector>

#include "openphotogrammetry/camera/camera.hpp"
#include "openphotogrammetry/core/core.hpp"
#include "openphotogrammetry/features/features.hpp"

namespace op::sfm {

// ── Existing API (retained for backward compatibility) ────────────────────────

int  minimum_view_count();
bool can_initialize_reconstruction();

// ── Camera pose ───────────────────────────────────────────────────────────────
// R rotates from world space to camera space; t is the camera-space translation.
// Camera centre in world space: C = -R^T * t.

struct CameraPose {
  op::core::Mat3d R{};   // rotation matrix (world → camera)
  op::core::Vec3d t{};   // translation    (world → camera)
};

// Project a world point through a posed, calibrated camera.
op::core::Vec2d project_world(const op::camera::CameraIntrinsics& cam,
                               const CameraPose&                   pose,
                               op::core::Vec3d                     world_pt);

// ── Sparse reconstruction types ───────────────────────────────────────────────

struct Track {
  int             view_idx{-1};   // which image
  op::core::Vec2d observation{};  // pixel coordinates
};

struct SparsePoint {
  op::core::Vec3d    position{};
  std::vector<Track> tracks;
};

struct SparseReconstruction {
  std::vector<op::camera::CameraIntrinsics> intrinsics;  // one per image
  std::vector<CameraPose>                   poses;        // one per image
  std::vector<SparsePoint>                  points;
};

// ── Two-view geometry ─────────────────────────────────────────────────────────

// Normalised 8-point algorithm with RANSAC for the fundamental matrix.
// pts1 / pts2 must be corresponding pixel coordinates.
// threshold: Sampson error threshold (pixels) for inlier classification.
// Returns false if fewer than 8 correspondences are provided.
bool compute_fundamental_matrix(const std::vector<op::core::Vec2d>& pts1,
                                const std::vector<op::core::Vec2d>& pts2,
                                op::core::Mat3d*                     F,
                                std::vector<bool>*                   inliers   = nullptr,
                                double                               threshold = 2.0,
                                int                                  ransac_iters = 1000);

// Convert fundamental matrix to essential matrix given shared intrinsics.
// E = K2^T * F * K1
op::core::Mat3d essential_from_fundamental(op::core::Mat3d                       F,
                                           const op::camera::CameraIntrinsics&   cam1,
                                           const op::camera::CameraIntrinsics&   cam2);

// Recover relative pose from essential matrix and inlier correspondences.
// The returned pose is relative: pose of cam2 with cam1 at the origin.
bool recover_pose(const op::core::Mat3d&                E,
                  const std::vector<op::core::Vec2d>&   pts1,
                  const std::vector<op::core::Vec2d>&   pts2,
                  const op::camera::CameraIntrinsics&   cam,
                  CameraPose*                            pose);

// ── Triangulation ─────────────────────────────────────────────────────────────

// DLT triangulation: given two calibrated views of a 3-D point, find the
// world-space position.  Returns false if the point is behind either camera.
bool triangulate_point(const op::camera::CameraIntrinsics& cam1,
                       const CameraPose&                   pose1,
                       const op::camera::CameraIntrinsics& cam2,
                       const CameraPose&                   pose2,
                       op::core::Vec2d                     obs1,
                       op::core::Vec2d                     obs2,
                       op::core::Vec3d*                    out);

// Triangulate a batch of correspondences between two calibrated views.
std::vector<SparsePoint> triangulate_matches(
    const op::camera::CameraIntrinsics&         cam1,
    const CameraPose&                           pose1,
    int                                         view_idx1,
    const std::vector<op::features::Keypoint>&  kps1,
    const op::camera::CameraIntrinsics&         cam2,
    const CameraPose&                           pose2,
    int                                         view_idx2,
    const std::vector<op::features::Keypoint>&  kps2,
    const std::vector<op::features::KeypointMatch>& matches);

// ── Incremental Structure from Motion ─────────────────────────────────────────

// Run a simple incremental SfM pipeline on a set of images.
//   images      – grayscale images to process
//   intrinsics  – per-image camera intrinsics (same size as images)
//   result      – output reconstruction
//   error       – human-readable error description on failure
bool run_incremental_sfm(const std::vector<op::features::GrayscaleImage>&  images,
                         const std::vector<op::camera::CameraIntrinsics>&  intrinsics,
                         SparseReconstruction*                              result,
                         std::string*                                       error = nullptr);

}  // namespace op::sfm

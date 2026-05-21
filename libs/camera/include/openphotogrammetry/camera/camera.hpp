#pragma once

#include "openphotogrammetry/core/core.hpp"

namespace op::camera {

// ── Existing API (retained for backward compatibility) ────────────────────────

double default_focal_length_px();
bool   has_valid_default_calibration();

// ── Camera intrinsics model ───────────────────────────────────────────────────
// Pinhole camera with Brown-Conrady radial + tangential distortion.
//   fx, fy   – focal lengths in pixels
//   cx, cy   – principal point in pixels
//   k1, k2   – radial distortion coefficients (Brown-Conrady)
//   p1, p2   – tangential distortion coefficients
//   width, height – sensor dimensions in pixels

struct CameraIntrinsics {
  double fx{0}, fy{0};
  double cx{0}, cy{0};
  double k1{0}, k2{0};
  double p1{0}, p2{0};
  int    width{0}, height{0};
};

// Build a zero-distortion CameraIntrinsics from a single focal-length value,
// centering the principal point in the sensor.
CameraIntrinsics make_intrinsics(double focal_px, int width, int height);

// Build CameraIntrinsics from a horizontal field-of-view (degrees).
CameraIntrinsics make_intrinsics_from_fov(double hfov_deg, int width, int height);

// Project a 3-D world point into pixel coordinates.
// Returns {nan, nan} if the point is behind the camera (z <= 0).
op::core::Vec2d project(const CameraIntrinsics& cam, op::core::Vec3d point);

// Unproject a pixel to a unit-length ray in camera space.
op::core::Vec3d unproject_ray(const CameraIntrinsics& cam, op::core::Vec2d pixel);

// Apply Brown-Conrady distortion to a normalised (undistorted) image point.
op::core::Vec2d distort(const CameraIntrinsics& cam, op::core::Vec2d normalised);

// Remove distortion from a distorted normalised image point (iterative).
op::core::Vec2d undistort(const CameraIntrinsics& cam, op::core::Vec2d distorted, int max_iter = 20);

}  // namespace op::camera

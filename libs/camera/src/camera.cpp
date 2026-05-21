#include "openphotogrammetry/camera/camera.hpp"

#include <cmath>
#include <limits>

#include "openphotogrammetry/core/core.hpp"

namespace op::camera {

// ── Existing API ──────────────────────────────────────────────────────────────

double default_focal_length_px() {
  return 1000.0;
}

bool has_valid_default_calibration() {
  return default_focal_length_px() > 0.0 && op::core::is_runtime_ready();
}

// ── CameraIntrinsics helpers ──────────────────────────────────────────────────

CameraIntrinsics make_intrinsics(double focal_px, int width, int height) {
  CameraIntrinsics cam;
  cam.fx     = focal_px;
  cam.fy     = focal_px;
  cam.cx     = static_cast<double>(width)  * 0.5;
  cam.cy     = static_cast<double>(height) * 0.5;
  cam.width  = width;
  cam.height = height;
  return cam;
}

CameraIntrinsics make_intrinsics_from_fov(double hfov_deg, int width, int height) {
  const double hfov_rad = hfov_deg * (3.14159265358979323846 / 180.0);
  const double focal_px = static_cast<double>(width) / (2.0 * std::tan(hfov_rad * 0.5));
  return make_intrinsics(focal_px, width, height);
}

// ── Projection / unprojection ─────────────────────────────────────────────────

op::core::Vec2d project(const CameraIntrinsics& cam, op::core::Vec3d p) {
  if (p.z <= 0.0) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return {nan, nan};
  }
  // Normalise to camera plane
  const op::core::Vec2d n = {p.x / p.z, p.y / p.z};
  // Apply distortion
  const op::core::Vec2d d = distort(cam, n);
  // Convert to pixel coordinates
  return {cam.fx * d.x + cam.cx, cam.fy * d.y + cam.cy};
}

op::core::Vec3d unproject_ray(const CameraIntrinsics& cam, op::core::Vec2d pixel) {
  // Convert to normalised (distorted) coordinates
  const op::core::Vec2d nd = {(pixel.x - cam.cx) / cam.fx, (pixel.y - cam.cy) / cam.fy};
  // Remove distortion
  const op::core::Vec2d n  = undistort(cam, nd);
  return op::core::normalize(op::core::Vec3d{n.x, n.y, 1.0});
}

// ── Distortion model (Brown-Conrady) ─────────────────────────────────────────

op::core::Vec2d distort(const CameraIntrinsics& cam, op::core::Vec2d n) {
  const double r2 = n.x * n.x + n.y * n.y;
  const double r4 = r2 * r2;
  const double radial = 1.0 + cam.k1 * r2 + cam.k2 * r4;
  return {
    n.x * radial + 2.0 * cam.p1 * n.x * n.y + cam.p2 * (r2 + 2.0 * n.x * n.x),
    n.y * radial + cam.p1 * (r2 + 2.0 * n.y * n.y) + 2.0 * cam.p2 * n.x * n.y,
  };
}

op::core::Vec2d undistort(const CameraIntrinsics& cam, op::core::Vec2d d, int max_iter) {
  // Iterative inversion of distortion model
  op::core::Vec2d n = d;  // start with distorted as initial guess
  for (int i = 0; i < max_iter; ++i) {
    const op::core::Vec2d residual = d - distort(cam, n);
    n = n + residual;
    if (op::core::norm(residual) < 1e-12) {
      break;
    }
  }
  return n;
}

}  // namespace op::camera

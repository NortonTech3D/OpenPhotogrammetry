#include "openphotogrammetry/mvs/mvs.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include "openphotogrammetry/sfm/sfm.hpp"

namespace op::mvs {

// ── Existing API ──────────────────────────────────────────────────────────────

int depth_map_layers() {
  return op::sfm::can_initialize_reconstruction() ? 4 : 0;
}

bool can_dense_reconstruct() {
  return depth_map_layers() >= 1;
}

// ── Depth map estimation (NCC block matching) ─────────────────────────────────

namespace {

// Bilinear sample of a grayscale image at floating-point (u, v).
// Returns -1 if out of bounds.
float bilinear_sample(const op::features::GrayscaleImage& img, float u, float v) {
  if (u < 0 || v < 0 || u >= static_cast<float>(img.width - 1)
      || v >= static_cast<float>(img.height - 1)) {
    return -1.0f;
  }
  const int   x0  = static_cast<int>(u);
  const int   y0  = static_cast<int>(v);
  const float fu  = u - static_cast<float>(x0);
  const float fv  = v - static_cast<float>(y0);
  const auto  px  = [&](int x, int y) {
    return static_cast<float>(img.pixels[static_cast<std::size_t>(y) * img.width + x]);
  };
  return (1.0f - fu) * (1.0f - fv) * px(x0,     y0)
       + fu           * (1.0f - fv) * px(x0 + 1, y0)
       + (1.0f - fu)  * fv           * px(x0,     y0 + 1)
       + fu            * fv           * px(x0 + 1, y0 + 1);
}

// Normalised cross-correlation between reference patch (pre-extracted) and a
// window centred at (u, v) in `other`.
// Returns NCC in [-1, 1] or -2 on failure.
float ncc_patch(const std::vector<float>& ref_patch, int half_w,
                const op::features::GrayscaleImage& other, float u, float v) {
  const int size = (2 * half_w + 1) * (2 * half_w + 1);
  float sum_o = 0.0f;
  float sum_o2 = 0.0f;
  std::vector<float> other_patch(static_cast<std::size_t>(size));
  for (int dy = -half_w; dy <= half_w; ++dy) {
    for (int dx = -half_w; dx <= half_w; ++dx) {
      const float s = bilinear_sample(other, u + static_cast<float>(dx),
                                            v + static_cast<float>(dy));
      if (s < 0.0f) {
        return -2.0f;
      }
      const int idx = (dy + half_w) * (2 * half_w + 1) + (dx + half_w);
      other_patch[static_cast<std::size_t>(idx)] = s;
      sum_o  += s;
      sum_o2 += s * s;
    }
  }
  const float mean_o  = sum_o  / static_cast<float>(size);
  const float var_o   = sum_o2 / static_cast<float>(size) - mean_o * mean_o;

  // Also compute mean/variance of reference patch
  float sum_r = 0.0f;
  float sum_r2 = 0.0f;
  for (float v2 : ref_patch) {
    sum_r  += v2;
    sum_r2 += v2 * v2;
  }
  const float mean_r = sum_r  / static_cast<float>(size);
  const float var_r  = sum_r2 / static_cast<float>(size) - mean_r * mean_r;

  if (var_r < 1e-4f || var_o < 1e-4f) {
    return -2.0f;  // constant patch
  }
  const float sigma_r = std::sqrt(var_r);
  const float sigma_o = std::sqrt(var_o);

  float cross = 0.0f;
  for (int i = 0; i < size; ++i) {
    cross += (ref_patch[static_cast<std::size_t>(i)] - mean_r) * (other_patch[static_cast<std::size_t>(i)] - mean_o);
  }
  return cross / (static_cast<float>(size) * sigma_r * sigma_o);
}

}  // namespace

DepthMap estimate_depth_map(const op::features::GrayscaleImage&   reference,
                            const op::camera::CameraIntrinsics&   ref_intrinsics,
                            const op::sfm::CameraPose&             ref_pose,
                            const op::features::GrayscaleImage&   neighbor,
                            const op::camera::CameraIntrinsics&   nbr_intrinsics,
                            const op::sfm::CameraPose&             nbr_pose,
                            int                                    half_w,
                            float                                  min_depth,
                            float                                  max_depth,
                            int                                    depth_steps) {
  const int W = reference.width;
  const int H = reference.height;

  DepthMap dmap;
  dmap.width      = W;
  dmap.height     = H;
  dmap.depth.assign(static_cast<std::size_t>(W) * H, 0.0f);
  dmap.confidence.assign(static_cast<std::size_t>(W) * H, 0.0f);

  const int patch_size = (2 * half_w + 1) * (2 * half_w + 1);

  for (int y = half_w; y < H - half_w; ++y) {
    for (int x = half_w; x < W - half_w; ++x) {
      // Extract reference patch
      std::vector<float> ref_patch(static_cast<std::size_t>(patch_size));
      for (int dy = -half_w; dy <= half_w; ++dy) {
        for (int dx = -half_w; dx <= half_w; ++dx) {
          const int ri = (dy + half_w) * (2 * half_w + 1) + (dx + half_w);
          ref_patch[static_cast<std::size_t>(ri)] =
            static_cast<float>(reference.pixels[static_cast<std::size_t>(y + dy) * W + (x + dx)]);
        }
      }

      // Unproject reference ray
      const op::core::Vec3d ref_ray = op::camera::unproject_ray(ref_intrinsics, {static_cast<double>(x), static_cast<double>(y)});

      // Camera centre in world space: C = -R^T * t
      const op::core::Mat3d Rt  = op::core::mat3_transpose(ref_pose.R);
      const op::core::Vec3d ref_centre = op::core::mat3_mul_vec(Rt, op::core::Vec3d{} - ref_pose.t);

      // Ray direction in world space: R^T * d
      const op::core::Vec3d ray_world = op::core::normalize(op::core::mat3_mul_vec(Rt, ref_ray));

      float best_ncc   = -1.5f;
      float best_depth = 0.0f;

      for (int di = 0; di < depth_steps; ++di) {
        const float depth = min_depth
          + static_cast<float>(di) / static_cast<float>(depth_steps - 1)
            * (max_depth - min_depth);

        // 3-D point at this depth hypothesis
        const op::core::Vec3d pt = ref_centre + ray_world * static_cast<double>(depth);

        // Project into neighbor camera
        const op::core::Vec3d cam_pt = op::core::mat3_mul_vec(nbr_pose.R, pt) + nbr_pose.t;
        const op::core::Vec2d nbr_px = op::camera::project(nbr_intrinsics, cam_pt);

        if (std::isnan(nbr_px.x) || std::isnan(nbr_px.y)) {
          continue;
        }

        const float ncc = ncc_patch(ref_patch, half_w, neighbor,
                                    static_cast<float>(nbr_px.x),
                                    static_cast<float>(nbr_px.y));
        if (ncc > best_ncc) {
          best_ncc   = ncc;
          best_depth = depth;
        }
      }

      const std::size_t idx = static_cast<std::size_t>(y) * W + x;
      if (best_ncc > 0.5f) {
        dmap.depth[idx]      = best_depth;
        dmap.confidence[idx] = (best_ncc + 1.0f) * 0.5f;
      }
    }
  }
  return dmap;
}

// ── Depth map fusion ──────────────────────────────────────────────────────────

DensePointCloud fuse_depth_maps(const std::vector<DepthMap>&                     depth_maps,
                                const std::vector<op::camera::CameraIntrinsics>& intrinsics,
                                const std::vector<op::sfm::CameraPose>&          poses,
                                float                                            min_confidence) {
  DensePointCloud cloud;

  const std::size_t n_views = depth_maps.size();
  if (intrinsics.size() != n_views || poses.size() != n_views) {
    return cloud;
  }

  for (std::size_t vi = 0; vi < n_views; ++vi) {
    const DepthMap&                       dmap = depth_maps[vi];
    const op::camera::CameraIntrinsics&   cam  = intrinsics[vi];
    const op::sfm::CameraPose&            pose = poses[vi];
    const op::core::Mat3d                 Rt   = op::core::mat3_transpose(pose.R);

    for (int y = 0; y < dmap.height; ++y) {
      for (int x = 0; x < dmap.width; ++x) {
        const std::size_t idx = static_cast<std::size_t>(y) * dmap.width + x;
        if (dmap.confidence[idx] < min_confidence || dmap.depth[idx] <= 0.0f) {
          continue;
        }

        // Unproject to world
        const op::core::Vec3d ray_cam = op::camera::unproject_ray(cam, {static_cast<double>(x), static_cast<double>(y)});
        const op::core::Vec3d pt_cam  = ray_cam * static_cast<double>(dmap.depth[idx]);
        // World point: R^T * (pt_cam - t)
        const op::core::Vec3d pt_world = op::core::mat3_mul_vec(Rt, pt_cam - pose.t);

        cloud.points.push_back(pt_world);
      }
    }
  }
  return cloud;
}

}  // namespace op::mvs

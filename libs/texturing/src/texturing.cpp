#include "openphotogrammetry/texturing/texturing.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include "openphotogrammetry/io/io.hpp"
#include "openphotogrammetry/meshing/meshing.hpp"

namespace op::texturing {

// ── Existing API ──────────────────────────────────────────────────────────────

int texture_resolution() {
  if (!op::meshing::can_generate_mesh()) {
    return 0;
  }

  return op::io::can_read_project_manifest("dataset.json") ? 2048 : 0;
}

bool can_texture_mesh() {
  return texture_resolution() >= 1024;
}

// ── project_texture_onto_mesh ─────────────────────────────────────────────────

TexturedMesh project_texture_onto_mesh(const op::meshing::Mesh&                         mesh,
                                       const std::vector<op::features::RgbImage>&        images,
                                       const std::vector<op::camera::CameraIntrinsics>&  intrinsics,
                                       const std::vector<op::sfm::CameraPose>&           poses,
                                       int                                                resolution) {
  TexturedMesh result;
  result.geometry = mesh;

  const std::size_t n_verts  = mesh.vertices.size();
  const std::size_t n_images = images.size();

  result.uvs.resize(n_verts, {0.0, 0.0});

  // Atlas: flat 2-D grid of UV patches — we use a simple row-by-row packing
  // where each vertex gets a unique 1×1 texel (for a per-vertex scheme).
  // For a production implementation, per-face charts with proper seams would
  // be needed; this baseline is intentionally simple.

  result.atlas.width  = resolution;
  result.atlas.height = resolution;
  result.atlas.pixels.assign(static_cast<std::size_t>(resolution) * resolution,
                              {128, 128, 128});  // default grey

  if (n_images == 0 || n_verts == 0) {
    return result;
  }

  // For each vertex: find the image with the best viewing angle (most-frontal)
  // that successfully projects the vertex inside the image bounds.
  for (std::size_t vi = 0; vi < n_verts; ++vi) {
    const op::core::Vec3d& vertex = mesh.vertices[vi];
    const op::core::Vec3d  vn     = (mesh.normals.size() == n_verts)
                                      ? mesh.normals[vi]
                                      : op::core::Vec3d{0, 0, 1};

    int    best_img   = -1;
    double best_score = -std::numeric_limits<double>::max();
    op::core::Vec2d best_px;

    for (std::size_t ii = 0; ii < n_images; ++ii) {
      const op::sfm::CameraPose& pose = poses[ii];
      // Camera centre in world: C = -R^T * t
      const op::core::Mat3d Rt     = op::core::mat3_transpose(pose.R);
      const op::core::Vec3d centre = op::core::mat3_mul_vec(Rt, op::core::Vec3d{} - pose.t);
      // View direction (towards vertex)
      const op::core::Vec3d view_dir = op::core::normalize(vertex - centre);

      // Score: negative dot of view direction with vertex normal (more frontal = higher)
      const double score = -op::core::dot(view_dir, vn);
      if (score <= 0.0) {
        continue;  // vertex facing away from this camera
      }

      // Project vertex into image
      const op::core::Vec3d cam_pt = op::core::mat3_mul_vec(pose.R, vertex) + pose.t;
      const op::core::Vec2d px     = op::camera::project(intrinsics[ii], cam_pt);

      if (std::isnan(px.x) || std::isnan(px.y)) {
        continue;
      }
      const op::camera::CameraIntrinsics& cam = intrinsics[ii];
      if (px.x < 0 || px.y < 0
          || px.x >= static_cast<double>(cam.width)
          || px.y >= static_cast<double>(cam.height)) {
        continue;
      }

      if (score > best_score) {
        best_score = score;
        best_img   = static_cast<int>(ii);
        best_px    = px;
      }
    }

    // Map vertex index to a UV coordinate (simple linear packing across atlas)
    const double u = (static_cast<double>(vi) + 0.5) / static_cast<double>(n_verts);
    const double v = 0.5;  // single-row strip
    result.uvs[vi] = {u, v};

    // Sample colour from the best-viewing image
    if (best_img >= 0) {
      const op::features::RgbImage& img = images[static_cast<std::size_t>(best_img)];
      const int sx = std::clamp(static_cast<int>(best_px.x), 0, img.width  - 1);
      const int sy = std::clamp(static_cast<int>(best_px.y), 0, img.height - 1);
      const std::array<uint8_t, 3> colour = {
        img.pixels[static_cast<std::size_t>(sy) * img.width * 3 + sx * 3],
        img.pixels[static_cast<std::size_t>(sy) * img.width * 3 + sx * 3 + 1],
        img.pixels[static_cast<std::size_t>(sy) * img.width * 3 + sx * 3 + 2],
      };

      // Write colour into atlas at the UV position
      const int atlas_x = std::clamp(static_cast<int>(u * resolution), 0, resolution - 1);
      const int atlas_y = std::clamp(static_cast<int>(v * resolution), 0, resolution - 1);
      result.atlas.pixels[static_cast<std::size_t>(atlas_y) * resolution + atlas_x] = colour;
    }
  }

  return result;
}

}  // namespace op::texturing

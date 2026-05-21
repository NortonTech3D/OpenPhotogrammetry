#pragma once

#include <string>
#include <vector>

#include "openphotogrammetry/core/core.hpp"
#include "openphotogrammetry/mvs/mvs.hpp"

namespace op::meshing {

// ── Existing API (retained for backward compatibility) ────────────────────────

int  target_face_budget();
bool can_generate_mesh();

// ── Mesh representation ───────────────────────────────────────────────────────

struct Mesh {
  std::vector<op::core::Vec3d>          vertices;
  std::vector<op::core::Vec3d>          normals;   // per-vertex normals (same size as vertices)
  std::vector<std::array<int, 3>>       faces;     // CCW winding; indices into vertices
};

// ── Surface reconstruction ────────────────────────────────────────────────────

// Reconstruct a surface mesh from a dense point cloud using 2.5D Delaunay
// triangulation.  Points are projected onto the plane of best fit and
// triangulated there; the resulting triangles are lifted back to 3-D.
Mesh triangulate_point_cloud(const op::mvs::DensePointCloud& cloud,
                             float                           max_edge_length = 0.0f);

// Estimate per-vertex normals from the mesh face normals.
void estimate_vertex_normals(Mesh& mesh);

// ── Output ────────────────────────────────────────────────────────────────────

// Write mesh to a PLY (ASCII) file.
bool save_mesh_ply(std::string_view path, const Mesh& mesh, std::string* error = nullptr);

// Write mesh to a Wavefront OBJ file.
bool save_mesh_obj(std::string_view path, const Mesh& mesh, std::string* error = nullptr);

}  // namespace op::meshing

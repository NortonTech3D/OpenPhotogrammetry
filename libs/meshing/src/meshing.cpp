#include "openphotogrammetry/meshing/meshing.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <unordered_map>
#include <vector>

#include "openphotogrammetry/mvs/mvs.hpp"

namespace op::meshing {

// ── Existing API ──────────────────────────────────────────────────────────────

int target_face_budget() {
  return op::mvs::can_dense_reconstruct() ? 50000 : 0;
}

bool can_generate_mesh() {
  return target_face_budget() > 0;
}

// ── PCA helpers for best-fit plane ───────────────────────────────────────────

namespace {

op::core::Vec3d centroid(const std::vector<op::core::Vec3d>& pts) {
  op::core::Vec3d c;
  for (const auto& p : pts) {
    c.x += p.x;
    c.y += p.y;
    c.z += p.z;
  }
  const double n = static_cast<double>(pts.size());
  c.x /= n;  c.y /= n;  c.z /= n;
  return c;
}

// Find the normal of the best-fit plane via PCA (smallest eigenvector of
// covariance matrix).  Returns the normal and sets u, v to two tangent axes.
op::core::Vec3d best_fit_plane(const std::vector<op::core::Vec3d>& pts,
                               op::core::Vec3d*                    u_out,
                               op::core::Vec3d*                    v_out) {
  const op::core::Vec3d c = centroid(pts);

  // 3×3 covariance matrix (symmetric, stored row-major)
  double cov[9] = {};
  for (const auto& p : pts) {
    const double dx = p.x - c.x;
    const double dy = p.y - c.y;
    const double dz = p.z - c.z;
    cov[0] += dx * dx;  cov[1] += dx * dy;  cov[2] += dx * dz;
    cov[3] += dx * dy;  cov[4] += dy * dy;  cov[5] += dy * dz;
    cov[6] += dx * dz;  cov[7] += dy * dz;  cov[8] += dz * dz;
  }

  op::core::Mat3d C;
  for (int i = 0; i < 9; ++i) {
    C.d[i] = cov[i] / static_cast<double>(pts.size());
  }

  op::core::Mat3d U;
  op::core::Vec3d S;
  op::core::Mat3d Vt;
  op::core::mat3_svd(C, &U, &S, &Vt);

  // Normal = last column of V = last row of Vt
  const op::core::Vec3d normal = op::core::normalize({Vt(2, 0), Vt(2, 1), Vt(2, 2)});
  // Two tangent axes: first two rows of Vt
  if (u_out) *u_out = {Vt(0, 0), Vt(0, 1), Vt(0, 2)};
  if (v_out) *v_out = {Vt(1, 0), Vt(1, 1), Vt(1, 2)};
  return normal;
}

// Project a 3-D point onto 2-D tangent coordinates (u, v).
op::core::Vec2d project_2d(op::core::Vec3d pt, op::core::Vec3d c,
                            op::core::Vec3d u, op::core::Vec3d v) {
  const op::core::Vec3d d = pt - c;
  return {op::core::dot(d, u), op::core::dot(d, v)};
}

// ── Bowyer-Watson Delaunay triangulation (2-D) ───────────────────────────────

struct Triangle2D {
  int a{0}, b{0}, c{0};  // indices into point set
};

struct CircumCircle {
  double cx{0}, cy{0}, r2{0};
};

CircumCircle circumcircle(const std::vector<op::core::Vec2d>& pts, const Triangle2D& tri) {
  const double ax = pts[tri.a].x;  const double ay = pts[tri.a].y;
  const double bx = pts[tri.b].x;  const double by = pts[tri.b].y;
  const double cx = pts[tri.c].x;  const double cy = pts[tri.c].y;

  const double D = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
  if (std::abs(D) < 1e-14) {
    return {0, 0, std::numeric_limits<double>::max()};
  }
  const double ux = ((ax * ax + ay * ay) * (by - cy)
                   + (bx * bx + by * by) * (cy - ay)
                   + (cx * cx + cy * cy) * (ay - by)) / D;
  const double uy = ((ax * ax + ay * ay) * (cx - bx)
                   + (bx * bx + by * by) * (ax - cx)
                   + (cx * cx + cy * cy) * (bx - ax)) / D;
  const double dx = ax - ux;
  const double dy = ay - uy;
  return {ux, uy, dx * dx + dy * dy};
}

bool in_circumcircle(const CircumCircle& cc, op::core::Vec2d p) {
  const double dx = p.x - cc.cx;
  const double dy = p.y - cc.cy;
  return dx * dx + dy * dy < cc.r2 * (1.0 + 1e-10);
}

std::vector<Triangle2D> bowyer_watson(const std::vector<op::core::Vec2d>& points) {
  if (points.size() < 3) {
    return {};
  }

  // Find bounding box and create super-triangle
  double min_x = points[0].x;  double max_x = min_x;
  double min_y = points[0].y;  double max_y = min_y;
  for (const auto& p : points) {
    min_x = std::min(min_x, p.x);  max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);  max_y = std::max(max_y, p.y);
  }
  const double dx = (max_x - min_x) * 3.0 + 10.0;
  const double dy = (max_y - min_y) * 3.0 + 10.0;

  // Augmented point list with 3 super-triangle vertices appended
  std::vector<op::core::Vec2d> pts(points);
  const int si_a = static_cast<int>(pts.size());  pts.push_back({min_x - dx,           min_y - dy});
  const int si_b = static_cast<int>(pts.size());  pts.push_back({min_x + dx * 2.0,     min_y - dy});
  const int si_c = static_cast<int>(pts.size());  pts.push_back({min_x + (max_x - min_x) * 0.5, max_y + dy});

  std::vector<Triangle2D> triangles;
  triangles.push_back({si_a, si_b, si_c});

  for (int pi = 0; pi < static_cast<int>(points.size()); ++pi) {
    const op::core::Vec2d p = points[static_cast<std::size_t>(pi)];

    // Find all triangles whose circumcircle contains p
    std::vector<std::array<int, 2>> boundary_edges;
    std::vector<Triangle2D> new_triangles;

    for (const auto& tri : triangles) {
      const CircumCircle cc = circumcircle(pts, tri);
      if (in_circumcircle(cc, p)) {
        // Add edges of bad triangle to boundary
        boundary_edges.push_back({tri.a, tri.b});
        boundary_edges.push_back({tri.b, tri.c});
        boundary_edges.push_back({tri.c, tri.a});
      }
      else {
        new_triangles.push_back(tri);
      }
    }

    // Remove duplicate boundary edges (shared edges are internal)
    std::vector<std::array<int, 2>> unique_edges;
    for (const auto& e : boundary_edges) {
      bool shared = false;
      for (const auto& other : boundary_edges) {
        if (&e == &other) {
          continue;
        }
        if ((e[0] == other[1] && e[1] == other[0])
            || (e[0] == other[0] && e[1] == other[1])) {
          shared = true;
          break;
        }
      }
      if (!shared) {
        unique_edges.push_back(e);
      }
    }

    for (const auto& e : unique_edges) {
      new_triangles.push_back({pi, e[0], e[1]});
    }
    triangles = std::move(new_triangles);
  }

  // Remove triangles that share a vertex with the super-triangle
  std::vector<Triangle2D> result;
  for (const auto& tri : triangles) {
    if (tri.a == si_a || tri.a == si_b || tri.a == si_c
        || tri.b == si_a || tri.b == si_b || tri.b == si_c
        || tri.c == si_a || tri.c == si_b || tri.c == si_c) {
      continue;
    }
    result.push_back(tri);
  }
  return result;
}

}  // namespace

// ── triangulate_point_cloud ───────────────────────────────────────────────────

Mesh triangulate_point_cloud(const op::mvs::DensePointCloud& cloud, float max_edge_length) {
  Mesh mesh;
  if (cloud.points.size() < 3) {
    return mesh;
  }

  const std::size_t N = cloud.points.size();
  const std::vector<op::core::Vec3d>& pts3 = cloud.points;

  // Find best-fit plane and project points
  op::core::Vec3d u_axis;
  op::core::Vec3d v_axis;
  const op::core::Vec3d c3  = centroid(pts3);
  const op::core::Vec3d norm = best_fit_plane(pts3, &u_axis, &v_axis);
  (void)norm;  // used implicitly via axes

  std::vector<op::core::Vec2d> pts2(N);
  for (std::size_t i = 0; i < N; ++i) {
    pts2[i] = project_2d(pts3[i], c3, u_axis, v_axis);
  }

  // Delaunay triangulation in 2-D
  const std::vector<Triangle2D> tris = bowyer_watson(pts2);

  mesh.vertices.reserve(N);
  for (std::size_t i = 0; i < N; ++i) {
    mesh.vertices.push_back(pts3[i]);
  }

  // Add faces, optionally filtering by max edge length
  const double max_e2 = (max_edge_length > 0.0f)
                          ? static_cast<double>(max_edge_length * max_edge_length)
                          : std::numeric_limits<double>::max();

  for (const auto& tri : tris) {
    const op::core::Vec3d& va = pts3[static_cast<std::size_t>(tri.a)];
    const op::core::Vec3d& vb = pts3[static_cast<std::size_t>(tri.b)];
    const op::core::Vec3d& vc = pts3[static_cast<std::size_t>(tri.c)];

    const auto edge_len2 = [](op::core::Vec3d p, op::core::Vec3d q) {
      const op::core::Vec3d d = p - q;
      return op::core::dot(d, d);
    };

    if (edge_len2(va, vb) > max_e2
        || edge_len2(vb, vc) > max_e2
        || edge_len2(vc, va) > max_e2) {
      continue;
    }

    mesh.faces.push_back({tri.a, tri.b, tri.c});
  }

  estimate_vertex_normals(mesh);
  return mesh;
}

// ── estimate_vertex_normals ───────────────────────────────────────────────────

void estimate_vertex_normals(Mesh& mesh) {
  mesh.normals.assign(mesh.vertices.size(), {0.0, 0.0, 0.0});

  for (const auto& face : mesh.faces) {
    const op::core::Vec3d& va = mesh.vertices[static_cast<std::size_t>(face[0])];
    const op::core::Vec3d& vb = mesh.vertices[static_cast<std::size_t>(face[1])];
    const op::core::Vec3d& vc = mesh.vertices[static_cast<std::size_t>(face[2])];
    const op::core::Vec3d fn  = op::core::cross(vb - va, vc - va);
    auto add = [](op::core::Vec3d& a, op::core::Vec3d b) {
      a.x += b.x;  a.y += b.y;  a.z += b.z;
    };
    add(mesh.normals[static_cast<std::size_t>(face[0])], fn);
    add(mesh.normals[static_cast<std::size_t>(face[1])], fn);
    add(mesh.normals[static_cast<std::size_t>(face[2])], fn);
  }
  for (auto& n : mesh.normals) {
    n = op::core::normalize(n);
  }
}

// ── save_mesh_ply ─────────────────────────────────────────────────────────────

bool save_mesh_ply(std::string_view path, const Mesh& mesh, std::string* error) {
  const std::string path_str{path};
  std::ofstream f{path_str};
  if (!f) {
    if (error) *error = "cannot open file: " + path_str;
    return false;
  }
  const bool has_normals = mesh.normals.size() == mesh.vertices.size();
  f << "ply\nformat ascii 1.0\n";
  f << "element vertex " << mesh.vertices.size() << '\n';
  f << "property float x\nproperty float y\nproperty float z\n";
  if (has_normals) {
    f << "property float nx\nproperty float ny\nproperty float nz\n";
  }
  f << "element face " << mesh.faces.size() << '\n';
  f << "property list uchar int vertex_index\nend_header\n";
  for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
    f << mesh.vertices[i].x << ' ' << mesh.vertices[i].y << ' ' << mesh.vertices[i].z;
    if (has_normals) {
      f << ' ' << mesh.normals[i].x << ' ' << mesh.normals[i].y << ' ' << mesh.normals[i].z;
    }
    f << '\n';
  }
  for (const auto& face : mesh.faces) {
    f << "3 " << face[0] << ' ' << face[1] << ' ' << face[2] << '\n';
  }
  if (!f) {
    if (error) *error = "write error";
    return false;
  }
  return true;
}

// ── save_mesh_obj ─────────────────────────────────────────────────────────────

bool save_mesh_obj(std::string_view path, const Mesh& mesh, std::string* error) {
  const std::string path_str{path};
  std::ofstream f{path_str};
  if (!f) {
    if (error) *error = "cannot open file: " + path_str;
    return false;
  }
  for (const auto& v : mesh.vertices) {
    f << "v " << v.x << ' ' << v.y << ' ' << v.z << '\n';
  }
  if (mesh.normals.size() == mesh.vertices.size()) {
    for (const auto& n : mesh.normals) {
      f << "vn " << n.x << ' ' << n.y << ' ' << n.z << '\n';
    }
  }
  for (const auto& face : mesh.faces) {
    // OBJ is 1-indexed
    f << "f " << (face[0] + 1) << ' ' << (face[1] + 1) << ' ' << (face[2] + 1) << '\n';
  }
  if (!f) {
    if (error) *error = "write error";
    return false;
  }
  return true;
}

}  // namespace op::meshing

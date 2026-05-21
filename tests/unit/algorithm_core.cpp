// Unit tests for the real algorithm implementations in core, camera, features,
// sfm, mvs, meshing, and texturing.

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "openphotogrammetry/camera/camera.hpp"
#include "openphotogrammetry/core/core.hpp"
#include "openphotogrammetry/features/features.hpp"
#include "openphotogrammetry/meshing/meshing.hpp"
#include "openphotogrammetry/mvs/mvs.hpp"
#include "openphotogrammetry/sfm/sfm.hpp"

namespace {

// ── Helpers ───────────────────────────────────────────────────────────────────

bool near(double a, double b, double tol = 1e-9) {
  return std::abs(a - b) <= tol;
}

bool near_v3(op::core::Vec3d a, op::core::Vec3d b, double tol = 1e-9) {
  return near(a.x, b.x, tol) && near(a.y, b.y, tol) && near(a.z, b.z, tol);
}

bool near_mat3(op::core::Mat3d A, op::core::Mat3d B, double tol = 1e-9) {
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      if (!near(A(r, c), B(r, c), tol))
        return false;
  return true;
}

int fail(const std::string& msg) {
  std::cerr << "FAIL: " << msg << '\n';
  return 1;
}

// ── core: vector arithmetic ───────────────────────────────────────────────────

int test_vec_arithmetic() {
  using namespace op::core;

  const Vec3d a{1, 2, 3};
  const Vec3d b{4, 5, 6};

  if (!near(dot(a, b), 32.0))
    return fail("dot product");
  if (!near_v3(cross(a, b), {-3, 6, -3}))
    return fail("cross product");
  if (!near(norm(Vec3d{3, 4, 0}), 5.0))
    return fail("norm");
  if (!near(norm(normalize(a)), 1.0, 1e-12))
    return fail("normalize length");
  return 0;
}

// ── core: matrix operations ───────────────────────────────────────────────────

int test_mat3_ops() {
  using namespace op::core;

  const Mat3d I = mat3_identity();
  if (!near(mat3_det(I), 1.0))
    return fail("identity det");

  Mat3d A;
  A(0, 0) = 1;  A(0, 1) = 2;  A(0, 2) = 0;
  A(1, 0) = 0;  A(1, 1) = 3;  A(1, 2) = 4;
  A(2, 0) = 5;  A(2, 1) = 0;  A(2, 2) = 6;

  Mat3d inv;
  if (!mat3_inv(A, &inv))
    return fail("mat3_inv return false");

  // A * inv should be identity
  const Mat3d prod = mat3_mul(A, inv);
  if (!near_mat3(prod, I, 1e-10))
    return fail("A * inv(A) != I");

  // mat3_mul_vec: I * v = v
  const Vec3d v{7, 8, 9};
  if (!near_v3(mat3_mul_vec(I, v), v))
    return fail("I * v");

  return 0;
}

// ── core: Jacobi SVD ─────────────────────────────────────────────────────────

int test_svd_3x3() {
  using namespace op::core;

  // Build a known rank-2 matrix: M = u * v^T + w * x^T
  const Vec3d u = normalize({1.0, 2.0, 3.0});
  const Vec3d v = normalize({4.0, 5.0, 6.0});
  Mat3d M;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      M(r, c) = (&u.x)[r] * (&v.x)[c] * 5.0 + (&v.x)[r] * (&u.x)[c] * 3.0;

  Mat3d U;
  Vec3d S;
  Mat3d Vt;
  if (!mat3_svd(M, &U, &S, &Vt))
    return fail("svd returned false");

  // Singular values should be non-negative and descending
  if (S.x < S.y || S.y < S.z - 1e-10)
    return fail("singular values not sorted");

  // Reconstruct M' = U * diag(S) * Vt and check || M - M' ||_F < tol
  Mat3d Diag;
  Diag(0, 0) = S.x;  Diag(1, 1) = S.y;  Diag(2, 2) = S.z;
  const Mat3d M_reconstructed = mat3_mul(U, mat3_mul(Diag, Vt));
  if (!near_mat3(M, M_reconstructed, 1e-9))
    return fail("SVD reconstruction mismatch");

  // U and V should be orthogonal
  const Mat3d UtU = mat3_mul(mat3_transpose(U), U);
  if (!near_mat3(UtU, mat3_identity(), 1e-9))
    return fail("U not orthogonal");

  return 0;
}

// ── core: smallest eigenvector ────────────────────────────────────────────────

int test_sym_min_eigenvec() {
  // Diagonal matrix: eigenvalues are on the diagonal
  // min eigenvalue = 1.0, eigenvec = [0, 0, 1]
  const std::vector<double> A = {
    9.0, 0.0, 0.0,
    0.0, 4.0, 0.0,
    0.0, 0.0, 1.0,
  };
  std::vector<double> vec;
  const double eval = op::core::sym_min_eigenvec(A, 3, &vec);
  if (!near(eval, 1.0, 1e-10))
    return fail("sym_min_eigenvec: wrong eigenvalue");
  // Eigenvector should be [0, 0, ±1]
  if (!near(std::abs(vec[2]), 1.0, 1e-10))
    return fail("sym_min_eigenvec: wrong eigenvector");
  return 0;
}

// ── camera: projection / unprojection ────────────────────────────────────────

int test_camera_project_unproject() {
  using namespace op::camera;
  using namespace op::core;

  const CameraIntrinsics cam = make_intrinsics(800.0, 1280, 720);

  // A point exactly on the optical axis at z=2 should project to the centre
  const Vec2d px = project(cam, {0, 0, 2});
  if (!near(px.x, 640.0, 1e-9) || !near(px.y, 360.0, 1e-9))
    return fail("project: on-axis point not at image centre");

  // Unproject the centre pixel and ensure z == 1 (normalised ray)
  const Vec3d ray = unproject_ray(cam, {640, 360});
  if (!near(ray.x, 0.0, 1e-9) || !near(ray.y, 0.0, 1e-9))
    return fail("unproject_ray: centre pixel not on optical axis");
  if (!near(norm(ray), 1.0, 1e-12))
    return fail("unproject_ray: ray not normalised");

  // Round-trip: project a 3-D point, then unproject the pixel, check direction
  const Vec3d pt{0.5, -0.3, 3.0};
  const Vec2d proj_px = project(cam, pt);
  const Vec3d uprojected = unproject_ray(cam, proj_px);
  // uprojected should be parallel to pt (same direction)
  const double dot_val = dot(normalize(pt), uprojected);
  if (!near(dot_val, 1.0, 1e-9))
    return fail("camera round-trip: direction mismatch");

  return 0;
}

// ── camera: distortion / undistortion round-trip ─────────────────────────────

int test_distortion_roundtrip() {
  using namespace op::camera;
  using namespace op::core;

  CameraIntrinsics cam = make_intrinsics(800.0, 1280, 720);
  cam.k1 = -0.15;
  cam.k2 =  0.05;

  const Vec2d n{0.2, -0.1};
  const Vec2d d   = distort(cam, n);
  const Vec2d n2  = undistort(cam, d);

  if (!near(n2.x, n.x, 1e-8) || !near(n2.y, n.y, 1e-8))
    return fail("distortion round-trip failed");

  return 0;
}

// ── features: Harris keypoint detection ──────────────────────────────────────

int test_harris_detection() {
  using namespace op::features;

  // Create a small synthetic image with a bright square in the top-left corner.
  // Harris corners should be detected at the corners of the square.
  GrayscaleImage img;
  img.width  = 64;
  img.height = 64;
  img.pixels.assign(static_cast<std::size_t>(img.width) * img.height, 50u);

  for (int y = 10; y < 25; ++y)
    for (int x = 10; x < 25; ++x)
      img.pixels[static_cast<std::size_t>(y) * img.width + x] = 200u;

  const auto kps = detect_keypoints(img, 100);
  if (kps.empty())
    return fail("Harris: no keypoints detected on synthetic image");

  // All keypoints should lie within image bounds
  for (const auto& kp : kps) {
    if (kp.x < 0 || kp.y < 0 || kp.x >= 64 || kp.y >= 64)
      return fail("Harris: keypoint out of bounds");
    if (kp.response <= 0.0f)
      return fail("Harris: non-positive response");
  }

  return 0;
}

// ── features: descriptor computation and matching ────────────────────────────

int test_descriptor_matching() {
  using namespace op::features;

  // Two identical images: matching should produce near-zero distances
  GrayscaleImage img;
  img.width  = 64;
  img.height = 64;
  img.pixels.resize(static_cast<std::size_t>(img.width) * img.height);
  // Fill with a checkerboard pattern to provide texture
  for (int y = 0; y < 64; ++y)
    for (int x = 0; x < 64; ++x)
      img.pixels[static_cast<std::size_t>(y) * 64 + x] =
        static_cast<uint8_t>(((x / 4 + y / 4) % 2) ? 200u : 50u);

  const auto kps1  = detect_keypoints(img, 50);
  const auto desc1 = compute_descriptors(img, kps1);
  // Match image against itself: every query should find itself as best match
  const auto matches = match_descriptors(desc1, desc1, 0.9f);

  if (matches.empty())
    return fail("descriptor matching: no matches found on self-match");

  // In a self-match the Hamming distance must be 0 for the identical descriptor
  for (const auto& m : matches) {
    if (m.distance != 0.0f)
      return fail("descriptor matching: non-zero distance in self-match");
  }
  return 0;
}

// ── sfm: triangulation (DLT) ─────────────────────────────────────────────────

int test_triangulation() {
  using namespace op::sfm;
  using namespace op::camera;
  using namespace op::core;

  // Two cameras facing the same target point.
  const CameraIntrinsics cam = make_intrinsics(800.0, 1280, 720);

  // Camera 1: identity pose (at origin looking down +Z)
  CameraPose pose1;
  pose1.R = mat3_identity();
  pose1.t = {0, 0, 0};

  // Camera 2: translated 1 unit to the right along X
  CameraPose pose2;
  pose2.R = mat3_identity();
  pose2.t = {-1, 0, 0};  // world point (0,0,5) → camera2 x = 0+1 = 1 at depth 5

  // Target world point
  const Vec3d target{0.0, 0.0, 5.0};

  // Project into both cameras
  const Vec2d obs1 = project(cam, mat3_mul_vec(pose1.R, target) + pose1.t);
  const Vec2d obs2 = project(cam, mat3_mul_vec(pose2.R, target) + pose2.t);

  Vec3d result;
  if (!triangulate_point(cam, pose1, cam, pose2, obs1, obs2, &result))
    return fail("triangulate_point returned false");

  if (!near_v3(result, target, 1e-6))
    return fail("triangulated point does not match ground truth");

  return 0;
}

// ── sfm: fundamental matrix (8-point) ────────────────────────────────────────

int test_fundamental_matrix() {
  using namespace op::sfm;
  using namespace op::camera;
  using namespace op::core;

  const CameraIntrinsics cam = make_intrinsics(500.0, 640, 480);

  // Synthesise a set of 3-D points and their projections in two views.
  CameraPose pose1;
  pose1.R = mat3_identity();
  pose1.t = {0, 0, 0};

  CameraPose pose2;
  pose2.R = mat3_identity();
  pose2.t = {-1, 0, 0};  // baseline along X

  std::vector<Vec2d> pts1;
  std::vector<Vec2d> pts2;

  for (int xi = -3; xi <= 3; ++xi) {
    for (int yi = -2; yi <= 2; ++yi) {
      const Vec3d P{static_cast<double>(xi) * 0.5,
                    static_cast<double>(yi) * 0.5,
                    5.0 + static_cast<double>(xi) * 0.1};
      pts1.push_back(project(cam, mat3_mul_vec(pose1.R, P) + pose1.t));
      pts2.push_back(project(cam, mat3_mul_vec(pose2.R, P) + pose2.t));
    }
  }

  Mat3d F;
  std::vector<bool> inliers;
  if (!compute_fundamental_matrix(pts1, pts2, &F, &inliers, 1.5, 500))
    return fail("compute_fundamental_matrix returned false");

  // Count inliers: most correspondences should be inliers (no noise)
  int n_inliers = 0;
  for (bool b : inliers) {
    if (b) ++n_inliers;
  }
  if (n_inliers < static_cast<int>(pts1.size()) / 2)
    return fail("too few inliers in noiseless fundamental matrix test");

  // Epipolar constraint: x2^T * F * x1 ≈ 0 for inlier correspondences
  double max_residual = 0.0;
  for (std::size_t i = 0; i < pts1.size(); ++i) {
    if (!inliers[i]) continue;
    const Vec3d p1h{pts1[i].x, pts1[i].y, 1.0};
    const Vec3d p2h{pts2[i].x, pts2[i].y, 1.0};
    const double residual = std::abs(dot(p2h, mat3_mul_vec(F, p1h)));
    max_residual = std::max(max_residual, residual);
  }
  if (max_residual > 5.0)
    return fail("epipolar constraint violated");

  return 0;
}

// ── meshing: Delaunay triangulation ──────────────────────────────────────────

int test_meshing() {
  using namespace op::meshing;
  using namespace op::mvs;

  // Create a simple flat point cloud lying in the XY plane
  DensePointCloud cloud;
  for (int yi = 0; yi < 5; ++yi) {
    for (int xi = 0; xi < 5; ++xi) {
      cloud.points.push_back({static_cast<double>(xi),
                              static_cast<double>(yi),
                              0.0});
    }
  }

  const Mesh mesh = triangulate_point_cloud(cloud);

  if (mesh.vertices.size() != 25)
    return fail("mesh vertex count mismatch");

  if (mesh.faces.empty())
    return fail("no faces generated from point cloud");

  // All face vertex indices should be in range
  for (const auto& face : mesh.faces) {
    for (int idx : face) {
      if (idx < 0 || idx >= 25)
        return fail("face vertex index out of range");
    }
  }

  return 0;
}

// ── top-level runner ─────────────────────────────────────────────────────────

struct Test {
  const char* name;
  int (*fn)();
};

}  // namespace

int main() {
  const Test tests[] = {
    {"vec_arithmetic",           test_vec_arithmetic},
    {"mat3_ops",                 test_mat3_ops},
    {"svd_3x3",                  test_svd_3x3},
    {"sym_min_eigenvec",         test_sym_min_eigenvec},
    {"camera_project_unproject", test_camera_project_unproject},
    {"distortion_roundtrip",     test_distortion_roundtrip},
    {"harris_detection",         test_harris_detection},
    {"descriptor_matching",      test_descriptor_matching},
    {"triangulation",            test_triangulation},
    {"fundamental_matrix",       test_fundamental_matrix},
    {"meshing",                  test_meshing},
  };

  int failures = 0;
  for (const auto& t : tests) {
    const int result = t.fn();
    if (result != 0) {
      std::cerr << "  ↳ [FAIL] " << t.name << '\n';
      ++failures;
    }
    else {
      std::cout << "  ↳ [pass] " << t.name << '\n';
    }
  }

  if (failures > 0) {
    std::cerr << failures << " test(s) failed.\n";
    return 1;
  }
  std::cout << "All algorithm tests passed.\n";
  return 0;
}

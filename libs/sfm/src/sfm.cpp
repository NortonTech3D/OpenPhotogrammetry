#include "openphotogrammetry/sfm/sfm.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <vector>

#include "openphotogrammetry/features/features.hpp"

namespace op::sfm {

// ── Existing API ──────────────────────────────────────────────────────────────

int minimum_view_count() {
  return op::features::has_viable_feature_budget() ? 2 : 0;
}

bool can_initialize_reconstruction() {
  return minimum_view_count() >= 2;
}

// ── Utility ───────────────────────────────────────────────────────────────────

op::core::Vec2d project_world(const op::camera::CameraIntrinsics& cam,
                               const CameraPose&                   pose,
                               op::core::Vec3d                     w) {
  // Transform to camera space
  const op::core::Vec3d c = op::core::mat3_mul_vec(pose.R, w) + pose.t;
  return op::camera::project(cam, c);
}

// ── Normalisation helpers for the 8-point algorithm ──────────────────────────

namespace {

struct NormTransform {
  double scale{1};
  op::core::Vec2d offset{};
};

// Translate and scale points so the centroid is at the origin and the RMS
// distance from the origin is sqrt(2).
NormTransform compute_normalisation(const std::vector<op::core::Vec2d>& pts) {
  NormTransform T;
  if (pts.empty()) {
    return T;
  }
  double cx = 0.0;
  double cy = 0.0;
  for (const auto& p : pts) {
    cx += p.x;
    cy += p.y;
  }
  cx /= static_cast<double>(pts.size());
  cy /= static_cast<double>(pts.size());
  double rms = 0.0;
  for (const auto& p : pts) {
    const double dx = p.x - cx;
    const double dy = p.y - cy;
    rms += dx * dx + dy * dy;
  }
  rms = std::sqrt(rms / static_cast<double>(pts.size()));
  T.scale  = (rms > 1e-12) ? (std::sqrt(2.0) / rms) : 1.0;
  T.offset = {cx, cy};
  return T;
}

op::core::Vec2d apply_norm(NormTransform T, op::core::Vec2d p) {
  return {T.scale * (p.x - T.offset.x), T.scale * (p.y - T.offset.y)};
}

// 3×3 normalisation matrix corresponding to transform T:
//   T_mat * [x, y, 1]^T = [T.scale*(x-cx), T.scale*(y-cy), 1]^T
op::core::Mat3d norm_matrix(NormTransform T) {
  op::core::Mat3d M;
  M(0, 0) = T.scale;  M(0, 2) = -T.scale * T.offset.x;
  M(1, 1) = T.scale;  M(1, 2) = -T.scale * T.offset.y;
  M(2, 2) = 1.0;
  return M;
}

// ── 8-point algorithm (single estimate, no RANSAC) ───────────────────────────

// Given ≥8 normalised point correspondences, compute F via linear least squares
// and enforce the rank-2 constraint.
bool eight_point(const std::vector<op::core::Vec2d>& p1n,
                 const std::vector<op::core::Vec2d>& p2n,
                 op::core::Mat3d*                     F_out) {
  const int N = static_cast<int>(p1n.size());
  if (N < 8) {
    return false;
  }

  // Build 9×9 symmetric matrix A^T * A where each row of A is
  // [x'x, x'y, x', y'x, y'y, y', x, y, 1]
  std::vector<double> AtA(81, 0.0);
  for (int i = 0; i < N; ++i) {
    const double x  = p1n[i].x;
    const double y  = p1n[i].y;
    const double xp = p2n[i].x;
    const double yp = p2n[i].y;
    const double row[9] = {xp * x, xp * y, xp, yp * x, yp * y, yp, x, y, 1.0};
    for (int r = 0; r < 9; ++r)
      for (int c = 0; c < 9; ++c)
        AtA[static_cast<std::size_t>(r) * 9 + c] += row[r] * row[c];
  }

  // Find the null space (smallest eigenvector of A^T * A)
  std::vector<double> fvec;
  op::core::sym_min_eigenvec(AtA, 9, &fvec);

  op::core::Mat3d F;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      F(r, c) = fvec[static_cast<std::size_t>(r) * 3 + c];

  // Enforce rank-2 constraint: set smallest singular value to zero
  op::core::Mat3d U;
  op::core::Vec3d S;
  op::core::Mat3d Vt;
  if (!op::core::mat3_svd(F, &U, &S, &Vt)) {
    return false;
  }
  // Reconstruct with S[2] = 0
  op::core::Mat3d D;
  D(0, 0) = S.x;
  D(1, 1) = S.y;
  D(2, 2) = 0.0;
  *F_out = op::core::mat3_mul(U, op::core::mat3_mul(D, Vt));
  return true;
}

// Sampson distance for a pair of points and a fundamental matrix F.
double sampson_error(op::core::Mat3d F, op::core::Vec2d p1, op::core::Vec2d p2) {
  // Fp1 = F * [x, y, 1]^T
  const op::core::Vec3d p1h = {p1.x, p1.y, 1.0};
  const op::core::Vec3d p2h = {p2.x, p2.y, 1.0};
  const op::core::Vec3d Fp1  = op::core::mat3_mul_vec(F,                     p1h);
  const op::core::Vec3d Ftp2 = op::core::mat3_mul_vec(op::core::mat3_transpose(F), p2h);
  const double num  = op::core::dot(p2h, Fp1);
  const double den  = Fp1.x  * Fp1.x  + Fp1.y  * Fp1.y
                    + Ftp2.x * Ftp2.x + Ftp2.y * Ftp2.y;
  return (den > 1e-20) ? (num * num / den) : 1e20;
}

}  // namespace

// ── compute_fundamental_matrix ────────────────────────────────────────────────

bool compute_fundamental_matrix(const std::vector<op::core::Vec2d>& pts1,
                                const std::vector<op::core::Vec2d>& pts2,
                                op::core::Mat3d*                     F_out,
                                std::vector<bool>*                   inliers_out,
                                double                               threshold,
                                int                                  ransac_iters) {
  if (pts1.size() != pts2.size() || pts1.size() < 8 || F_out == nullptr) {
    return false;
  }
  const int N = static_cast<int>(pts1.size());

  // Compute normalisation transforms
  const NormTransform T1 = compute_normalisation(pts1);
  const NormTransform T2 = compute_normalisation(pts2);
  const op::core::Mat3d T1m = norm_matrix(T1);
  const op::core::Mat3d T2m = norm_matrix(T2);

  std::vector<op::core::Vec2d> n1(static_cast<std::size_t>(N));
  std::vector<op::core::Vec2d> n2(static_cast<std::size_t>(N));
  for (int i = 0; i < N; ++i) {
    n1[i] = apply_norm(T1, pts1[i]);
    n2[i] = apply_norm(T2, pts2[i]);
  }

  // RANSAC
  std::mt19937 rng(42);
  std::uniform_int_distribution<int> dist(0, N - 1);

  int best_count = 0;
  op::core::Mat3d best_F;

  const double threshold_sq = threshold * threshold;

  for (int iter = 0; iter < ransac_iters; ++iter) {
    // Sample 8 distinct points
    int indices[8];
    for (int k = 0; k < 8; ++k) {
      bool unique = false;
      while (!unique) {
        indices[k] = dist(rng);
        unique = true;
        for (int j = 0; j < k; ++j) {
          if (indices[j] == indices[k]) {
            unique = false;
            break;
          }
        }
      }
    }

    std::vector<op::core::Vec2d> s1(8);
    std::vector<op::core::Vec2d> s2(8);
    for (int k = 0; k < 8; ++k) {
      s1[k] = n1[indices[k]];
      s2[k] = n2[indices[k]];
    }

    op::core::Mat3d Fn;
    if (!eight_point(s1, s2, &Fn)) {
      continue;
    }
    // Denormalise: F = T2^T * Fn * T1
    const op::core::Mat3d Fd = op::core::mat3_mul(op::core::mat3_transpose(T2m),
                                                    op::core::mat3_mul(Fn, T1m));

    // Count inliers
    int count = 0;
    for (int i = 0; i < N; ++i) {
      if (sampson_error(Fd, pts1[i], pts2[i]) < threshold_sq) {
        ++count;
      }
    }

    if (count > best_count) {
      best_count = count;
      best_F     = Fd;
    }
  }

  if (best_count < 8) {
    return false;
  }

  // Refit using all inliers
  std::vector<op::core::Vec2d> in1;
  std::vector<op::core::Vec2d> in2;
  in1.reserve(static_cast<std::size_t>(best_count));
  in2.reserve(static_cast<std::size_t>(best_count));

  if (inliers_out != nullptr) {
    inliers_out->assign(static_cast<std::size_t>(N), false);
  }

  for (int i = 0; i < N; ++i) {
    if (sampson_error(best_F, pts1[i], pts2[i]) < threshold_sq) {
      in1.push_back(apply_norm(T1, pts1[i]));
      in2.push_back(apply_norm(T2, pts2[i]));
      if (inliers_out != nullptr) {
        (*inliers_out)[i] = true;
      }
    }
  }

  op::core::Mat3d Fn;
  if (!eight_point(in1, in2, &Fn)) {
    *F_out = best_F;
    return true;
  }
  *F_out = op::core::mat3_mul(op::core::mat3_transpose(T2m), op::core::mat3_mul(Fn, T1m));
  return true;
}

// ── essential_from_fundamental ────────────────────────────────────────────────

op::core::Mat3d essential_from_fundamental(op::core::Mat3d                     F,
                                           const op::camera::CameraIntrinsics& cam1,
                                           const op::camera::CameraIntrinsics& cam2) {
  // K1 = [[fx1, 0, cx1], [0, fy1, cy1], [0, 0, 1]]
  op::core::Mat3d K1;
  K1(0, 0) = cam1.fx;  K1(0, 2) = cam1.cx;
  K1(1, 1) = cam1.fy;  K1(1, 2) = cam1.cy;
  K1(2, 2) = 1.0;

  op::core::Mat3d K2;
  K2(0, 0) = cam2.fx;  K2(0, 2) = cam2.cx;
  K2(1, 1) = cam2.fy;  K2(1, 2) = cam2.cy;
  K2(2, 2) = 1.0;

  // E = K2^T * F * K1
  return op::core::mat3_mul(op::core::mat3_transpose(K2), op::core::mat3_mul(F, K1));
}

// ── recover_pose ──────────────────────────────────────────────────────────────

namespace {

// Build skew-symmetric matrix for cross-product: [v]×
[[maybe_unused]] op::core::Mat3d skew(op::core::Vec3d v) {
  op::core::Mat3d S;
  S(0, 1) = -v.z;  S(0, 2) =  v.y;
  S(1, 0) =  v.z;  S(1, 2) = -v.x;
  S(2, 0) = -v.y;  S(2, 1) =  v.x;
  return S;
}

// Count points with positive depth in both cameras (cheirality check).
int cheirality_count(const op::core::Mat3d& R, const op::core::Vec3d& t,
                     const std::vector<op::core::Vec2d>& pts1,
                     const std::vector<op::core::Vec2d>& pts2,
                     const op::camera::CameraIntrinsics& cam) {
  // Camera 1: identity pose
  op::sfm::CameraPose pose1;
  pose1.R = op::core::mat3_identity();
  pose1.t = {0, 0, 0};

  op::sfm::CameraPose pose2;
  pose2.R = R;
  pose2.t = t;

  int count = 0;
  for (std::size_t i = 0; i < pts1.size(); ++i) {
    op::core::Vec3d pt;
    if (!op::sfm::triangulate_point(cam, pose1, cam, pose2, pts1[i], pts2[i], &pt)) {
      continue;
    }
    // Check depth in cam1 (z > 0 in camera space)
    const op::core::Vec3d c1 = op::core::mat3_mul_vec(pose1.R, pt) + pose1.t;
    const op::core::Vec3d c2 = op::core::mat3_mul_vec(pose2.R, pt) + pose2.t;
    if (c1.z > 0.0 && c2.z > 0.0) {
      ++count;
    }
  }
  return count;
}

}  // namespace

bool recover_pose(const op::core::Mat3d&              E,
                  const std::vector<op::core::Vec2d>& pts1,
                  const std::vector<op::core::Vec2d>& pts2,
                  const op::camera::CameraIntrinsics& cam,
                  CameraPose*                          out) {
  if (out == nullptr || pts1.empty()) {
    return false;
  }

  op::core::Mat3d U;
  op::core::Vec3d S;
  op::core::Mat3d Vt;
  if (!op::core::mat3_svd(E, &U, &S, &Vt)) {
    return false;
  }

  // Enforce singular values [1, 1, 0]
  op::core::Mat3d D;
  D(0, 0) = 1.0;  D(1, 1) = 1.0;  D(2, 2) = 0.0;
  const op::core::Mat3d E2 = op::core::mat3_mul(U, op::core::mat3_mul(D, Vt));
  op::core::mat3_svd(E2, &U, &S, &Vt);

  // W = [[0,-1,0],[1,0,0],[0,0,1]]
  op::core::Mat3d W;
  W(0, 1) = -1.0;  W(1, 0) = 1.0;  W(2, 2) = 1.0;
  const op::core::Mat3d Wt = op::core::mat3_transpose(W);

  // Two rotation candidates
  const op::core::Mat3d R1 = op::core::mat3_mul(U, op::core::mat3_mul(W,  Vt));
  const op::core::Mat3d R2 = op::core::mat3_mul(U, op::core::mat3_mul(Wt, Vt));

  // Ensure proper rotation (det = +1)
  auto fix_rot = [](op::core::Mat3d R) {
    if (op::core::mat3_det(R) < 0.0) {
      for (int i = 0; i < 9; ++i) {
        R.d[i] = -R.d[i];
      }
    }
    return R;
  };
  const op::core::Mat3d Ra = fix_rot(R1);
  const op::core::Mat3d Rb = fix_rot(R2);

  // Translation: third column of U
  const op::core::Vec3d ta = {U(0, 2),  U(1, 2),  U(2, 2)};
  const op::core::Vec3d tb = {-U(0, 2), -U(1, 2), -U(2, 2)};

  // Pick the solution with the most points in front of both cameras
  struct Candidate {
    op::core::Mat3d R;
    op::core::Vec3d t;
  };
  const Candidate candidates[4] = {{Ra, ta}, {Ra, tb}, {Rb, ta}, {Rb, tb}};

  int   best_count  = -1;
  int   best_idx    = 0;
  for (int i = 0; i < 4; ++i) {
    const int cnt = cheirality_count(candidates[i].R, candidates[i].t, pts1, pts2, cam);
    if (cnt > best_count) {
      best_count = cnt;
      best_idx   = i;
    }
  }

  out->R = candidates[best_idx].R;
  out->t = candidates[best_idx].t;
  return true;
}

// ── triangulate_point ─────────────────────────────────────────────────────────

bool triangulate_point(const op::camera::CameraIntrinsics& cam1,
                       const CameraPose&                   pose1,
                       const op::camera::CameraIntrinsics& cam2,
                       const CameraPose&                   pose2,
                       op::core::Vec2d                     obs1,
                       op::core::Vec2d                     obs2,
                       op::core::Vec3d*                    out) {
  if (out == nullptr) {
    return false;
  }

  // Normalised (undistorted) ray directions in each camera's coordinate frame
  const op::core::Vec3d d1 = op::camera::unproject_ray(cam1, obs1);
  const op::core::Vec3d d2 = op::camera::unproject_ray(cam2, obs2);

  // Build the DLT system: each view contributes 2 rows.
  // Row from view 1:  d1.x * [R1 t1][3] - d1.z * [R1 t1][1]  etc.
  // We use the cross-product formulation:
  //   (d × (R*X + t)) = 0  →  [d]× * P * X = 0

  // Build 3×4 projection matrices P = [R | t]
  // Then for a point X (homogeneous), and ray direction d (homogeneous at z=1):
  //   Build 4×4 system, solve for X.

  // P1 = [R1 | t1]  (3×4)
  // P2 = [R2 | t2]

  auto make_row = [](const op::core::Mat3d& R, const op::core::Vec3d& t,
                     const op::core::Vec3d& d, int row_type) -> std::array<double, 4> {
    // row_type 0: cross product gives  d.y * P[2] - d.z * P[1]
    // row_type 1:                      d.z * P[0] - d.x * P[2]
    double P[3][4];
    P[0][0] = R(0, 0);  P[0][1] = R(0, 1);  P[0][2] = R(0, 2);  P[0][3] = t.x;
    P[1][0] = R(1, 0);  P[1][1] = R(1, 1);  P[1][2] = R(1, 2);  P[1][3] = t.y;
    P[2][0] = R(2, 0);  P[2][1] = R(2, 1);  P[2][2] = R(2, 2);  P[2][3] = t.z;

    std::array<double, 4> row{};
    if (row_type == 0) {
      for (int c = 0; c < 4; ++c)
        row[c] = d.y * P[2][c] - d.z * P[1][c];
    }
    else {
      for (int c = 0; c < 4; ++c)
        row[c] = d.z * P[0][c] - d.x * P[2][c];
    }
    return row;
  };

  // Form 4×4 system A where A*X = 0, X = [X, Y, Z, W]^T
  std::vector<double> A(16, 0.0);
  auto r0 = make_row(pose1.R, pose1.t, d1, 0);
  auto r1 = make_row(pose1.R, pose1.t, d1, 1);
  auto r2 = make_row(pose2.R, pose2.t, d2, 0);
  auto r3 = make_row(pose2.R, pose2.t, d2, 1);
  for (int c = 0; c < 4; ++c) {
    A[c]      = r0[c];
    A[4 + c]  = r1[c];
    A[8 + c]  = r2[c];
    A[12 + c] = r3[c];
  }

  // Compute A^T * A (4×4 symmetric) and find smallest eigenvector
  std::vector<double> AtA(16, 0.0);
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k)
        s += A[static_cast<std::size_t>(k) * 4 + r] * A[static_cast<std::size_t>(k) * 4 + c];
      AtA[static_cast<std::size_t>(r) * 4 + c] = s;
    }

  std::vector<double> Xh;
  op::core::sym_min_eigenvec(AtA, 4, &Xh);

  if (std::abs(Xh[3]) < 1e-12) {
    return false;  // point at infinity
  }

  *out = {Xh[0] / Xh[3], Xh[1] / Xh[3], Xh[2] / Xh[3]};

  // Cheirality check: point must be in front of both cameras
  const op::core::Vec3d c1 = op::core::mat3_mul_vec(pose1.R, *out) + pose1.t;
  const op::core::Vec3d c2 = op::core::mat3_mul_vec(pose2.R, *out) + pose2.t;
  return c1.z > 0.0 && c2.z > 0.0;
}

// ── triangulate_matches ───────────────────────────────────────────────────────

std::vector<SparsePoint> triangulate_matches(
    const op::camera::CameraIntrinsics&             cam1,
    const CameraPose&                               pose1,
    int                                             view_idx1,
    const std::vector<op::features::Keypoint>&      kps1,
    const op::camera::CameraIntrinsics&             cam2,
    const CameraPose&                               pose2,
    int                                             view_idx2,
    const std::vector<op::features::Keypoint>&      kps2,
    const std::vector<op::features::KeypointMatch>& matches) {
  std::vector<SparsePoint> pts;
  pts.reserve(matches.size());

  for (const auto& m : matches) {
    if (m.query_idx < 0 || m.train_idx < 0) {
      continue;
    }
    const op::features::Keypoint& kp1 = kps1[static_cast<std::size_t>(m.query_idx)];
    const op::features::Keypoint& kp2 = kps2[static_cast<std::size_t>(m.train_idx)];

    op::core::Vec3d pt;
    if (!triangulate_point(cam1, pose1, cam2, pose2,
                           {kp1.x, kp1.y}, {kp2.x, kp2.y}, &pt)) {
      continue;
    }

    SparsePoint sp;
    sp.position = pt;
    sp.tracks.push_back({view_idx1, {kp1.x, kp1.y}});
    sp.tracks.push_back({view_idx2, {kp2.x, kp2.y}});
    pts.push_back(std::move(sp));
  }
  return pts;
}

// ── run_incremental_sfm ───────────────────────────────────────────────────────

bool run_incremental_sfm(const std::vector<op::features::GrayscaleImage>&  images,
                         const std::vector<op::camera::CameraIntrinsics>&  intrinsics,
                         SparseReconstruction*                              result,
                         std::string*                                       error) {
  auto fail = [&](std::string msg) {
    if (error) *error = std::move(msg);
    return false;
  };

  if (images.size() != intrinsics.size()) {
    return fail("images and intrinsics size mismatch");
  }
  const int N = static_cast<int>(images.size());
  if (N < 2) {
    return fail("need at least 2 images");
  }

  // Detect and describe features in all images
  std::vector<std::vector<op::features::Keypoint>>   all_kps(static_cast<std::size_t>(N));
  std::vector<std::vector<op::features::Descriptor>> all_desc(static_cast<std::size_t>(N));
  for (int i = 0; i < N; ++i) {
    all_kps[i]  = op::features::detect_keypoints(images[i]);
    all_desc[i] = op::features::compute_descriptors(images[i], all_kps[i]);
  }

  // Initialise reconstruction from first pair
  result->intrinsics = intrinsics;
  result->poses.resize(static_cast<std::size_t>(N));
  result->points.clear();

  // Camera 0 is at the origin
  result->poses[0].R = op::core::mat3_identity();
  result->poses[0].t = {0, 0, 0};

  bool any_registered = false;

  // Process sequential pairs: 0-1, 1-2, ...
  for (int i = 0; i < N - 1; ++i) {
    const int j = i + 1;

    // Match features
    auto matches = op::features::match_descriptors(all_desc[i], all_desc[j]);
    if (static_cast<int>(matches.size()) < 8) {
      continue;
    }

    // Build correspondence vectors
    std::vector<op::core::Vec2d> pts_i;
    std::vector<op::core::Vec2d> pts_j;
    for (const auto& m : matches) {
      pts_i.push_back({all_kps[i][m.query_idx].x, all_kps[i][m.query_idx].y});
      pts_j.push_back({all_kps[j][m.train_idx].x, all_kps[j][m.train_idx].y});
    }

    // Compute fundamental matrix with RANSAC
    op::core::Mat3d F;
    std::vector<bool> inliers;
    if (!compute_fundamental_matrix(pts_i, pts_j, &F, &inliers)) {
      continue;
    }

    // Convert to essential matrix
    const op::core::Mat3d E = essential_from_fundamental(F, intrinsics[i], intrinsics[j]);

    // Filter to inlier correspondences for pose recovery
    std::vector<op::core::Vec2d> in_i;
    std::vector<op::core::Vec2d> in_j;
    for (std::size_t k = 0; k < inliers.size(); ++k) {
      if (inliers[k]) {
        in_i.push_back(pts_i[k]);
        in_j.push_back(pts_j[k]);
      }
    }
    if (in_i.size() < 8) {
      continue;
    }

    // Recover relative pose of camera j w.r.t. camera i
    CameraPose rel_pose;
    if (!recover_pose(E, in_i, in_j, intrinsics[i], &rel_pose)) {
      continue;
    }

    if (i == 0) {
      result->poses[j] = rel_pose;
    }
    else {
      // Chain pose: pose_j = rel_pose ∘ pose_i
      result->poses[j].R = op::core::mat3_mul(rel_pose.R, result->poses[i].R);
      result->poses[j].t = op::core::mat3_mul_vec(rel_pose.R, result->poses[i].t) + rel_pose.t;
    }

    // Triangulate inlier matches
    std::vector<op::features::KeypointMatch> inlier_matches;
    for (std::size_t k = 0; k < inliers.size(); ++k) {
      if (inliers[k]) {
        inlier_matches.push_back(matches[k]);
      }
    }

    auto new_pts = triangulate_matches(intrinsics[i], result->poses[i], i,
                                       all_kps[i],
                                       intrinsics[j], result->poses[j], j,
                                       all_kps[j],
                                       inlier_matches);
    for (auto& pt : new_pts) {
      result->points.push_back(std::move(pt));
    }
    any_registered = true;
  }

  if (!any_registered) {
    return fail("no image pairs could be registered");
  }
  return true;
}

}  // namespace op::sfm

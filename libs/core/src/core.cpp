#include "openphotogrammetry/core/core.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace op::core {

// ── Existing runtime checks ───────────────────────────────────────────────────

int version_major() {
  return 1;
}

bool is_runtime_ready() {
  return version_major() == 1;
}

// ── Matrix construction ───────────────────────────────────────────────────────

Mat3d mat3_identity() {
  Mat3d m;
  m(0, 0) = m(1, 1) = m(2, 2) = 1.0;
  return m;
}

Mat3d mat3_from_rows(Vec3d r0, Vec3d r1, Vec3d r2) {
  Mat3d m;
  m(0, 0) = r0.x;  m(0, 1) = r0.y;  m(0, 2) = r0.z;
  m(1, 0) = r1.x;  m(1, 1) = r1.y;  m(1, 2) = r1.z;
  m(2, 0) = r2.x;  m(2, 1) = r2.y;  m(2, 2) = r2.z;
  return m;
}

Mat3d mat3_from_cols(Vec3d c0, Vec3d c1, Vec3d c2) {
  Mat3d m;
  m(0, 0) = c0.x;  m(0, 1) = c1.x;  m(0, 2) = c2.x;
  m(1, 0) = c0.y;  m(1, 1) = c1.y;  m(1, 2) = c2.y;
  m(2, 0) = c0.z;  m(2, 1) = c1.z;  m(2, 2) = c2.z;
  return m;
}

Mat3d mat3_diag(Vec3d s) {
  Mat3d m;
  m(0, 0) = s.x;  m(1, 1) = s.y;  m(2, 2) = s.z;
  return m;
}

// ── Matrix operations ─────────────────────────────────────────────────────────

Mat3d mat3_transpose(Mat3d m) {
  Mat3d t;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      t(r, c) = m(c, r);
  return t;
}

Mat3d mat3_mul(Mat3d a, Mat3d b) {
  Mat3d c;
  for (int r = 0; r < 3; ++r)
    for (int col = 0; col < 3; ++col) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k)
        s += a(r, k) * b(k, col);
      c(r, col) = s;
    }
  return c;
}

Vec3d mat3_mul_vec(Mat3d m, Vec3d v) {
  return {
    m(0, 0) * v.x + m(0, 1) * v.y + m(0, 2) * v.z,
    m(1, 0) * v.x + m(1, 1) * v.y + m(1, 2) * v.z,
    m(2, 0) * v.x + m(2, 1) * v.y + m(2, 2) * v.z,
  };
}

double mat3_det(Mat3d m) {
  return m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1))
       - m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0))
       + m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
}

bool mat3_inv(Mat3d m, Mat3d* out) {
  const double det = mat3_det(m);
  if (std::abs(det) < 1e-15) {
    return false;
  }
  const double inv_det = 1.0 / det;
  Mat3d inv;
  inv(0, 0) = (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)) * inv_det;
  inv(0, 1) = (m(0, 2) * m(2, 1) - m(0, 1) * m(2, 2)) * inv_det;
  inv(0, 2) = (m(0, 1) * m(1, 2) - m(0, 2) * m(1, 1)) * inv_det;
  inv(1, 0) = (m(1, 2) * m(2, 0) - m(1, 0) * m(2, 2)) * inv_det;
  inv(1, 1) = (m(0, 0) * m(2, 2) - m(0, 2) * m(2, 0)) * inv_det;
  inv(1, 2) = (m(0, 2) * m(1, 0) - m(0, 0) * m(1, 2)) * inv_det;
  inv(2, 0) = (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0)) * inv_det;
  inv(2, 1) = (m(0, 1) * m(2, 0) - m(0, 0) * m(2, 1)) * inv_det;
  inv(2, 2) = (m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0)) * inv_det;
  if (out != nullptr) {
    *out = inv;
  }
  return true;
}

// ── Jacobi SVD for 3×3 matrices ──────────────────────────────────────────────
// Computes M = U * diag(S) * Vt where S is sorted in descending order.
// Based on one-sided Jacobi applied to M^T * M to obtain V and eigenvalues,
// then U = M * V * diag(1/S).

namespace {

Vec3d mat3_col(Mat3d m, int c) {
  return {m(0, c), m(1, c), m(2, c)};
}

void mat3_set_col(Mat3d* m, int c, Vec3d v) {
  (*m)(0, c) = v.x;
  (*m)(1, c) = v.y;
  (*m)(2, c) = v.z;
}

Vec3d orthogonal_unit(Vec3d v) {
  const Vec3d axis_x{1.0, 0.0, 0.0};
  const Vec3d axis_y{0.0, 1.0, 0.0};
  Vec3d n = normalize(cross(v, axis_x));
  if (norm(n) < 1e-12) {
    n = normalize(cross(v, axis_y));
  }
  return n;
}

// Apply a Jacobi rotation in the (p,q) plane to a symmetric 3×3 matrix A and
// accumulate into V (right eigenvectors):
//   A  ← G^T * A * G
//   V  ← V * G
// where G is the Givens rotation matrix for angle theta.
void jacobi_rotate_sym3(double A[3][3], double V[3][3], int p, int q) {
  const double app = A[p][p];
  const double aqq = A[q][q];
  const double apq = A[p][q];

  if (std::abs(apq) < 1e-18) {
    return;
  }

  const double tau = (aqq - app) / (2.0 * apq);
  const double t   = (tau >= 0.0) ? (1.0 / (tau + std::sqrt(1.0 + tau * tau)))
                                   : (1.0 / (tau - std::sqrt(1.0 + tau * tau)));
  const double c   = 1.0 / std::sqrt(1.0 + t * t);
  const double s   = c * t;

  // Update diagonal
  A[p][p] = app - t * apq;
  A[q][q] = aqq + t * apq;
  A[p][q] = A[q][p] = 0.0;

  // Update off-diagonal rows/cols (r != p, r != q)
  for (int r = 0; r < 3; ++r) {
    if (r == p || r == q) {
      continue;
    }
    const double arp = A[r][p];
    const double arq = A[r][q];
    A[r][p] = A[p][r] = c * arp - s * arq;
    A[r][q] = A[q][r] = s * arp + c * arq;
  }

  // Accumulate V
  for (int r = 0; r < 3; ++r) {
    const double vrp = V[r][p];
    const double vrq = V[r][q];
    V[r][p] = c * vrp - s * vrq;
    V[r][q] = s * vrp + c * vrq;
  }
}

}  // namespace

bool mat3_svd(Mat3d M, Mat3d* U_out, Vec3d* S_out, Mat3d* Vt_out) {
  // Form M^T * M (symmetric positive semi-definite)
  double MtM[3][3] = {};
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k)
        s += M(k, r) * M(k, c);
      MtM[r][c] = s;
    }

  // Jacobi diagonalization of MtM
  double V[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

  for (int iter = 0; iter < 200; ++iter) {
    // Find largest off-diagonal element
    double max_off = 0.0;
    for (int r = 0; r < 3; ++r)
      for (int c = r + 1; c < 3; ++c)
        if (std::abs(MtM[r][c]) > max_off)
          max_off = std::abs(MtM[r][c]);
    if (max_off < 1e-18) {
      break;
    }
    // Sweep all pairs
    for (int p = 0; p < 3; ++p)
      for (int q = p + 1; q < 3; ++q)
        jacobi_rotate_sym3(MtM, V, p, q);
  }

  // Eigenvalues are diagonal of MtM; singular values = sqrt(eigenvalues)
  double sv[3] = {std::sqrt(std::max(0.0, MtM[0][0])),
                  std::sqrt(std::max(0.0, MtM[1][1])),
                  std::sqrt(std::max(0.0, MtM[2][2]))};

  // Sort singular values in descending order, carrying V
  int idx[3] = {0, 1, 2};
  for (int i = 0; i < 2; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      if (sv[idx[j]] > sv[idx[i]]) {
        std::swap(idx[i], idx[j]);
      }
    }
  }

  // Build sorted Vt (row i = column idx[i] of V)
  Mat3d Vt;
  Vec3d S = {sv[idx[0]], sv[idx[1]], sv[idx[2]]};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      Vt(i, j) = V[j][idx[i]];

  // Compute U = M * V * diag(1/S)
  Mat3d Vm;  // sorted columns of V
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      Vm(i, j) = V[i][idx[j]];

  Mat3d U = mat3_mul(M, Vm);
  const double sv_vals[3] = {S.x, S.y, S.z};
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      U(r, c) = (sv_vals[c] > 1e-15) ? U(r, c) / sv_vals[c] : 0.0;

  // Re-orthonormalize U to handle rank-deficient inputs.
  const bool has0 = sv_vals[0] > 1e-12;
  const bool has1 = sv_vals[1] > 1e-12;
  const bool has2 = sv_vals[2] > 1e-12;
  Vec3d u0 = has0 ? normalize(mat3_col(U, 0)) : Vec3d{1.0, 0.0, 0.0};
  Vec3d u1 = mat3_col(U, 1) - dot(u0, mat3_col(U, 1)) * u0;
  if (!has1 || norm(u1) < 1e-12) {
    u1 = orthogonal_unit(u0);
  }
  else {
    u1 = normalize(u1);
  }
  Vec3d u2 = mat3_col(U, 2) - dot(u0, mat3_col(U, 2)) * u0 - dot(u1, mat3_col(U, 2)) * u1;
  if (!has2 || norm(u2) < 1e-12) {
    u2 = cross(u0, u1);
  }
  else {
    u2 = normalize(u2);
  }
  mat3_set_col(&U, 0, u0);
  mat3_set_col(&U, 1, u1);
  mat3_set_col(&U, 2, u2);

  // Enforce det(U) == +1 (handle reflections)
  if (mat3_det(U) < 0.0) {
    for (int r = 0; r < 3; ++r)
      U(r, 2) = -U(r, 2);
    // Also flip the paired right-singular vector to keep U*diag(S)*Vt unchanged.
    for (int c = 0; c < 3; ++c)
      Vt(2, c) = -Vt(2, c);
  }

  if (U_out)  *U_out  = U;
  if (S_out)  *S_out  = S;
  if (Vt_out) *Vt_out = Vt;
  return true;
}

// ── Smallest eigenvector of a symmetric N×N matrix ───────────────────────────
// Uses Jacobi diagonalization.  The matrix A is stored row-major (length n*n).

double sym_min_eigenvec(const std::vector<double>& A, int n, std::vector<double>* vec) {
  // Copy A into a mutable array; also initialise V to I
  std::vector<double> D(A);  // will be diagonalised in place
  std::vector<double> V(static_cast<std::size_t>(n) * n, 0.0);
  for (int i = 0; i < n; ++i) {
    V[static_cast<std::size_t>(i) * n + i] = 1.0;
  }

  auto Dref = [&](int r, int c) -> double& {
    return D[static_cast<std::size_t>(r) * n + c];
  };
  auto Vref = [&](int r, int c) -> double& {
    return V[static_cast<std::size_t>(r) * n + c];
  };

  const int max_iter = 100 * n * n;
  for (int iter = 0; iter < max_iter; ++iter) {
    // Find largest off-diagonal element
    double max_off = 0.0;
    int   best_p   = 0;
    int   best_q   = 1;
    for (int p = 0; p < n; ++p)
      for (int q = p + 1; q < n; ++q) {
        const double v = std::abs(Dref(p, q));
        if (v > max_off) {
          max_off = v;
          best_p  = p;
          best_q  = q;
        }
      }
    if (max_off < 1e-14) {
      break;
    }
    const int p = best_p;
    const int q = best_q;

    const double app = Dref(p, p);
    const double aqq = Dref(q, q);
    const double apq = Dref(p, q);
    const double tau = (aqq - app) / (2.0 * apq);
    const double t   = (tau >= 0.0) ? (1.0 / (tau + std::sqrt(1.0 + tau * tau)))
                                     : (1.0 / (tau - std::sqrt(1.0 + tau * tau)));
    const double c   = 1.0 / std::sqrt(1.0 + t * t);
    const double s   = c * t;

    Dref(p, p) = app - t * apq;
    Dref(q, q) = aqq + t * apq;
    Dref(p, q) = Dref(q, p) = 0.0;
    for (int r = 0; r < n; ++r) {
      if (r == p || r == q) {
        continue;
      }
      const double arp = Dref(r, p);
      const double arq = Dref(r, q);
      Dref(r, p) = Dref(p, r) = c * arp - s * arq;
      Dref(r, q) = Dref(q, r) = s * arp + c * arq;
    }
    for (int r = 0; r < n; ++r) {
      const double vrp = Vref(r, p);
      const double vrq = Vref(r, q);
      Vref(r, p) = c * vrp - s * vrq;
      Vref(r, q) = s * vrp + c * vrq;
    }
  }

  // Find index of smallest eigenvalue
  int min_idx = 0;
  for (int i = 1; i < n; ++i) {
    if (Dref(i, i) < Dref(min_idx, min_idx)) {
      min_idx = i;
    }
  }

  const double min_eval = Dref(min_idx, min_idx);
  if (vec != nullptr) {
    vec->resize(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
      (*vec)[static_cast<std::size_t>(i)] = Vref(i, min_idx);
    }
    // Normalise
    double len = 0.0;
    for (double v : *vec) {
      len += v * v;
    }
    len = std::sqrt(len);
    if (len > 1e-15) {
      for (double& v : *vec) {
        v /= len;
      }
    }
  }
  return min_eval;
}

}  // namespace op::core

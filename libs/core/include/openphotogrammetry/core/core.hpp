#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace op::core {

int  version_major();
bool is_runtime_ready();

// ── Vector types ─────────────────────────────────────────────────────────────

struct Vec2d {
  double x{0}, y{0};
};

struct Vec3d {
  double x{0}, y{0}, z{0};
};

struct Vec4d {
  double x{0}, y{0}, z{0}, w{0};
};

// ── Row-major 3×3 matrix ─────────────────────────────────────────────────────

struct Mat3d {
  double d[9]{};
  double&       operator()(int r, int c) { return d[r * 3 + c]; }
  double        operator()(int r, int c) const { return d[r * 3 + c]; }
};

// ── Inline vector arithmetic ─────────────────────────────────────────────────

inline Vec2d operator+(Vec2d a, Vec2d b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2d operator-(Vec2d a, Vec2d b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2d operator*(Vec2d v, double s) { return {v.x * s, v.y * s}; }
inline Vec2d operator*(double s, Vec2d v) { return {v.x * s, v.y * s}; }

inline Vec3d operator+(Vec3d a, Vec3d b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3d operator-(Vec3d a, Vec3d b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3d operator*(Vec3d v, double s) { return {v.x * s, v.y * s, v.z * s}; }
inline Vec3d operator*(double s, Vec3d v) { return {v.x * s, v.y * s, v.z * s}; }
inline Vec3d operator-(Vec3d v) { return {-v.x, -v.y, -v.z}; }

inline double dot(Vec2d a, Vec2d b) { return a.x * b.x + a.y * b.y; }
inline double dot(Vec3d a, Vec3d b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline double norm(Vec2d v) { return std::sqrt(dot(v, v)); }
inline double norm(Vec3d v) { return std::sqrt(dot(v, v)); }

inline Vec2d normalize(Vec2d v) {
  const double n = norm(v);
  return n > 1e-15 ? Vec2d{v.x / n, v.y / n} : Vec2d{};
}
inline Vec3d normalize(Vec3d v) {
  const double n = norm(v);
  return n > 1e-15 ? Vec3d{v.x / n, v.y / n, v.z / n} : Vec3d{};
}

inline Vec3d cross(Vec3d a, Vec3d b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// ── Matrix construction ───────────────────────────────────────────────────────

Mat3d mat3_identity();
Mat3d mat3_from_rows(Vec3d r0, Vec3d r1, Vec3d r2);
Mat3d mat3_from_cols(Vec3d c0, Vec3d c1, Vec3d c2);
Mat3d mat3_diag(Vec3d s);

// ── Matrix operations ─────────────────────────────────────────────────────────

Mat3d  mat3_transpose(Mat3d m);
Mat3d  mat3_mul(Mat3d a, Mat3d b);
Vec3d  mat3_mul_vec(Mat3d m, Vec3d v);
double mat3_det(Mat3d m);
bool   mat3_inv(Mat3d m, Mat3d* out);

// ── Decompositions ────────────────────────────────────────────────────────────

// Jacobi SVD: M = U * diag(S) * Vt, singular values sorted descending.
bool mat3_svd(Mat3d M, Mat3d* U, Vec3d* S, Mat3d* Vt);

// Find the eigenvector of the smallest eigenvalue of a symmetric N×N matrix
// stored row-major in `A` (length N*N).  Writes the unit eigenvector into
// `vec` (length N) and returns the eigenvalue.
double sym_min_eigenvec(const std::vector<double>& A, int n, std::vector<double>* vec);

}  // namespace op::core

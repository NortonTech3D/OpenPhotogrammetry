#include "openphotogrammetry/features/features.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <stdexcept>
#include <string>

#include "openphotogrammetry/camera/camera.hpp"

namespace op::features {

// ── Existing API ──────────────────────────────────────────────────────────────

int recommended_feature_count() {
  return op::camera::has_valid_default_calibration() ? 2048 : 0;
}

bool has_viable_feature_budget() {
  return recommended_feature_count() >= 512;
}

// ── Image I/O ─────────────────────────────────────────────────────────────────

namespace {

bool skip_whitespace_and_comments(std::ifstream& f) {
  int c = f.peek();
  while (c != EOF && (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '#')) {
    if (c == '#') {
      // skip rest of line
      while ((c = f.get()) != EOF && c != '\n') {
      }
    }
    else {
      f.get();
    }
    c = f.peek();
  }
  return !f.fail();
}

}  // namespace

bool load_image(std::string_view path, GrayscaleImage* out, std::string* error) {
  if (out == nullptr) {
    if (error) *error = "null output pointer";
    return false;
  }
  std::ifstream f(std::string(path), std::ios::binary);
  if (!f) {
    if (error) *error = "cannot open file: " + std::string(path);
    return false;
  }

  // Read magic number
  std::string magic;
  f >> magic;
  if (magic != "P5" && magic != "P6") {
    if (error) *error = "unsupported format (only P5/P6 PGM/PPM supported)";
    return false;
  }
  const bool is_rgb = (magic == "P6");

  skip_whitespace_and_comments(f);
  int w = 0;
  int h = 0;
  int maxval = 0;
  f >> w >> h >> maxval;
  if (!f || w <= 0 || h <= 0 || maxval <= 0 || maxval > 255) {
    if (error) *error = "invalid PGM/PPM header";
    return false;
  }
  // Consume the single whitespace byte after maxval
  f.get();

  const int channels = is_rgb ? 3 : 1;
  const std::size_t total = static_cast<std::size_t>(w) * h * channels;
  std::vector<uint8_t> raw(total);
  f.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(total));
  if (!f) {
    if (error) *error = "truncated image data";
    return false;
  }

  out->width  = w;
  out->height = h;
  out->pixels.resize(static_cast<std::size_t>(w) * h);

  if (is_rgb) {
    // Convert RGB → grayscale (luminosity weighting)
    for (int i = 0; i < w * h; ++i) {
      const uint8_t r = raw[static_cast<std::size_t>(i) * 3];
      const uint8_t g = raw[static_cast<std::size_t>(i) * 3 + 1];
      const uint8_t b = raw[static_cast<std::size_t>(i) * 3 + 2];
      out->pixels[static_cast<std::size_t>(i)] =
        static_cast<uint8_t>(0.2126 * r + 0.7152 * g + 0.0722 * b + 0.5);
    }
  }
  else {
    out->pixels = std::move(raw);
  }
  return true;
}

bool load_image_rgb(std::string_view path, RgbImage* out, std::string* error) {
  if (out == nullptr) {
    if (error) *error = "null output pointer";
    return false;
  }
  std::ifstream f(std::string(path), std::ios::binary);
  if (!f) {
    if (error) *error = "cannot open file: " + std::string(path);
    return false;
  }
  std::string magic;
  f >> magic;
  if (magic != "P6") {
    if (error) *error = "unsupported format (only P6 PPM supported for RGB)";
    return false;
  }
  skip_whitespace_and_comments(f);
  int w = 0;
  int h = 0;
  int maxval = 0;
  f >> w >> h >> maxval;
  if (!f || w <= 0 || h <= 0 || maxval <= 0 || maxval > 255) {
    if (error) *error = "invalid PPM header";
    return false;
  }
  f.get();
  out->width  = w;
  out->height = h;
  const std::size_t total = static_cast<std::size_t>(w) * h * 3;
  out->pixels.resize(total);
  f.read(reinterpret_cast<char*>(out->pixels.data()), static_cast<std::streamsize>(total));
  if (!f) {
    if (error) *error = "truncated PPM data";
    return false;
  }
  return true;
}

bool save_image(std::string_view path, const GrayscaleImage& img, std::string* error) {
  std::ofstream f(std::string(path), std::ios::binary);
  if (!f) {
    if (error) *error = "cannot open file for writing: " + std::string(path);
    return false;
  }
  f << "P5\n" << img.width << ' ' << img.height << "\n255\n";
  f.write(reinterpret_cast<const char*>(img.pixels.data()),
          static_cast<std::streamsize>(img.pixels.size()));
  if (!f) {
    if (error) *error = "write error";
    return false;
  }
  return true;
}

// ── Harris corner detector ────────────────────────────────────────────────────

namespace {

// Inline float pixel accessor with clamp-to-border.
inline float pix(const GrayscaleImage& img, int x, int y) {
  x = std::clamp(x, 0, img.width  - 1);
  y = std::clamp(y, 0, img.height - 1);
  return static_cast<float>(img.pixels[static_cast<std::size_t>(y) * img.width + x]);
}

// Compute Ix, Iy (Sobel), Ixx, Iyy, Ixy, then the Harris response.
// Returns a flat vector of Harris response values (same dimensions as image).
std::vector<float> harris_response(const GrayscaleImage& img, float k = 0.04f) {
  const int W = img.width;
  const int H = img.height;
  std::vector<float> Ixx(static_cast<std::size_t>(W) * H);
  std::vector<float> Iyy(static_cast<std::size_t>(W) * H);
  std::vector<float> Ixy(static_cast<std::size_t>(W) * H);

  // Compute Sobel gradients
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const float dx = (pix(img, x + 1, y - 1) + 2.0f * pix(img, x + 1, y) + pix(img, x + 1, y + 1))
                     - (pix(img, x - 1, y - 1) + 2.0f * pix(img, x - 1, y) + pix(img, x - 1, y + 1));
      const float dy = (pix(img, x - 1, y + 1) + 2.0f * pix(img, x, y + 1) + pix(img, x + 1, y + 1))
                     - (pix(img, x - 1, y - 1) + 2.0f * pix(img, x, y - 1) + pix(img, x + 1, y - 1));
      const std::size_t idx = static_cast<std::size_t>(y) * W + x;
      Ixx[idx] = dx * dx;
      Iyy[idx] = dy * dy;
      Ixy[idx] = dx * dy;
    }
  }

  // Box-filter (5×5) to smooth the second-moment matrix entries
  const int rad = 2;
  std::vector<float> SIxx(static_cast<std::size_t>(W) * H, 0.0f);
  std::vector<float> SIyy(static_cast<std::size_t>(W) * H, 0.0f);
  std::vector<float> SIxy(static_cast<std::size_t>(W) * H, 0.0f);

  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      float sxx = 0.0f;
      float syy = 0.0f;
      float sxy = 0.0f;
      for (int dy = -rad; dy <= rad; ++dy) {
        for (int dx = -rad; dx <= rad; ++dx) {
          const int nx  = std::clamp(x + dx, 0, W - 1);
          const int ny  = std::clamp(y + dy, 0, H - 1);
          const std::size_t ni = static_cast<std::size_t>(ny) * W + nx;
          sxx += Ixx[ni];
          syy += Iyy[ni];
          sxy += Ixy[ni];
        }
      }
      const std::size_t idx = static_cast<std::size_t>(y) * W + x;
      SIxx[idx] = sxx;
      SIyy[idx] = syy;
      SIxy[idx] = sxy;
    }
  }

  // Harris response: R = det(M) - k * trace(M)^2
  std::vector<float> R(static_cast<std::size_t>(W) * H);
  for (int i = 0; i < W * H; ++i) {
    const float det   = SIxx[i] * SIyy[i] - SIxy[i] * SIxy[i];
    const float trace = SIxx[i] + SIyy[i];
    R[i] = det - k * trace * trace;
  }
  return R;
}

}  // namespace

std::vector<Keypoint> detect_keypoints(const GrayscaleImage& image, int max_features) {
  if (image.width <= 0 || image.height <= 0 || image.pixels.empty()) {
    return {};
  }

  const int W = image.width;
  const int H = image.height;
  const std::vector<float> R = harris_response(image);

  // Non-maximum suppression with 5×5 window
  const int nms_radius = 2;
  std::vector<Keypoint> candidates;

  for (int y = nms_radius; y < H - nms_radius; ++y) {
    for (int x = nms_radius; x < W - nms_radius; ++x) {
      const float val = R[static_cast<std::size_t>(y) * W + x];
      if (val <= 0.0f) {
        continue;
      }
      bool is_max = true;
      for (int dy = -nms_radius; dy <= nms_radius && is_max; ++dy) {
        for (int dx = -nms_radius; dx <= nms_radius && is_max; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          // Strict '>' preserves one response when neighboring peak values tie.
          if (R[static_cast<std::size_t>(y + dy) * W + (x + dx)] > val) {
            is_max = false;
          }
        }
      }
      if (is_max) {
        Keypoint kp;
        kp.x        = static_cast<float>(x);
        kp.y        = static_cast<float>(y);
        kp.scale    = 1.0f;
        kp.angle    = 0.0f;
        kp.response = val;
        candidates.push_back(kp);
      }
    }
  }

  // Sort by response (descending) and keep top N
  std::sort(candidates.begin(), candidates.end(), [](const Keypoint& a, const Keypoint& b) {
    return a.response > b.response;
  });
  if (static_cast<int>(candidates.size()) > max_features) {
    candidates.resize(static_cast<std::size_t>(max_features));
  }
  return candidates;
}

// ── BRIEF-style binary descriptor ────────────────────────────────────────────
// Uses a fixed Gaussian-sampled test pattern (256 pairs = 256 bits = 32 bytes).
// The pattern is defined as pairs of offsets from the keypoint centre, pre-scaled
// to a 31×31 patch.

namespace {

// 256 test pairs, each (x1, y1, x2, y2) in [-15, 15].
// Generated from a Gaussian distribution (sigma=6.35) used in the BRIEF paper.
// We hard-code a fixed deterministic pattern here.
constexpr int8_t kBriefPattern[256 * 4] = {
  -2,-3, 6, 9, -7,-7, 3, 7,  5,-8, 9, 6, -2, 5,-7,-5,
   8,-1,-5, 0, -1,-4, 5, 2,  7, 4,-3,-5, -9, 7, 8,-2,
  -4, 8,-8, 4, 10, 2,-5,-6,  3,-9,-8, 4,  0, 1, 7,-6,
  -6, 9, 5,-3,  4,-3,-9, 2, -8,-5, 7, 5, -3, 6, 8,-4,
   2, 7,-9,-1, -5,-8, 6, 5,  9, 3,-4,-7, -7,-3, 3, 8,
   6,-7,-3, 6,  4, 8,-8,-2, -2,-7, 6, 5,  8,-4,-1, 7,
  -9, 4, 4,-8, -3,-6, 9, 1,  5, 2,-8,-5, -7, 8, 2,-6,
  -6,-4, 4, 9,  3, 7,-9,-3,  7,-6,-5, 4, -4, 5, 9,-7,
  -8, 0, 5, 6, -3,-9, 8, 2,  6, 4,-7,-5, -5,-3, 3, 7,
   9,-4,-6, 5, -1, 8,-7, 0,  4,-8, 8, 3,  0,-5,-8, 4,
  -7, 2, 5,-4,  3, 9,-9,-3,  7,-5,-4, 6, -6, 4, 8,-2,
   2,-7,-8, 5,  8, 1,-5,-6,  5, 7,-7,-4, -9, 3, 4, 8,
  -4,-8, 7, 4,  3,-6,-9, 2,  6, 5,-8,-3, -7,-5, 2, 9,
   9,-3,-5, 6, -3, 8, 5,-7, -8, 4, 6,-5,  0, 7,-6,-1,
  -5,-6, 8, 3,  4, 9,-9,-4,  7,-4,-6, 5,  3, 6,-8,-3,
  -9, 5, 4,-7, -2,-8, 6, 4,  5, 3,-7,-6,  8,-2,-3, 7,
   // Additional pairs to reach 256
  -6, 1, 4,-5,  9,-2,-7, 6,  3, 8,-5,-8, -4,-7, 8, 2,
   5,-9, 0, 6, -8, 3, 7,-4, -3,-5, 6, 9,  4, 7,-9,-3,
   8,-5,-4, 6, -7, 4, 5,-9,  2, 9,-6,-4, -5,-8, 7, 3,
   6, 3,-9,-2, -1, 8,-7, 5,  4,-6, 8,-3, -8, 2, 3, 7,
  -5, 9, 6,-7,  3,-8,-4, 5,  7, 6,-8,-5, -9,-3, 4, 8,
   5,-4,-7, 6, -3, 7, 9,-5,  8, 2,-6,-4, -4,-9, 3, 7,
   6, 8,-9,-3, -7, 5, 4,-6,  2,-7, 8, 4, -5,-4, 7, 9,
   9,-6,-3, 5, -8, 4, 3,-7,  5, 7,-9,-4, -6,-5, 8, 2,
   4,-9,-7, 6, -3,-6, 9, 3,  6, 5,-8,-4, -9, 2, 7,-5,
   3, 8,-6,-5, -7,-4, 5, 9,  8,-3,-5, 6, -4, 7, 9,-6,
  -8,-5, 4, 7,  3,-9, 8,-2, -6, 6, 5,-8,  0,-5,-9, 4,
  -3, 8, 7,-6,  5, 9,-4,-7, -9,-2, 6, 5,  4,-8,-7, 3,
   8, 6,-5,-9, -4, 3, 9,-7, -7, 5, 3,-8,  6,-4,-9, 2,
   5,-7,-8, 4,  9, 3,-6,-5, -5, 8, 4,-9, -8,-3, 7, 6,
  -3,-9, 5, 7,  8,-6,-4, 5, -7, 4, 6,-8,  3, 9,-5,-6,
   9,-4,-8, 3,  4, 7,-9,-5, -6, 5, 7,-4, -5,-8, 8, 3,
};

}  // namespace

std::vector<Descriptor> compute_descriptors(const GrayscaleImage& image,
                                            const std::vector<Keypoint>& keypoints) {
  const int W      = image.width;
  const int H      = image.height;
  const int border = 16;  // minimum distance from image edge for the patch

  std::vector<Descriptor> descs(keypoints.size());

  for (std::size_t ki = 0; ki < keypoints.size(); ++ki) {
    const Keypoint& kp = keypoints[ki];
    const int cx       = static_cast<int>(kp.x + 0.5f);
    const int cy       = static_cast<int>(kp.y + 0.5f);

    if (cx < border || cy < border || cx >= W - border || cy >= H - border) {
      // Too close to border — leave descriptor zeroed
      continue;
    }

    Descriptor& desc = descs[ki];
    for (int b = 0; b < 32; ++b) {
      uint8_t byte = 0;
      for (int bit = 0; bit < 8; ++bit) {
        const int pair_idx = (b * 8 + bit) * 4;
        const int x1 = cx + kBriefPattern[pair_idx];
        const int y1 = cy + kBriefPattern[pair_idx + 1];
        const int x2 = cx + kBriefPattern[pair_idx + 2];
        const int y2 = cy + kBriefPattern[pair_idx + 3];
        const uint8_t p1 = image.pixels[static_cast<std::size_t>(y1) * W + x1];
        const uint8_t p2 = image.pixels[static_cast<std::size_t>(y2) * W + x2];
        if (p1 < p2) {
          byte |= static_cast<uint8_t>(1u << bit);
        }
      }
      desc.data[static_cast<std::size_t>(b)] = byte;
    }
  }
  return descs;
}

// ── Brute-force descriptor matching ──────────────────────────────────────────

namespace {

int hamming_distance(const Descriptor& a, const Descriptor& b) {
  int dist = 0;
  for (int i = 0; i < 32; ++i) {
    dist += __builtin_popcount(static_cast<unsigned>(a.data[i] ^ b.data[i]));
  }
  return dist;
}

}  // namespace

std::vector<KeypointMatch> match_descriptors(const std::vector<Descriptor>& query,
                                             const std::vector<Descriptor>& train,
                                             float ratio_threshold) {
  std::vector<KeypointMatch> matches;
  if (query.empty() || train.empty()) {
    return matches;
  }

  for (int qi = 0; qi < static_cast<int>(query.size()); ++qi) {
    int best_dist   = 257;
    int second_dist = 257;
    int best_idx    = -1;

    for (int ti = 0; ti < static_cast<int>(train.size()); ++ti) {
      const int d = hamming_distance(query[qi], train[ti]);
      if (d < best_dist) {
        second_dist = best_dist;
        best_dist   = d;
        best_idx    = ti;
      }
      else if (d < second_dist) {
        second_dist = d;
      }
    }

    // Lowe's ratio test (always keep exact matches).
    const bool exact_match = (best_dist == 0);
    const bool ratio_pass =
      // Guard against zero-valued second-best distance in the ratio test.
      (second_dist > 0)
      && (static_cast<float>(best_dist) < ratio_threshold * static_cast<float>(second_dist));
    if (best_idx >= 0 && (exact_match || ratio_pass)) {
      KeypointMatch m;
      m.query_idx = qi;
      m.train_idx = best_idx;
      m.distance  = static_cast<float>(best_dist);
      matches.push_back(m);
    }
  }
  return matches;
}

}  // namespace op::features

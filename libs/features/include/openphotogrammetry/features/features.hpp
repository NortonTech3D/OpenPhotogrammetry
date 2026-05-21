#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "openphotogrammetry/camera/camera.hpp"
#include "openphotogrammetry/core/core.hpp"

namespace op::features {

// ── Existing API (retained for backward compatibility) ────────────────────────

int  recommended_feature_count();
bool has_viable_feature_budget();

// ── Image types ───────────────────────────────────────────────────────────────

// 8-bit grayscale image stored row-major (pixel = pixels[y * width + x]).
struct GrayscaleImage {
  int                    width{0};
  int                    height{0};
  std::vector<uint8_t>   pixels;
};

// 8-bit RGB image stored row-major (3 bytes per pixel: R, G, B).
struct RgbImage {
  int                    width{0};
  int                    height{0};
  std::vector<uint8_t>   pixels;
};

// ── Feature types ─────────────────────────────────────────────────────────────

struct Keypoint {
  float x{0};         // sub-pixel column
  float y{0};         // sub-pixel row
  float scale{1};     // detection scale (sigma)
  float angle{0};     // orientation in radians
  float response{0};  // detector strength (larger = stronger)
};

// 256-bit binary descriptor (BRIEF-style, 32 bytes).
struct Descriptor {
  std::array<uint8_t, 32> data{};
};

struct KeypointMatch {
  int   query_idx{-1};
  int   train_idx{-1};
  float distance{0};  // Hamming distance in [0, 256]
};

// ── Image I/O (Netpbm formats, no external dependencies) ─────────────────────

// Load a PGM (P5) or PPM (P6) file as a grayscale image.
// PPM inputs are converted to grayscale via luminosity weighting.
bool load_image(std::string_view path, GrayscaleImage* out, std::string* error = nullptr);

// Load a PPM (P6) file as an RGB image.
bool load_image_rgb(std::string_view path, RgbImage* out, std::string* error = nullptr);

// Save a grayscale image as a PGM (P5) file.
bool save_image(std::string_view path, const GrayscaleImage& img, std::string* error = nullptr);

// ── Feature detection and description ────────────────────────────────────────

// Detect Harris corners in an image.
// Returns up to max_features keypoints sorted by descending response.
std::vector<Keypoint> detect_keypoints(const GrayscaleImage& image, int max_features = 2048);

// Compute BRIEF-style binary descriptors for the given keypoints.
// Descriptors for keypoints that are too close to the border will be zeroed.
std::vector<Descriptor> compute_descriptors(const GrayscaleImage& image,
                                            const std::vector<Keypoint>& keypoints);

// ── Feature matching ──────────────────────────────────────────────────────────

// Brute-force Hamming-distance matcher with Lowe's ratio test.
// ratio_threshold: reject matches where dist_1 / dist_2 >= ratio_threshold.
std::vector<KeypointMatch> match_descriptors(const std::vector<Descriptor>& query,
                                             const std::vector<Descriptor>& train,
                                             float ratio_threshold = 0.75f);

}  // namespace op::features

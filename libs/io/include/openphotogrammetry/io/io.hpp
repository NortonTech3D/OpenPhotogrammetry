#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace op::io {

bool can_read_project_manifest(std::string_view path);

inline constexpr int kCurrentManifestVersion = 1;

enum class ManifestType {
  dataset_metadata,
  camera_models,
  calibration_state,
  sparse_model,
  dense_model,
  mesh,
  texture,
  qa_report,
};

bool is_supported_manifest_version(int version);
std::string_view manifest_type_name(ManifestType type);
bool is_supported_manifest_type(std::string_view type);

struct DatasetMetadataManifest {
  int version{kCurrentManifestVersion};
  std::string dataset_id;
  int image_count{0};
  std::string coordinate_system;
};

struct CameraModelsManifest {
  int version{kCurrentManifestVersion};
  std::string dataset_id;
  int camera_count{0};
  bool has_distortion{false};
};

struct CalibrationStateManifest {
  int version{kCurrentManifestVersion};
  std::string dataset_id;
  int calibrated_views{0};
  double rms_reprojection_error{0.0};
};

struct SparseModelManifest {
  int version{kCurrentManifestVersion};
  std::string run_id;
  int registered_views{0};
  int sparse_points{0};
};

struct DenseModelManifest {
  int version{kCurrentManifestVersion};
  std::string run_id;
  int depth_maps{0};
  int fused_points{0};
};

struct MeshManifest {
  int version{kCurrentManifestVersion};
  std::string run_id;
  int vertex_count{0};
  int face_count{0};
};

struct TextureManifest {
  int version{kCurrentManifestVersion};
  std::string run_id;
  int texture_count{0};
  int atlas_resolution{0};
};

struct QaReportManifest {
  int version{kCurrentManifestVersion};
  std::string run_id;
  double completeness{0.0};
  double reprojection_quality{0.0};
  bool passed{false};
};

bool validate_manifest(const DatasetMetadataManifest& manifest, std::string* error = nullptr);
bool validate_manifest(const CameraModelsManifest& manifest, std::string* error = nullptr);
bool validate_manifest(const CalibrationStateManifest& manifest, std::string* error = nullptr);
bool validate_manifest(const SparseModelManifest& manifest, std::string* error = nullptr);
bool validate_manifest(const DenseModelManifest& manifest, std::string* error = nullptr);
bool validate_manifest(const MeshManifest& manifest, std::string* error = nullptr);
bool validate_manifest(const TextureManifest& manifest, std::string* error = nullptr);
bool validate_manifest(const QaReportManifest& manifest, std::string* error = nullptr);

struct ProfileContract {
  std::string name;
  std::vector<std::string> compute_backend_priority;
  std::string max_parallel_jobs;
  int image_cache_gb{0};
  std::string dense_reconstruction_quality;
  bool fail_fast_quality_gates{true};
  std::string sparse_matching_strategy;
  std::string dense_confidence_filtering;
  std::string qa_report_level;
  bool unified_memory_mode{false};
};

bool validate_profile_contract_json(std::string_view json, std::string* error = nullptr);
bool load_profile_contract(std::string_view path, ProfileContract* profile, std::string* error = nullptr);
bool profile_contract_defaults(std::string_view profile_name, ProfileContract* profile, std::string* error = nullptr);

enum class PipelineStage {
  intake,
  quality_screening,
  capture_adequacy,
  sparse,
  sparse_qa,
  dense,
  dense_qa,
  meshing_texturing,
  packaging_export,
};

enum class FailureClassification {
  none,
  validation,
  data_quality,
  numerical,
  resource_exhaustion,
  internal_error,
};

std::string_view pipeline_stage_name(PipelineStage stage);
bool parse_pipeline_stage(std::string_view stage_name, PipelineStage* stage);
std::vector<PipelineStage> reconstruction_stage_graph();

struct StageObservation {
  bool success{false};
  FailureClassification failure{FailureClassification::none};
  int runtime_ms{0};
  std::size_t peak_memory_mb{0};
  double qa_metric{0.0};
  std::string artifact_uri;
  std::string details;
};

struct StageCheckpoint {
  PipelineStage stage{PipelineStage::intake};
  bool completed{false};
  int attempts{0};
  FailureClassification failure{FailureClassification::none};
  int runtime_ms{0};
  std::size_t peak_memory_mb{0};
  double qa_metric{0.0};
  std::string artifact_uri;
  std::string details;
  std::uint64_t artifact_checksum{0};
};

struct StageExecutionOptions {
  std::string project_name;
  std::string dataset_id;
  std::string profile_name;
  std::string resume_from_stage;
  int max_retries{1};
  bool fail_fast_quality_gates{true};
  std::size_t memory_budget_mb{8192};
  int runtime_budget_ms{300000};
};

using StageExecutor = std::function<StageObservation(PipelineStage stage, int attempt)>;

struct StageExecutionResult {
  std::string run_id;
  bool success{false};
  std::vector<StageCheckpoint> checkpoints;
  FailureClassification terminal_failure{FailureClassification::none};
  std::string failed_stage;
};

std::uint64_t fnv1a_checksum(std::string_view value);
std::string deterministic_run_id(std::string_view project_name, std::string_view dataset_id, std::string_view profile_name);

StageExecutionResult execute_reconstruction_stage_graph(const StageExecutionOptions& options, const StageExecutor& executor);
bool write_stage_execution_manifest(std::string_view path, const StageExecutionResult& result, std::string* error = nullptr);

}  // namespace op::io

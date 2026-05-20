#include "openphotogrammetry/io/io.hpp"

#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include "openphotogrammetry/core/core.hpp"

namespace {

void set_error(std::string* error, std::string_view message) {
  if (error != nullptr) {
    *error = std::string(message);
  }
}

bool contains_required_key(std::string_view json, std::string_view key) {
  const std::string token = std::string("\"") + std::string(key) + "\"";
  return json.find(token) != std::string_view::npos;
}

bool extract_json_string(std::string_view json, std::string_view key, std::string* out) {
  const std::string token = std::string("\"") + std::string(key) + "\"";
  const std::size_t key_pos = json.find(token);
  if (key_pos == std::string_view::npos) {
    return false;
  }

  const std::size_t colon_pos = json.find(':', key_pos + token.size());
  if (colon_pos == std::string_view::npos) {
    return false;
  }

  const std::size_t first_quote = json.find('"', colon_pos + 1);
  if (first_quote == std::string_view::npos) {
    return false;
  }

  const std::size_t second_quote = json.find('"', first_quote + 1);
  if (second_quote == std::string_view::npos || second_quote <= first_quote) {
    return false;
  }

  if (out != nullptr) {
    *out = std::string(json.substr(first_quote + 1, second_quote - first_quote - 1));
  }

  return true;
}

bool extract_json_int(std::string_view json, std::string_view key, int* out) {
  const std::string token = std::string("\"") + std::string(key) + "\"";
  const std::size_t key_pos = json.find(token);
  if (key_pos == std::string_view::npos) {
    return false;
  }

  const std::size_t colon_pos = json.find(':', key_pos + token.size());
  if (colon_pos == std::string_view::npos) {
    return false;
  }

  std::size_t value_start = colon_pos + 1;
  while (value_start < json.size() && (json[value_start] == ' ' || json[value_start] == '\t')) {
    ++value_start;
  }

  std::size_t value_end = value_start;
  while (value_end < json.size() && ((json[value_end] >= '0' && json[value_end] <= '9') || json[value_end] == '-')) {
    ++value_end;
  }

  if (value_end == value_start) {
    return false;
  }

  try {
    const int value = std::stoi(std::string(json.substr(value_start, value_end - value_start)));
    if (out != nullptr) {
      *out = value;
    }
  }
  catch (...) {
    return false;
  }

  return true;
}

bool extract_json_bool(std::string_view json, std::string_view key, bool* out) {
  const std::string token = std::string("\"") + std::string(key) + "\"";
  const std::size_t key_pos = json.find(token);
  if (key_pos == std::string_view::npos) {
    return false;
  }

  const std::size_t colon_pos = json.find(':', key_pos + token.size());
  if (colon_pos == std::string_view::npos) {
    return false;
  }

  std::size_t value_start = colon_pos + 1;
  while (value_start < json.size() && (json[value_start] == ' ' || json[value_start] == '\t')) {
    ++value_start;
  }

  if (json.substr(value_start, 4) == "true") {
    if (out != nullptr) {
      *out = true;
    }
    return true;
  }

  if (json.substr(value_start, 5) == "false") {
    if (out != nullptr) {
      *out = false;
    }
    return true;
  }

  return false;
}

bool extract_json_string_array(std::string_view json, std::string_view key, std::vector<std::string>* out) {
  const std::string token = std::string("\"") + std::string(key) + "\"";
  const std::size_t key_pos = json.find(token);
  if (key_pos == std::string_view::npos) {
    return false;
  }

  const std::size_t colon_pos = json.find(':', key_pos + token.size());
  if (colon_pos == std::string_view::npos) {
    return false;
  }

  const std::size_t array_start = json.find('[', colon_pos + 1);
  const std::size_t array_end = json.find(']', array_start + 1);
  if (array_start == std::string_view::npos || array_end == std::string_view::npos || array_end <= array_start) {
    return false;
  }

  std::vector<std::string> values;
  std::size_t cursor = array_start;
  while (cursor < array_end) {
    const std::size_t first_quote = json.find('"', cursor);
    if (first_quote == std::string_view::npos || first_quote >= array_end) {
      break;
    }

    const std::size_t second_quote = json.find('"', first_quote + 1);
    if (second_quote == std::string_view::npos || second_quote >= array_end) {
      return false;
    }

    values.emplace_back(json.substr(first_quote + 1, second_quote - first_quote - 1));
    cursor = second_quote + 1;
  }

  if (values.empty()) {
    return false;
  }

  if (out != nullptr) {
    *out = std::move(values);
  }

  return true;
}

bool value_in(std::string_view value, std::initializer_list<std::string_view> allowed) {
  for (const std::string_view candidate : allowed) {
    if (value == candidate) {
      return true;
    }
  }
  return false;
}

std::string_view failure_name(op::io::FailureClassification failure) {
  switch (failure) {
    case op::io::FailureClassification::none:
      return "none";
    case op::io::FailureClassification::validation:
      return "validation";
    case op::io::FailureClassification::data_quality:
      return "data_quality";
    case op::io::FailureClassification::numerical:
      return "numerical";
    case op::io::FailureClassification::resource_exhaustion:
      return "resource_exhaustion";
    case op::io::FailureClassification::internal_error:
      return "internal_error";
  }

  return "internal_error";
}

}  // namespace

namespace op::io {

bool can_read_project_manifest(std::string_view path) {
  return !path.empty() && op::core::is_runtime_ready();
}

bool is_supported_manifest_version(int version) {
  return version == kCurrentManifestVersion;
}

std::string_view manifest_type_name(ManifestType type) {
  switch (type) {
    case ManifestType::dataset_metadata:
      return "dataset_metadata";
    case ManifestType::camera_models:
      return "camera_models";
    case ManifestType::calibration_state:
      return "calibration_state";
    case ManifestType::sparse_model:
      return "sparse_model";
    case ManifestType::dense_model:
      return "dense_model";
    case ManifestType::mesh:
      return "mesh";
    case ManifestType::texture:
      return "texture";
    case ManifestType::qa_report:
      return "qa_report";
  }

  return "unknown";
}

bool is_supported_manifest_type(std::string_view type) {
  return value_in(type,
                  {"dataset_metadata",
                   "camera_models",
                   "calibration_state",
                   "sparse_model",
                   "dense_model",
                   "mesh",
                   "texture",
                   "qa_report"});
}

bool validate_manifest(const DatasetMetadataManifest& manifest, std::string* error) {
  if (!is_supported_manifest_version(manifest.version)) {
    set_error(error, "dataset metadata manifest version is unsupported");
    return false;
  }

  if (manifest.dataset_id.empty() || manifest.image_count <= 0 || manifest.coordinate_system.empty()) {
    set_error(error, "dataset metadata manifest is missing required fields");
    return false;
  }

  return true;
}

bool validate_manifest(const CameraModelsManifest& manifest, std::string* error) {
  if (!is_supported_manifest_version(manifest.version)) {
    set_error(error, "camera models manifest version is unsupported");
    return false;
  }

  if (manifest.dataset_id.empty() || manifest.camera_count <= 0) {
    set_error(error, "camera models manifest is missing required fields");
    return false;
  }

  return true;
}

bool validate_manifest(const CalibrationStateManifest& manifest, std::string* error) {
  if (!is_supported_manifest_version(manifest.version)) {
    set_error(error, "calibration state manifest version is unsupported");
    return false;
  }

  if (manifest.dataset_id.empty() || manifest.calibrated_views < 0 || manifest.rms_reprojection_error < 0.0) {
    set_error(error, "calibration state manifest is invalid");
    return false;
  }

  return true;
}

bool validate_manifest(const SparseModelManifest& manifest, std::string* error) {
  if (!is_supported_manifest_version(manifest.version)) {
    set_error(error, "sparse model manifest version is unsupported");
    return false;
  }

  if (manifest.run_id.empty() || manifest.registered_views < 2 || manifest.sparse_points <= 0) {
    set_error(error, "sparse model manifest is invalid");
    return false;
  }

  return true;
}

bool validate_manifest(const DenseModelManifest& manifest, std::string* error) {
  if (!is_supported_manifest_version(manifest.version)) {
    set_error(error, "dense model manifest version is unsupported");
    return false;
  }

  if (manifest.run_id.empty() || manifest.depth_maps <= 0 || manifest.fused_points <= 0) {
    set_error(error, "dense model manifest is invalid");
    return false;
  }

  return true;
}

bool validate_manifest(const MeshManifest& manifest, std::string* error) {
  if (!is_supported_manifest_version(manifest.version)) {
    set_error(error, "mesh manifest version is unsupported");
    return false;
  }

  if (manifest.run_id.empty() || manifest.vertex_count <= 0 || manifest.face_count <= 0) {
    set_error(error, "mesh manifest is invalid");
    return false;
  }

  return true;
}

bool validate_manifest(const TextureManifest& manifest, std::string* error) {
  if (!is_supported_manifest_version(manifest.version)) {
    set_error(error, "texture manifest version is unsupported");
    return false;
  }

  if (manifest.run_id.empty() || manifest.texture_count <= 0 || manifest.atlas_resolution < 1024) {
    set_error(error, "texture manifest is invalid");
    return false;
  }

  return true;
}

bool validate_manifest(const QaReportManifest& manifest, std::string* error) {
  if (!is_supported_manifest_version(manifest.version)) {
    set_error(error, "qa report manifest version is unsupported");
    return false;
  }

  const bool within_bounds = manifest.completeness >= 0.0 && manifest.completeness <= 1.0
                             && manifest.reprojection_quality >= 0.0 && manifest.reprojection_quality <= 1.0;
  if (manifest.run_id.empty() || !within_bounds) {
    set_error(error, "qa report manifest is invalid");
    return false;
  }

  return true;
}

bool validate_profile_contract_json(std::string_view json, std::string* error) {
  for (std::string_view key : {"name",
                               "compute_backend_priority",
                               "max_parallel_jobs",
                               "image_cache_gb",
                               "dense_reconstruction_quality",
                               "fail_fast_quality_gates",
                               "sparse_matching_strategy",
                               "dense_confidence_filtering",
                               "qa_report_level"}) {
    if (!contains_required_key(json, key)) {
      set_error(error, "profile contract is missing required key");
      return false;
    }
  }

  std::string name;
  std::vector<std::string> backend_priority;
  std::string max_parallel_jobs;
  int image_cache_gb = 0;
  std::string dense_quality;
  bool fail_fast = false;
  std::string sparse_matching;
  std::string dense_filtering;
  std::string qa_report_level;

  if (!extract_json_string(json, "name", &name) || name.empty()) {
    set_error(error, "profile name is invalid");
    return false;
  }

  if (!extract_json_string_array(json, "compute_backend_priority", &backend_priority) || backend_priority.empty()) {
    set_error(error, "compute backend priority is invalid");
    return false;
  }

  if (!extract_json_string(json, "max_parallel_jobs", &max_parallel_jobs)
      || !value_in(max_parallel_jobs, {"auto", "max"})) {
    set_error(error, "max_parallel_jobs is invalid");
    return false;
  }

  if (!extract_json_int(json, "image_cache_gb", &image_cache_gb) || image_cache_gb <= 0) {
    set_error(error, "image_cache_gb is invalid");
    return false;
  }

  if (!extract_json_string(json, "dense_reconstruction_quality", &dense_quality)
      || !value_in(dense_quality, {"balanced", "high", "ultra"})) {
    set_error(error, "dense_reconstruction_quality is invalid");
    return false;
  }

  if (!extract_json_bool(json, "fail_fast_quality_gates", &fail_fast)) {
    set_error(error, "fail_fast_quality_gates is invalid");
    return false;
  }

  if (!extract_json_string(json, "sparse_matching_strategy", &sparse_matching)
      || !value_in(sparse_matching, {"adaptive", "exhaustive", "sequential"})) {
    set_error(error, "sparse_matching_strategy is invalid");
    return false;
  }

  if (!extract_json_string(json, "dense_confidence_filtering", &dense_filtering)
      || !value_in(dense_filtering, {"disabled", "enabled", "aggressive"})) {
    set_error(error, "dense_confidence_filtering is invalid");
    return false;
  }

  if (!extract_json_string(json, "qa_report_level", &qa_report_level)
      || !value_in(qa_report_level, {"standard", "detailed", "forensic"})) {
    set_error(error, "qa_report_level is invalid");
    return false;
  }

  if (contains_required_key(json, "unified_memory_mode")) {
    bool unified_memory_mode = false;
    if (!extract_json_bool(json, "unified_memory_mode", &unified_memory_mode)) {
      set_error(error, "unified_memory_mode must be a boolean");
      return false;
    }
  }

  return true;
}

bool load_profile_contract(std::string_view path, ProfileContract* profile, std::string* error) {
  if (path.empty()) {
    set_error(error, "profile path cannot be empty");
    return false;
  }

  std::ifstream in{std::string(path)};
  if (!in) {
    set_error(error, "unable to open profile file");
    return false;
  }

  std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (!validate_profile_contract_json(content, error)) {
    return false;
  }

  if (profile == nullptr) {
    return true;
  }

  extract_json_string(content, "name", &profile->name);
  extract_json_string_array(content, "compute_backend_priority", &profile->compute_backend_priority);
  extract_json_string(content, "max_parallel_jobs", &profile->max_parallel_jobs);
  extract_json_int(content, "image_cache_gb", &profile->image_cache_gb);
  extract_json_string(content, "dense_reconstruction_quality", &profile->dense_reconstruction_quality);
  extract_json_bool(content, "fail_fast_quality_gates", &profile->fail_fast_quality_gates);
  extract_json_string(content, "sparse_matching_strategy", &profile->sparse_matching_strategy);
  extract_json_string(content, "dense_confidence_filtering", &profile->dense_confidence_filtering);
  extract_json_string(content, "qa_report_level", &profile->qa_report_level);

  bool unified_memory_mode = false;
  if (extract_json_bool(content, "unified_memory_mode", &unified_memory_mode)) {
    profile->unified_memory_mode = unified_memory_mode;
  }

  return true;
}

bool profile_contract_defaults(std::string_view profile_name, ProfileContract* profile, std::string* error) {
  if (profile == nullptr) {
    set_error(error, "profile output cannot be null");
    return false;
  }

  if (profile_name == "desktop") {
    *profile = ProfileContract{
      .name = "desktop",
      .compute_backend_priority = {"cuda", "metal", "cpu"},
      .max_parallel_jobs = "auto",
      .image_cache_gb = 8,
      .dense_reconstruction_quality = "high",
      .fail_fast_quality_gates = true,
      .sparse_matching_strategy = "adaptive",
      .dense_confidence_filtering = "enabled",
      .qa_report_level = "standard",
      .unified_memory_mode = false,
    };
    return true;
  }

  if (profile_name == "server") {
    *profile = ProfileContract{
      .name = "server",
      .compute_backend_priority = {"cuda", "cpu"},
      .max_parallel_jobs = "max",
      .image_cache_gb = 32,
      .dense_reconstruction_quality = "high",
      .fail_fast_quality_gates = true,
      .sparse_matching_strategy = "adaptive",
      .dense_confidence_filtering = "aggressive",
      .qa_report_level = "detailed",
      .unified_memory_mode = false,
    };
    return true;
  }

  if (profile_name == "apple-silicon") {
    *profile = ProfileContract{
      .name = "apple-silicon",
      .compute_backend_priority = {"metal", "cpu"},
      .max_parallel_jobs = "auto",
      .image_cache_gb = 6,
      .dense_reconstruction_quality = "balanced",
      .fail_fast_quality_gates = true,
      .sparse_matching_strategy = "adaptive",
      .dense_confidence_filtering = "enabled",
      .qa_report_level = "standard",
      .unified_memory_mode = true,
    };
    return true;
  }

  set_error(error, "unsupported profile name");
  return false;
}

std::string_view pipeline_stage_name(PipelineStage stage) {
  switch (stage) {
    case PipelineStage::intake:
      return "intake";
    case PipelineStage::quality_screening:
      return "quality_screening";
    case PipelineStage::capture_adequacy:
      return "capture_adequacy";
    case PipelineStage::sparse:
      return "sparse";
    case PipelineStage::sparse_qa:
      return "sparse_qa";
    case PipelineStage::dense:
      return "dense";
    case PipelineStage::dense_qa:
      return "dense_qa";
    case PipelineStage::meshing_texturing:
      return "meshing_texturing";
    case PipelineStage::packaging_export:
      return "packaging_export";
  }

  return "unknown";
}

bool parse_pipeline_stage(std::string_view stage_name, PipelineStage* stage) {
  if (stage == nullptr) {
    return false;
  }

  const std::vector<PipelineStage> stages = reconstruction_stage_graph();
  for (const PipelineStage candidate : stages) {
    if (pipeline_stage_name(candidate) == stage_name) {
      *stage = candidate;
      return true;
    }
  }

  return false;
}

std::vector<PipelineStage> reconstruction_stage_graph() {
  return {
    PipelineStage::intake,
    PipelineStage::quality_screening,
    PipelineStage::capture_adequacy,
    PipelineStage::sparse,
    PipelineStage::sparse_qa,
    PipelineStage::dense,
    PipelineStage::dense_qa,
    PipelineStage::meshing_texturing,
    PipelineStage::packaging_export,
  };
}

std::uint64_t fnv1a_checksum(std::string_view value) {
  std::uint64_t hash = 14695981039346656037ULL;
  for (const unsigned char c : value) {
    hash ^= static_cast<std::uint64_t>(c);
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::string deterministic_run_id(std::string_view project_name,
                                 std::string_view dataset_id,
                                 std::string_view profile_name) {
  const std::string seed = std::string(project_name) + "|" + std::string(dataset_id) + "|" + std::string(profile_name);
  const std::uint64_t hash = fnv1a_checksum(seed);
  std::ostringstream out;
  out << "run-" << std::hex << std::nouppercase << std::setw(16) << std::setfill('0') << hash;
  return out.str();
}

StageExecutionResult execute_reconstruction_stage_graph(const StageExecutionOptions& options, const StageExecutor& executor) {
  StageExecutionResult result;
  result.run_id = deterministic_run_id(options.project_name, options.dataset_id, options.profile_name);

  if (!executor) {
    result.terminal_failure = FailureClassification::internal_error;
    result.failed_stage = "executor_not_configured";
    return result;
  }

  int max_retries = options.max_retries;
  if (max_retries < 1) {
    max_retries = 1;
  }

  const std::vector<PipelineStage> graph = reconstruction_stage_graph();
  std::size_t start_index = 0;

  if (!options.resume_from_stage.empty()) {
    PipelineStage resume_stage = PipelineStage::intake;
    if (!parse_pipeline_stage(options.resume_from_stage, &resume_stage)) {
      result.terminal_failure = FailureClassification::validation;
      result.failed_stage = std::string(options.resume_from_stage);
      return result;
    }

    for (std::size_t i = 0; i < graph.size(); ++i) {
      if (graph[i] == resume_stage) {
        start_index = i;
        break;
      }
    }
  }

  for (std::size_t i = 0; i < start_index; ++i) {
    StageCheckpoint checkpoint;
    checkpoint.stage = graph[i];
    checkpoint.completed = true;
    checkpoint.attempts = 0;
    checkpoint.failure = FailureClassification::none;
    checkpoint.artifact_uri = "checkpoint://resumed/" + std::string(pipeline_stage_name(graph[i]));
    checkpoint.details = "reused from previous successful run";
    checkpoint.artifact_checksum = fnv1a_checksum(checkpoint.artifact_uri);
    result.checkpoints.push_back(std::move(checkpoint));
  }

  for (std::size_t i = start_index; i < graph.size(); ++i) {
    const PipelineStage stage = graph[i];
    StageCheckpoint checkpoint;
    checkpoint.stage = stage;

    for (int attempt = 1; attempt <= max_retries; ++attempt) {
      StageObservation observation = executor(stage, attempt);
      checkpoint.attempts = attempt;
      checkpoint.runtime_ms = observation.runtime_ms;
      checkpoint.peak_memory_mb = observation.peak_memory_mb;
      checkpoint.qa_metric = observation.qa_metric;
      checkpoint.artifact_uri = observation.artifact_uri;
      checkpoint.details = observation.details;
      checkpoint.artifact_checksum = fnv1a_checksum(checkpoint.artifact_uri);

      bool success = observation.success;
      FailureClassification failure = observation.failure;

      if (observation.runtime_ms > options.runtime_budget_ms || observation.peak_memory_mb > options.memory_budget_mb) {
        success = false;
        failure = FailureClassification::resource_exhaustion;
      }

      if ((stage == PipelineStage::sparse_qa || stage == PipelineStage::dense_qa) && observation.qa_metric < 0.75) {
        success = false;
        if (failure == FailureClassification::none) {
          failure = FailureClassification::data_quality;
        }
      }

      if (success) {
        checkpoint.completed = true;
        checkpoint.failure = FailureClassification::none;
        break;
      }

      checkpoint.completed = false;
      checkpoint.failure = failure == FailureClassification::none ? FailureClassification::internal_error : failure;

      if (attempt == max_retries) {
        break;
      }
    }

    result.checkpoints.push_back(checkpoint);

    if (!checkpoint.completed) {
      result.terminal_failure = checkpoint.failure;
      result.failed_stage = std::string(pipeline_stage_name(stage));
      result.success = false;
      if (options.fail_fast_quality_gates || stage == PipelineStage::sparse_qa || stage == PipelineStage::dense_qa) {
        return result;
      }
    }
  }

  result.success = true;
  result.terminal_failure = FailureClassification::none;
  result.failed_stage.clear();
  return result;
}

bool write_stage_execution_manifest(std::string_view path, const StageExecutionResult& result, std::string* error) {
  if (path.empty()) {
    set_error(error, "manifest output path is empty");
    return false;
  }

  std::ostringstream json;
  json << "{\n";
  json << "  \"schema\": \"op-run-manifest-v1\",\n";
  json << "  \"run_id\": \"" << result.run_id << "\",\n";
  json << "  \"success\": " << (result.success ? "true" : "false") << ",\n";
  json << "  \"terminal_failure\": \"" << failure_name(result.terminal_failure) << "\",\n";
  json << "  \"failed_stage\": \"" << result.failed_stage << "\",\n";
  json << "  \"checkpoints\": [\n";

  for (std::size_t i = 0; i < result.checkpoints.size(); ++i) {
    const StageCheckpoint& checkpoint = result.checkpoints[i];
    json << "    {\n";
    json << "      \"stage\": \"" << pipeline_stage_name(checkpoint.stage) << "\",\n";
    json << "      \"completed\": " << (checkpoint.completed ? "true" : "false") << ",\n";
    json << "      \"attempts\": " << checkpoint.attempts << ",\n";
    json << "      \"failure\": \"" << failure_name(checkpoint.failure) << "\",\n";
    json << "      \"runtime_ms\": " << checkpoint.runtime_ms << ",\n";
    json << "      \"peak_memory_mb\": " << checkpoint.peak_memory_mb << ",\n";
    json << "      \"qa_metric\": " << std::fixed << std::setprecision(3) << checkpoint.qa_metric << ",\n";
    json << "      \"artifact_uri\": \"" << checkpoint.artifact_uri << "\",\n";
    json << "      \"artifact_checksum\": \"" << checkpoint.artifact_checksum << "\",\n";
    json << "      \"details\": \"" << checkpoint.details << "\"\n";
    json << "    }";
    if (i + 1 < result.checkpoints.size()) {
      json << ',';
    }
    json << "\n";
  }

  json << "  ]\n";
  json << "}\n";

  const std::filesystem::path output_path(path);
  const std::filesystem::path temp_path = output_path.string() + ".tmp";

  std::ofstream out(temp_path, std::ios::binary | std::ios::trunc);
  if (!out) {
    set_error(error, "unable to open temporary manifest output");
    return false;
  }

  out << json.str();
  out.flush();
  if (!out) {
    set_error(error, "unable to write manifest output");
    return false;
  }

  out.close();
  std::error_code ec;
  std::filesystem::rename(temp_path, output_path, ec);
  if (ec) {
    std::filesystem::remove(output_path, ec);
    ec.clear();
    std::filesystem::rename(temp_path, output_path, ec);
    if (ec) {
      set_error(error, "unable to atomically replace manifest output");
      return false;
    }
  }

  return true;
}

}  // namespace op::io

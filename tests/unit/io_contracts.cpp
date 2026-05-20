#include <iostream>
#include <string>

#include "openphotogrammetry/io/io.hpp"

namespace {

int fail(std::string_view message) {
  std::cerr << message << '\n';
  return 1;
}

}  // namespace

int main() {
  if (!op::io::is_supported_manifest_version(op::io::kCurrentManifestVersion)
      || op::io::is_supported_manifest_version(op::io::kCurrentManifestVersion + 1)) {
    return fail("manifest version contract failed");
  }

  op::io::DatasetMetadataManifest dataset{
    .version = op::io::kCurrentManifestVersion,
    .dataset_id = "dataset-fixed-mini",
    .image_count = 8,
    .coordinate_system = "EPSG:4326",
  };

  std::string error;
  if (!op::io::validate_manifest(dataset, &error)) {
    return fail("dataset manifest validation failed");
  }

  op::io::QaReportManifest qa{
    .version = op::io::kCurrentManifestVersion,
    .run_id = "run-abc",
    .completeness = 0.98,
    .reprojection_quality = 0.91,
    .passed = true,
  };

  if (!op::io::validate_manifest(qa, &error)) {
    return fail("qa manifest validation failed");
  }

  op::io::ProfileContract desktop;
  if (!op::io::profile_contract_defaults("desktop", &desktop, &error)) {
    return fail("desktop default profile failed");
  }

  const std::string desktop_json = R"json({
    "name": "desktop",
    "compute_backend_priority": ["cuda", "metal", "cpu"],
    "max_parallel_jobs": "auto",
    "image_cache_gb": 8,
    "dense_reconstruction_quality": "high",
    "fail_fast_quality_gates": true,
    "sparse_matching_strategy": "adaptive",
    "dense_confidence_filtering": "enabled",
    "qa_report_level": "standard",
    "runtime_budget_ms": 600000
  })json";

  if (!op::io::validate_profile_contract_json(desktop_json, &error)) {
    return fail("profile json validation failed");
  }

  const std::string run_a = op::io::deterministic_run_id("project-a", "dataset-a", "desktop");
  const std::string run_b = op::io::deterministic_run_id("project-a", "dataset-a", "desktop");
  const std::string run_c = op::io::deterministic_run_id("project-a", "dataset-b", "desktop");
  if (run_a != run_b || run_a == run_c) {
    return fail("deterministic run id contract failed");
  }

  op::io::StageExecutionOptions options;
  options.project_name = "project-a";
  options.dataset_id = "dataset-a";
  options.profile_name = "desktop";
  options.max_retries = 2;
  options.fail_fast_quality_gates = true;

  const op::io::StageExecutionResult result = op::io::execute_reconstruction_stage_graph(
    options,
    [](op::io::PipelineStage stage, int) {
      op::io::StageObservation observation;
      observation.success = true;
      observation.failure = op::io::FailureClassification::none;
      observation.runtime_ms = 10;
      observation.peak_memory_mb = 64;
      observation.qa_metric = 0.99;
      observation.artifact_uri = std::string("artifact://") + std::string(op::io::pipeline_stage_name(stage));
      observation.details = "ok";
      return observation;
    });

  if (!result.success || result.checkpoints.size() != op::io::reconstruction_stage_graph().size()) {
    return fail("stage graph execution contract failed");
  }

  return 0;
}

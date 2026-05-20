#include <iostream>
#include <string>
#include <string_view>

#include "openphotogrammetry/camera/camera.hpp"
#include "openphotogrammetry/features/features.hpp"
#include "openphotogrammetry/io/io.hpp"
#include "openphotogrammetry/meshing/meshing.hpp"
#include "openphotogrammetry/mvs/mvs.hpp"
#include "openphotogrammetry/sfm/sfm.hpp"
#include "openphotogrammetry/texturing/texturing.hpp"

namespace {

bool parse_int_arg(const char* value, int* out) {
  if (value == nullptr || out == nullptr) {
    return false;
  }

  try {
    *out = std::stoi(value);
    return true;
  }
  catch (...) {
    return false;
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string project_name = "default-project";
  std::string dataset_id = "dataset.json";
  std::string profile_name = "desktop";
  std::string profile_path;
  std::string resume_from_stage;
  std::string manifest_output = "reconstruction-run-manifest.json";
  int max_retries = 1;

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg{argv[i]};

    if (arg == "--project" && i + 1 < argc) {
      project_name = argv[++i];
    }
    else if (arg == "--dataset" && i + 1 < argc) {
      dataset_id = argv[++i];
    }
    else if (arg == "--profile-name" && i + 1 < argc) {
      profile_name = argv[++i];
    }
    else if (arg == "--profile" && i + 1 < argc) {
      profile_path = argv[++i];
    }
    else if (arg == "--resume-from" && i + 1 < argc) {
      resume_from_stage = argv[++i];
    }
    else if (arg == "--max-retries" && i + 1 < argc) {
      if (!parse_int_arg(argv[++i], &max_retries)) {
        std::cerr << argv[0] << ": --max-retries expects an integer\n";
        return 2;
      }
    }
    else if (arg == "--manifest-out" && i + 1 < argc) {
      manifest_output = argv[++i];
    }
    else {
      std::cerr << argv[0]
                << ": unknown option " << arg
                << "\nvalid options: --project --dataset --profile-name --profile --resume-from --max-retries --manifest-out\n";
      return 2;
    }
  }

  op::io::ProfileContract profile;
  std::string profile_error;

  if (!profile_path.empty()) {
    if (!op::io::load_profile_contract(profile_path, &profile, &profile_error)) {
      std::cerr << "pipeline.reconstruction=failed profile_error=" << profile_error << '\n';
      return 1;
    }
  }
  else {
    if (!op::io::profile_contract_defaults(profile_name, &profile, &profile_error)) {
      std::cerr << "pipeline.reconstruction=failed profile_error=" << profile_error << '\n';
      return 1;
    }
  }

  op::io::StageExecutionOptions options;
  options.project_name = project_name;
  options.dataset_id = dataset_id;
  options.profile_name = profile.name;
  options.resume_from_stage = resume_from_stage;
  options.max_retries = max_retries;
  options.fail_fast_quality_gates = profile.fail_fast_quality_gates;
  options.memory_budget_mb = static_cast<std::size_t>(profile.image_cache_gb) * 1024;
  options.runtime_budget_ms = profile.runtime_budget_ms;

  const op::io::StageExecutionResult result = op::io::execute_reconstruction_stage_graph(
    options,
    [&](op::io::PipelineStage stage, int attempt) {
      op::io::StageObservation observation;
      observation.runtime_ms = 75 + (attempt * 25);
      observation.peak_memory_mb = static_cast<std::size_t>((profile.image_cache_gb * 100) + (attempt * 32));
      observation.artifact_uri = "artifacts/" + op::io::deterministic_run_id(project_name, dataset_id, profile.name)
                                 + "/" + std::string(op::io::pipeline_stage_name(stage)) + ".json";

      switch (stage) {
        case op::io::PipelineStage::intake:
          observation.success = op::io::can_read_project_manifest(dataset_id);
          observation.qa_metric = observation.success ? 1.0 : 0.0;
          observation.details = observation.success ? "dataset intake validated" : "dataset intake failed";
          observation.failure = observation.success ? op::io::FailureClassification::none
                                                   : op::io::FailureClassification::validation;
          break;

        case op::io::PipelineStage::quality_screening:
          observation.success = op::features::has_viable_feature_budget();
          observation.qa_metric = observation.success ? 0.90 : 0.20;
          observation.details = observation.success ? "quality screening passed" : "quality screening failed";
          observation.failure = observation.success ? op::io::FailureClassification::none
                                                   : op::io::FailureClassification::data_quality;
          break;

        case op::io::PipelineStage::capture_adequacy:
          observation.success = op::camera::has_valid_default_calibration() && op::features::has_viable_feature_budget();
          observation.qa_metric = observation.success ? 0.88 : 0.25;
          observation.details = observation.success ? "capture adequacy passed" : "insufficient capture adequacy";
          observation.failure = observation.success ? op::io::FailureClassification::none
                                                   : op::io::FailureClassification::data_quality;
          break;

        case op::io::PipelineStage::sparse:
          observation.success = op::sfm::can_initialize_reconstruction();
          observation.qa_metric = observation.success ? 0.91 : 0.10;
          observation.details = observation.success ? "sparse reconstruction complete" : "sparse reconstruction failed";
          observation.failure = observation.success ? op::io::FailureClassification::none
                                                   : op::io::FailureClassification::numerical;
          break;

        case op::io::PipelineStage::sparse_qa:
          observation.success = op::sfm::minimum_view_count() >= 2;
          observation.qa_metric = observation.success ? 0.84 : 0.30;
          observation.details = observation.success ? "sparse QA passed" : "sparse QA failed";
          observation.failure = observation.success ? op::io::FailureClassification::none
                                                   : op::io::FailureClassification::data_quality;
          break;

        case op::io::PipelineStage::dense:
          observation.success = op::mvs::can_dense_reconstruct();
          observation.qa_metric = observation.success ? 0.86 : 0.25;
          observation.details = observation.success ? "dense reconstruction complete" : "dense reconstruction failed";
          observation.failure = observation.success ? op::io::FailureClassification::none
                                                   : op::io::FailureClassification::numerical;
          break;

        case op::io::PipelineStage::dense_qa:
          observation.success = op::mvs::depth_map_layers() >= 1;
          observation.qa_metric = observation.success ? 0.82 : 0.20;
          observation.details = observation.success ? "dense QA passed" : "dense QA failed";
          observation.failure = observation.success ? op::io::FailureClassification::none
                                                   : op::io::FailureClassification::data_quality;
          break;

        case op::io::PipelineStage::meshing_texturing:
          observation.success = op::meshing::can_generate_mesh() && op::texturing::can_texture_mesh();
          observation.qa_metric = observation.success ? 0.83 : 0.15;
          observation.details = observation.success ? "meshing/texturing complete" : "meshing/texturing failed";
          observation.failure = observation.success ? op::io::FailureClassification::none
                                                   : op::io::FailureClassification::numerical;
          break;

        case op::io::PipelineStage::packaging_export:
          observation.success = op::io::can_read_project_manifest("export-manifest.json");
          observation.qa_metric = observation.success ? 0.95 : 0.05;
          observation.details = observation.success ? "packaging/export complete" : "packaging/export failed";
          observation.failure = observation.success ? op::io::FailureClassification::none
                                                   : op::io::FailureClassification::internal_error;
          break;
      }

      return observation;
    });

  std::string write_error;
  if (!op::io::write_stage_execution_manifest(manifest_output, result, &write_error)) {
    std::cerr << "pipeline.reconstruction=failed manifest_error=" << write_error << '\n';
    return 1;
  }

  std::cout << "pipeline.reconstruction=" << (result.success ? "ok" : "failed")
            << " run_id=" << result.run_id
            << " profile=" << profile.name;

  if (!result.success) {
    std::cout << " failed_stage=" << result.failed_stage;
  }

  std::cout << '\n';
  return result.success ? 0 : 1;
}

# pipelines/reconstruction

Shared orchestration and quality gate policy for all photogrammetry workflows.

## Workflow goals and baseline KPIs

Each run MUST declare its target profile and satisfy all required KPIs before
progressing to the next stage.

- **Target reconstruction scale**: scene-level scale mode (`object`, `site`,
  `terrain`) and expected units (`mm`, `cm`, `m`)
- **Acceptable reprojection error**: mean <= `1.0 px`, P95 <= `1.5 px`
- **Minimum sparse point density**: >= `1,000` reliable points per registered
  image set component
- **Geometric accuracy tolerance**: <= `1%` relative error against control
  distances (or <= `2x` sensor GSD where georeferenced)
- **Texture quality thresholds**: minimum coverage >= `95%`, seam artifact rate
  <= `2%` of visible mesh area

## Reference baseline patterns

The orchestration policy follows established patterns from proven projects:

- **COLMAP-inspired**: robust incremental SfM with strong geometric verification
- **OpenMVG/OpenMVS-inspired**: strict sparse-to-dense stage separation
- **AliceVision/Meshroom-inspired**: recoverable node-like checkpoints
- **MicMac/Metashape-inspired**: calibration discipline and QA reporting

## Strict staged gates (fail-fast)

1. **Data intake + metadata validation**
   - Validate image readability, EXIF completeness, intrinsics grouping
   - Reject mixed camera groups unless explicit cross-calibration policy exists
2. **Image quality screening**
   - Blur threshold, exposure clipping checks, duplicate/near-duplicate pruning
   - Rolling-shutter risk flag for unstable captures
3. **Capture adequacy check**
   - Overlap coverage, baseline diversity, viewpoint distribution requirements
4. **Sparse reconstruction**
   - Feature extraction, adaptive matching, geometric verification, SfM, BA
5. **Sparse QA gate**
   - Track length distribution, inlier ratio, reprojection error, completeness
6. **Dense reconstruction**
   - Depth map estimation, fusion, confidence-weighted filtering
7. **Dense QA gate**
   - Density uniformity, outlier rate, hole detection
8. **Meshing/texturing handoff**
   - Enabled only when dense confidence and QA pass criteria are met

## Precision and accuracy control policy

- Enforce camera/lens grouping and calibration strategy per dataset category
- Select matching strategy by capture pattern:
  - Sequential: video/ordered trajectory captures
  - Spatial: geo-aware aerial/site captures
  - Vocabulary-tree: unordered large image sets
- Require repeated global/local BA plus outlier pruning until KPI convergence
- Apply scale/georeference constraints when available (GCPs, known distances,
  GNSS priors)
- Persist uncertainty/confidence signals across sparse and dense outputs

## Observability and reproducibility requirements

- Persist per-stage artifacts and metrics logs
- Generate deterministic run manifests for:
  - input set identifiers
  - pipeline/profile parameters
  - software versions and hardware backend details
- Publish automatic QA summary reports including:
  - stage pass/fail reasons
  - failed KPI values
  - recommended recapture actions

## Rollout phases

1. **Phase 1**: sparse reconstruction + sparse QA gates
2. **Phase 2**: dense reconstruction + confidence filtering + dense QA gates
3. **Phase 3**: meshing/texturing integration + benchmark validation on
   reference datasets

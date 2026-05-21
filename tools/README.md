# tools

Runtime orchestration helpers for hardware-aware photogrammetry execution.

## Python pipeline utility

`tools/openphotogrammetry_pipeline.py` implements:

- AMD driver inspection and Meshroom routing with optional ZLUDA fallback
- OpenCLOn12 warning detection
- VRAM-aware PatchMatch option tuning (`max_image_size`, `window_radius`, `num_iterations`, `geom_consistency`)
- Sparse camera clustering and sequential `colmap stereo_fusion` partition execution for RAM-capped fusion
- OPF export through `pyopf` for `InputCameras`, `ProjectedCameras`, `CalibratedCameras`

### Environment hooks

- `OPENPHOTOGRAMMETRY_GPU_VENDOR`
- `OPENPHOTOGRAMMETRY_AMD_DRIVER_VERSION`
- `OPENPHOTOGRAMMETRY_ZLUDA_WRAPPER`
- `OPENPHOTOGRAMMETRY_MESHROOMCL_BIN`
- `OPENPHOTOGRAMMETRY_MESHROOM_CUDA_BIN`
- `OPENPHOTOGRAMMETRY_OPENCL_PLATFORM`
- `OPENPHOTOGRAMMETRY_COLMAP_BIN`

### Quick examples

```bash
python tools/openphotogrammetry_pipeline.py tune-patchmatch --vram-gb 10
python tools/openphotogrammetry_pipeline.py route-meshroom -- --pipeline default.mg
```

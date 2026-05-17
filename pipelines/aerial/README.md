# pipelines/aerial

Large-scene aerial workflow profile for nadir/oblique, geo-aware reconstruction.

## Dataset assumptions

- Nadir-only or nadir + oblique flight plans
- GPS/IMU priors available when possible
- Large area coverage requiring tiling/chunked processing

## Execution behavior

- Prefer spatial + sequential matching strategies for flight line continuity
- Enforce georeference-aware constraints and control-point integration
- Apply tile-wise dense reconstruction with cross-tile consistency checks
- Gate meshing/texturing on dense QA and geo-consistency thresholds

## Quality focus

- Coverage completeness across survey extents
- Stable global scale and geolocation residuals
- Controlled outlier rates near vegetation, water, and low-texture regions

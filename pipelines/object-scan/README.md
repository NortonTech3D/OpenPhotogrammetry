# pipelines/object-scan

Close-range, high-detail workflow profile for turntable and multi-orbit capture.

## Dataset assumptions

- High overlap, short baseline arcs around an object
- Controlled lighting preferred, fixed or grouped intrinsics expected
- Scale constraints from known object dimensions when available

## Execution behavior

- Prioritize high-detail sparse tracks and dense depth quality
- Prefer sequential + vocabulary-tree matching for looped/object-centric capture
- Use stricter blur and duplicate filtering to protect fine geometry
- Require dense confidence gate pass before meshing/texturing handoff
- Optional `--live-scan` mode enables active model growth while the object or
  scanner moves; default mode remains offline batch scanning
- In `--live-scan` mode, the pipeline also requires a viable feature budget to
  support incremental model updates during motion

## Quality focus

- Tight reprojection limits for sub-centimeter object fidelity
- Uniform point density around silhouettes and concavities
- Texture completeness and seam cleanliness suitable for asset export

# Dataset Versioning Policy

Large datasets and binary captures must not be stored directly in Git history.

Recommended policy:
1. Keep small metadata and manifests in `data/`.
2. Store large datasets in remote object storage or release artifacts.
3. Use immutable dataset version IDs and checksum manifests.
4. Mirror benchmark subsets for CI sanity checks only (small footprints).
5. Record schema changes to dataset manifests through pull requests.

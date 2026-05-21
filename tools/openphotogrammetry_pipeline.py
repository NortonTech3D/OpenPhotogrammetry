#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable


MODERN_AMD_DRIVER = (23, 7, 1)


@dataclass(frozen=True)
class HardwareRuntime:
    vendor: str
    driver_version: str | None
    modern_amd_driver: bool
    opencl_on_12: bool
    zluda_wrapper: str | None


@dataclass(frozen=True)
class MeshroomLaunchDecision:
    command: list[str]
    backend: str
    warnings: list[str] = field(default_factory=list)


def _json_log(event: str, **fields: Any) -> None:
    payload = {
        "event": event,
        "iteration_state": fields.pop("iteration_state", None),
        "gpu_memory_delta_mb": fields.pop("gpu_memory_delta_mb", None),
        "spatial_error": fields.pop("spatial_error", None),
        "hardware_temp_c": fields.pop("hardware_temp_c", None),
    }
    payload.update(fields)
    print(json.dumps({k: v for k, v in payload.items() if v is not None}, sort_keys=True))


def _parse_version(version: str | None) -> tuple[int, int, int] | None:
    if not version:
        return None
    parts = []
    for token in version.split(".")[:3]:
        token = token.strip()
        if not token.isdigit():
            return None
        parts.append(int(token))
    while len(parts) < 3:
        parts.append(0)
    return tuple(parts)  # type: ignore[return-value]


def detect_hardware_runtime(env: dict[str, str] | None = None) -> HardwareRuntime:
    env_map = env or os.environ
    vendor = (
        env_map.get("OPENPHOTOGRAMMETRY_GPU_VENDOR")
        or env_map.get("GPU_VENDOR")
        or env_map.get("OP_GPU_VENDOR")
        or "unknown"
    ).strip()
    vendor_lower = vendor.lower()

    driver_version = (
        env_map.get("OPENPHOTOGRAMMETRY_AMD_DRIVER_VERSION")
        or env_map.get("AMD_DRIVER_VERSION")
        or env_map.get("GPU_DRIVER_VERSION")
    )
    parsed_driver = _parse_version(driver_version)
    modern_amd_driver = "amd" in vendor_lower and parsed_driver is not None and parsed_driver >= MODERN_AMD_DRIVER

    opencl_platform = (
        env_map.get("OPENPHOTOGRAMMETRY_OPENCL_PLATFORM")
        or env_map.get("OPENCL_PLATFORM_NAME")
        or ""
    ).lower()
    opencl_vendor_path = (env_map.get("OPENCL_VENDOR_PATH") or "").lower()
    opencl_on_12 = "openclon12" in opencl_platform or "openclon12" in opencl_vendor_path

    zluda_wrapper = env_map.get("OPENPHOTOGRAMMETRY_ZLUDA_WRAPPER") or env_map.get("ZLUDA_WRAPPER")

    runtime = HardwareRuntime(
        vendor=vendor,
        driver_version=driver_version,
        modern_amd_driver=modern_amd_driver,
        opencl_on_12=opencl_on_12,
        zluda_wrapper=zluda_wrapper,
    )

    _json_log(
        "hardware.detected",
        vendor=runtime.vendor,
        driver_version=runtime.driver_version,
        modern_amd_driver=runtime.modern_amd_driver,
        opencl_on_12=runtime.opencl_on_12,
        zluda_wrapper_present=bool(runtime.zluda_wrapper),
    )
    return runtime


def build_meshroom_launch_command(args: Iterable[str], env: dict[str, str] | None = None) -> MeshroomLaunchDecision:
    env_map = env or os.environ
    runtime = detect_hardware_runtime(env_map)

    meshroomcl_bin = env_map.get("OPENPHOTOGRAMMETRY_MESHROOMCL_BIN", "meshroomcl")
    meshroom_cuda_bin = env_map.get("OPENPHOTOGRAMMETRY_MESHROOM_CUDA_BIN", "meshroom_batch")
    extra_args = list(args)

    warnings: list[str] = []
    if runtime.opencl_on_12:
        warnings.append("Detected OpenCLOn12 software layer; hardware acceleration may be unavailable.")

    if runtime.modern_amd_driver and runtime.zluda_wrapper:
        cmd = [runtime.zluda_wrapper, meshroom_cuda_bin, *extra_args]
        _json_log("meshroom.redirect", backend="cuda-via-zluda", command=cmd)
        return MeshroomLaunchDecision(command=cmd, backend="cuda-via-zluda", warnings=warnings)

    if runtime.modern_amd_driver and not runtime.zluda_wrapper:
        warnings.append("Modern AMD driver detected without ZLUDA wrapper; using native OpenCL path.")

    cmd = [meshroomcl_bin, *extra_args]
    _json_log("meshroom.redirect", backend="opencl-native", command=cmd)
    return MeshroomLaunchDecision(command=cmd, backend="opencl-native", warnings=warnings)


def tune_patchmatch_options(vram_gb: float) -> dict[str, int]:
    max_image_size = 2000 if vram_gb < 12 else 2400
    tuned = {
        "max_image_size": max_image_size,
        "window_radius": 4,
        "num_iterations": 3,
        "geom_consistency": 1,
    }
    _json_log("patchmatch.tuned", vram_gb=vram_gb, **tuned)
    return tuned


def _extract_center(camera: dict[str, Any]) -> tuple[float, float, float] | None:
    if isinstance(camera.get("center"), list) and len(camera["center"]) >= 3:
        c = camera["center"]
        return float(c[0]), float(c[1]), float(c[2])

    if isinstance(camera.get("position"), list) and len(camera["position"]) >= 3:
        p = camera["position"]
        return float(p[0]), float(p[1]), float(p[2])

    if isinstance(camera.get("tvec"), list) and len(camera["tvec"]) >= 3:
        t = camera["tvec"]
        return float(t[0]), float(t[1]), float(t[2])

    if all(k in camera for k in ("x", "y", "z")):
        return float(camera["x"]), float(camera["y"]), float(camera["z"])

    return None


def load_sparse_camera_centers(path: str | Path) -> list[dict[str, Any]]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))

    cameras = []
    if isinstance(data.get("images"), list):
        cameras = data["images"]
    elif isinstance(data.get("cameras"), list):
        cameras = data["cameras"]
    elif isinstance(data.get("views"), list):
        cameras = data["views"]

    centers: list[dict[str, Any]] = []
    for idx, camera in enumerate(cameras):
        center = _extract_center(camera)
        if center is None:
            continue
        centers.append(
            {
                "id": camera.get("id", camera.get("image_id", idx)),
                "center": center,
            }
        )

    if not centers:
        raise ValueError("No camera centers found in sparse model JSON.")

    return centers


def _largest_spread_axis(centers: list[dict[str, Any]]) -> int:
    spreads = []
    for axis in range(3):
        values = [c["center"][axis] for c in centers]
        spreads.append(max(values) - min(values))
    return int(max(range(3), key=lambda i: spreads[i]))


def partition_camera_clusters(
    camera_centers: list[dict[str, Any]],
    memory_limit_gb: float,
    estimated_mb_per_camera: float = 180.0,
) -> list[list[dict[str, Any]]]:
    if memory_limit_gb <= 0:
        raise ValueError("memory_limit_gb must be > 0")
    if estimated_mb_per_camera <= 0:
        raise ValueError("estimated_mb_per_camera must be > 0")

    chunk_size = max(1, int(math.floor((memory_limit_gb * 1024.0) / estimated_mb_per_camera)))
    axis = _largest_spread_axis(camera_centers)
    ordered = sorted(camera_centers, key=lambda item: item["center"][axis])

    clusters = [ordered[i : i + chunk_size] for i in range(0, len(ordered), chunk_size)]
    _json_log(
        "cmvs.partitioned",
        camera_count=len(camera_centers),
        chunk_count=len(clusters),
        chunk_size=chunk_size,
        memory_limit_gb=memory_limit_gb,
    )
    return clusters


def run_partitioned_stereo_fusion(
    sparse_model_json: str | Path,
    workspace_path: str | Path,
    output_dir: str | Path,
    memory_limit_gb: float,
    estimated_mb_per_camera: float = 180.0,
    colmap_bin: str | None = None,
) -> list[Path]:
    camera_centers = load_sparse_camera_centers(sparse_model_json)
    clusters = partition_camera_clusters(camera_centers, memory_limit_gb, estimated_mb_per_camera)

    workspace = Path(workspace_path)
    out_root = Path(output_dir)
    out_root.mkdir(parents=True, exist_ok=True)

    executable = colmap_bin or os.environ.get("OPENPHOTOGRAMMETRY_COLMAP_BIN", "colmap")
    outputs: list[Path] = []

    for cluster_index, cluster in enumerate(clusters):
        cluster_dir = out_root / f"cluster_{cluster_index:03d}"
        cluster_dir.mkdir(parents=True, exist_ok=True)

        image_ids_file = cluster_dir / "image_ids.txt"
        image_ids_file.write_text("\n".join(str(c["id"]) for c in cluster), encoding="utf-8")

        fused_ply = cluster_dir / "fused.ply"
        cmd = [
            executable,
            "stereo_fusion",
            "--workspace_path",
            str(workspace),
            "--output_path",
            str(fused_ply),
            "--image_list_path",
            str(image_ids_file),
        ]

        _json_log(
            "fusion.cluster.start",
            iteration_state=f"cluster_{cluster_index + 1}/{len(clusters)}",
            cluster_size=len(cluster),
            output_path=str(fused_ply),
        )
        subprocess.run(cmd, check=True)

        outputs.append(fused_ply)
        _json_log(
            "fusion.cluster.complete",
            iteration_state=f"cluster_{cluster_index + 1}/{len(clusters)}",
            cluster_size=len(cluster),
            output_path=str(fused_ply),
        )

    return outputs


def export_opf_project(
    output_path: str | Path,
    input_cameras: list[dict[str, Any]],
    projected_cameras: list[dict[str, Any]],
    calibrated_cameras: list[dict[str, Any]],
    physical_scale_reference: dict[str, Any],
) -> None:
    try:
        import pyopf  # type: ignore
    except ImportError as exc:
        raise RuntimeError("pyopf is required to export .opf files.") from exc

    document = {
        "InputCameras": input_cameras,
        "ProjectedCameras": projected_cameras,
        "CalibratedCameras": calibrated_cameras,
        "PhysicalScaleReference": physical_scale_reference,
    }

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)

    if hasattr(pyopf, "dump"):
        with output.open("w", encoding="utf-8") as handle:
            pyopf.dump(document, handle)
    elif hasattr(pyopf, "dumps"):
        output.write_text(pyopf.dumps(document), encoding="utf-8")
    elif hasattr(pyopf, "serialize"):
        output.write_text(pyopf.serialize(document), encoding="utf-8")
    else:
        raise RuntimeError("Unsupported pyopf API. Expected dump, dumps, or serialize.")

    _json_log(
        "opf.exported",
        output_path=str(output),
        input_camera_count=len(input_cameras),
        projected_camera_count=len(projected_cameras),
        calibrated_camera_count=len(calibrated_cameras),
    )


def _cmd_tune_patchmatch(argv: argparse.Namespace) -> int:
    tuned = tune_patchmatch_options(argv.vram_gb)
    print(json.dumps(tuned, sort_keys=True))
    return 0


def _cmd_route_meshroom(argv: argparse.Namespace) -> int:
    decision = build_meshroom_launch_command(argv.args)
    payload = {
        "backend": decision.backend,
        "command": decision.command,
        "warnings": decision.warnings,
    }
    print(json.dumps(payload, sort_keys=True))
    return 0


def _build_cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="OpenPhotogrammetry runtime orchestration utilities")
    subparsers = parser.add_subparsers(dest="command", required=True)

    tune = subparsers.add_parser("tune-patchmatch", help="Tune PatchMatch defaults from VRAM")
    tune.add_argument("--vram-gb", type=float, required=True)
    tune.set_defaults(func=_cmd_tune_patchmatch)

    route = subparsers.add_parser("route-meshroom", help="Generate Meshroom launch decision")
    route.add_argument("args", nargs=argparse.REMAINDER)
    route.set_defaults(func=_cmd_route_meshroom)

    return parser


def main() -> int:
    parser = _build_cli()
    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())

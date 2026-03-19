# Drone SLAM Handoff

## What This Is

A drone 3D mapping simulation: Isaac Sim 4.5 flies a drone with a stereo/depth camera on a circular orbit around a procedural house scene, captures depth images, reprojects them to 3D, and saves the accumulated point cloud as a PLY file.

**Two execution modes:**
1. `sim_map.py` — Self-contained depth-mapping (no ROS 2 / cuVSLAM needed). **This is the working path.**
2. `sim_app.py` — Full ROS 2 pipeline publishing stereo to cuVSLAM. Requires Isaac ROS container (not yet tested).

## Current State (2026-03-19)

**Working end-to-end on Linux (RTX 3070):**
- Isaac Sim 4.5 container (`nvcr.io/nvidia/isaac-sim:4.5.0`) runs headless
- `sim_map.py` completes a full 360° orbit, captures 180 depth frames, produces a 2.2M point PLY map
- Map quality verified: Z range [-1, 8]m covering ground → roof peak, matches scene geometry
- Runtime: ~4min shader warmup (first launch) + 46s simulation

**Output files:**
- `output/maps/drone_map.ply` — 2.2M point ASCII PLY (46MB)
- `output/maps/drone_map_views.png` — Top-down + side + front view renders

### File Map

| File | Status | Notes |
|------|--------|-------|
| `drone_slam/sim_map.py` | **Working** | Standalone depth-mapping, no ROS 2 needed |
| `drone_slam/sim_app.py` | Done, untested | Full ROS 2 + OmniGraph pipeline for cuVSLAM |
| `drone_slam/scene_builder.py` | **Working** | USD prims via `stage.DefinePrim()` + XformOps |
| `drone_slam/path_follower.py` | **Working** | `CircularOrbitFollower` — analytical circular orbit |
| `drone_slam/map_saver.py` | Done, untested | ROS 2 PointCloud2 → PLY converter for cuVSLAM output |
| `config/scene.yaml` | Done | 50×50m ground, 8×8×6m house + cone roof, tree, sidewalk |
| `config/flight_path.yaml` | Done | 15m radius, 5.5m alt, 2m/s CCW, 10cm baseline, 640×480 |
| `config/cuVSLAM.yaml` | Done | sim_time, rectified, all viz enabled |
| `launch/slam.launch.py` | Done | cuVSLAM ComposableNode + map_saver |
| `launch/sim.launch.py` | Done | ExecuteProcess wrapper for Isaac Sim |
| `launch/full.launch.py` | Done | Both launches with TimerAction delay |
| `rviz/slam_view.rviz` | Done | landmarks + observations + path + camera feed |

## How to Run (Linux with NVIDIA GPU)

```bash
# Requires: Docker, NVIDIA Container Toolkit, nvcr.io/nvidia/isaac-sim:4.5.0

cd /path/to/drone_slam
mkdir -p output/maps logs

docker run --rm --gpus all \
  --network host \
  -e "ACCEPT_EULA=Y" \
  -v "$(pwd):/workspace" \
  -w /workspace \
  --entrypoint bash \
  nvcr.io/nvidia/isaac-sim:4.5.0 \
  -c '
export PYTHONPATH=/workspace:$PYTHONPATH
/isaac-sim/python.sh /workspace/drone_slam/sim_map.py \
  --headless \
  --config-dir /workspace/config \
  --output-dir /workspace/output/maps \
  --capture-every 15 \
  --downsample 4 \
  --max-depth 40.0
'

# Output: output/maps/drone_map.ply
```

**First run takes ~5 minutes** (GPU shader compilation). Subsequent runs: ~1 minute.

## Design Decisions

1. **Stereo-only, no IMU** — sufficient for slow smooth orbit.
2. **Raw USD Camera prims** under `/World/Drone/` XForm. Cameras are children and inherit parent transform.
3. **Camera orientation**: USD cameras look along -Z by default. A -90° rotation around Y is applied so cameras look forward (+X in drone frame, toward orbit center).
4. **XformOp precision**: Must use `PrecisionDouble` explicitly — `stage.DefinePrim()` creates float-precision attributes by default. Mismatched precision causes segfaults.
5. **Quaternion order**: scipy outputs `(x,y,z,w)` → USD `Quatd` takes `(w,x,y,z)`.
6. **Depth reprojection**: Camera local frame is X-right, Y-up, -Z-forward. Image v-axis is inverted (down). World rotation = `drone_yaw * camera_local_rotation`.
7. **Scene prims**: Use `stage.DefinePrim()` directly, NOT `isaacsim.core.utils.prims.create_prim()` — the latter adds default xform ops that conflict with our explicit ops.
8. **Atomic PLY writes**: `tempfile.mkstemp()` + `os.replace()` to prevent corruption.

## Bugs Fixed During Integration

| Bug | Symptom | Fix |
|-----|---------|-----|
| `create_prim` xform conflict | `pxr.Tf.ErrorException` on AddScaleOp | Replaced with `stage.DefinePrim()` |
| XformOp precision mismatch | Segfault on `orient_op.Set(Gf.Quatd(...))` | Added `precision=PrecisionDouble` |
| Camera pointing down | Depth = 5.5m everywhere (= altitude) | Added -90° Y rotation to camera prim |
| Flat point cloud (Z ≈ 0) | Only ground captured | Fixed reprojection: negate v-axis, -Z forward |
| World transform missing camera rot | Points at wrong Z | Compose `R_drone * R_cam_local` for world transform |

## Setting Up on M3 Mac

**Isaac Sim does NOT run on Apple Silicon natively.** It requires an NVIDIA GPU for RTX rendering. Options:

### Option A: Remote GPU (Recommended)

Run Isaac Sim on a Linux machine/cloud instance with NVIDIA GPU, view results locally.

```bash
# On remote Linux box with GPU:
# Follow "How to Run" above, then scp the PLY file

# On Mac, view the PLY:
brew install meshlab
meshlab output/maps/drone_map.ply
```

Cloud options: AWS g5.xlarge (A10G), Lambda Cloud, vast.ai, RunPod — all work with the Isaac Sim container.

### Option B: Isaac Sim via Omniverse Streaming

NVIDIA offers cloud streaming where Isaac Sim renders remotely and streams to a thin client. This requires an Omniverse Nucleus setup — overkill for this project.

### Option C: Strip Isaac Sim, Use a Different Simulator

If you need the full pipeline on Mac:

1. **Replace Isaac Sim with a lightweight renderer** — e.g., PyBullet, Gazebo (runs on Mac via Homebrew), or Habitat-Sim (has Apple Silicon builds).
2. Keep `path_follower.py` and `scene_builder.py` concept (they're pure math / geometry).
3. Rewrite the sim driver to use the new renderer's depth API.
4. `map_saver.py` and the PLY writing are pure Python — work anywhere.

### Option D: Run the Existing Code Without Isaac Sim

The point cloud math, path follower, and PLY export are all pure Python. You can develop/test everything except the actual rendering:

```bash
# Works on Mac as-is:
pip install numpy scipy pyyaml matplotlib

# Test path follower
python -m drone_slam.path_follower --config config/flight_path.yaml

# View existing PLY
pip install open3d
python -c "import open3d as o3d; o3d.visualization.draw_geometries([o3d.io.read_point_cloud('output/maps/drone_map.ply')])"
```

### Mac Development Dependencies

```bash
# Python env
python3 -m venv .venv && source .venv/bin/activate
pip install numpy scipy pyyaml matplotlib

# Optional: view PLY files
pip install open3d
# or
brew install meshlab

# Optional: ROS 2 (for map_saver / launch file development)
# ROS 2 Humble has experimental Mac support via conda:
# https://docs.ros.org/en/humble/Installation/Alternatives/macOS-Development-Setup.html
```

## Dependencies

| Package | Where Used | Available on Mac? |
|---------|-----------|-------------------|
| `numpy` | Depth reprojection | Yes |
| `scipy` | Quaternion math | Yes |
| `pyyaml` | Config loading | Yes |
| `matplotlib` | PLY visualization | Yes |
| `isaacsim` | Simulation + rendering | No (NVIDIA GPU only) |
| `omni.replicator.core` | Depth annotator | No (Isaac Sim only) |
| `pxr` (USD) | Scene construction | Yes (`pip install usd-core`) |
| `rclpy` | ROS 2 communication | Experimental on Mac |

## What Hasn't Been Done

1. **cuVSLAM pipeline** — Isaac ROS container not pulled/tested. `sim_app.py` + `slam.launch.py` are written but untested.
2. **No git repo** — not under version control yet.
3. **No pytest suite** — `path_follower` verified via main(), no formal tests.
4. **No Docker Compose** — single `docker run` command, no orchestration file.
5. **Color in point cloud** — currently XYZ only, no RGB. Could add by capturing RGB alongside depth.
6. **IMU integration** — deferred to RL path planning phase.

"""Standalone drone mapping script — no ROS 2 or cuVSLAM required.

Flies a drone on a circular orbit in Isaac Sim, captures depth images from
the left camera, reprojects to 3D world coordinates, and saves the accumulated
point cloud as an ASCII PLY file.

Run inside the Isaac Sim container:
    ./python.sh drone_slam/sim_map.py --headless --config-dir config/
"""

from __future__ import annotations

import argparse
import os
import tempfile
import time
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="Drone depth-mapping simulation")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--config-dir", default="config")
    parser.add_argument("--output-dir", default="output/maps")
    parser.add_argument("--capture-every", type=int, default=10,
                        help="Capture depth every N physics steps")
    parser.add_argument("--max-depth", type=float, default=50.0,
                        help="Discard points beyond this depth (meters)")
    parser.add_argument("--downsample", type=int, default=4,
                        help="Spatial downsample factor for depth images")
    args = parser.parse_args()
    config_dir = Path(args.config_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # ── 1. Create SimulationApp ───────────────────────────────────────
    print("[sim_map] Creating SimulationApp...", flush=True)
    from isaacsim import SimulationApp

    sim_app = SimulationApp({"headless": args.headless})

    # ── 2. Imports (after SimulationApp) ──────────────────────────────
    print("[sim_map] Importing modules...", flush=True)
    import numpy as np
    from isaacsim.core.api import World
    from omni.usd import get_context
    from pxr import Gf, UsdGeom

    from drone_slam.path_follower import load_flight_config, make_follower
    from drone_slam.scene_builder import build_scene, load_scene_config

    # ── 3. Create World + Scene ───────────────────────────────────────
    print("[sim_map] Creating world and scene...", flush=True)
    world = World(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)

    scene_cfg = load_scene_config(config_dir / "scene.yaml")
    build_scene(scene_cfg)

    # ── 4. Create drone + camera ──────────────────────────────────────
    flight_cfg = load_flight_config(config_dir / "flight_path.yaml")
    cam_cfg = flight_cfg["camera"]
    res_w, res_h = cam_cfg["resolution"]
    focal_mm = cam_cfg["focal_length_mm"]
    near_clip, far_clip = cam_cfg["clipping"]

    stage = get_context().get_stage()

    # Drone XForm
    drone_prim = stage.DefinePrim("/World/Drone", "Xform")
    drone_xform = UsdGeom.Xformable(drone_prim)
    translate_op = drone_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    orient_op = drone_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)

    # Single depth camera (left position)
    baseline_half = cam_cfg["baseline"] / 2.0
    cam_path = "/World/Drone/CameraLeft"
    cam_prim = stage.DefinePrim(cam_path, "Camera")
    cam = UsdGeom.Camera(cam_prim)
    cam_xform = UsdGeom.Xformable(cam_prim)
    cam_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(0.0, baseline_half, 0.0)
    )
    # USD cameras look along -Z by default. Rotate -90° around Y to look
    # along +X (the drone's forward direction toward center after yaw).
    cam_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(0.7071067811865476, 0.0, -0.7071067811865476, 0.0)  # w,x,y,z
    )
    cam.CreateFocalLengthAttr().Set(focal_mm)
    cam.CreateHorizontalApertureAttr().Set(focal_mm * res_w / res_w)
    cam.CreateClippingRangeAttr().Set(Gf.Vec2f(near_clip, far_clip))

    print("[sim_map] Scene and drone created.", flush=True)

    # ── 5. Set up Replicator for depth capture ────────────────────────
    print("[sim_map] Setting up depth capture...", flush=True)
    import omni.replicator.core as rep

    render_product = rep.create.render_product(cam_path, (res_w, res_h))

    # Use the annotator API to get depth
    depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
    depth_annotator.attach([render_product])

    # ── 6. Path follower ──────────────────────────────────────────────
    follower = make_follower(flight_cfg)
    physics_dt = 1.0 / 60.0

    # Camera intrinsics for reprojection
    # focal_mm is in mm, horizontal_aperture is also in mm
    # For USD cameras: fx_pixels = focal_length_mm / horizontal_aperture_mm * width
    # Since we set horizontal_aperture = focal_mm, fx = res_w
    # This seems wrong — let's compute properly.
    # Standard: fx = focal_length_mm * image_width / sensor_width_mm
    # We set horizontal_aperture = focal_mm (which is a simplification from the original code)
    # So fx = focal_mm / focal_mm * res_w = res_w
    # That gives a very wide FOV. Let's just use it as-is and see what we get.
    horiz_aperture_mm = focal_mm  # as set above
    fx = focal_mm / horiz_aperture_mm * res_w  # = res_w
    fy = fx  # square pixels
    cx_img = res_w / 2.0
    cy_img = res_h / 2.0

    print(f"[sim_map] Camera intrinsics: fx={fx:.1f}, fy={fy:.1f}, "
          f"cx={cx_img:.1f}, cy={cy_img:.1f}", flush=True)

    # ── 7. Main loop — orbit + depth capture ──────────────────────────
    world.reset()
    print("[sim_map] Simulation started. Drone orbiting...", flush=True)

    all_points: list[np.ndarray] = []
    step_count = 0
    capture_count = 0
    ds = args.downsample
    max_d = args.max_depth

    # Need a few warmup frames for renderer
    for _ in range(10):
        world.step(render=True)

    start_time = time.time()

    try:
        while sim_app.is_running():
            # Advance path
            pos, quat_xyzw, done = follower.step(physics_dt)

            # Update drone pose (USD quaternion: w,x,y,z)
            translate_op.Set(Gf.Vec3d(*pos))
            orient_op.Set(
                Gf.Quatd(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2])
            )

            world.step(render=True)
            step_count += 1

            # Capture depth at intervals
            if step_count % args.capture_every == 0:
                depth_data = depth_annotator.get_data()
                if depth_data is not None and depth_data.size > 0:
                    depth = np.array(depth_data, dtype=np.float32)
                    if depth.ndim == 2:
                        h, w = depth.shape

                        # Downsample
                        depth_ds = depth[::ds, ::ds]
                        h_ds, w_ds = depth_ds.shape

                        # Create pixel coordinate grids
                        u = np.arange(0, w, ds, dtype=np.float32)
                        v = np.arange(0, h, ds, dtype=np.float32)
                        uu, vv = np.meshgrid(u, v)

                        # Filter valid depth
                        valid = (depth_ds > 0.1) & (depth_ds < max_d) & np.isfinite(depth_ds)

                        if valid.any():
                            d = depth_ds[valid]
                            u_v = uu[valid]
                            v_v = vv[valid]

                            # Reproject to camera-local 3D
                            # USD camera convention: looks along -Z, X right, Y up
                            # Image convention: u right, v down
                            x_cam = (u_v - cx_img) * d / fx
                            y_cam = -(v_v - cy_img) * d / fy  # negate: v down → y up
                            z_cam = -d  # camera looks along -Z

                            # Camera-local points (N, 3)
                            pts_cam = np.stack([x_cam, y_cam, z_cam], axis=-1)

                            # Transform to world coordinates
                            # Camera world rotation = drone_yaw * camera_local_rotation
                            # Camera local rotation: -90° around Y (looks along +X)
                            from scipy.spatial.transform import Rotation

                            R_drone = Rotation.from_quat(quat_xyzw)
                            R_cam_local = Rotation.from_euler("y", -90, degrees=True)
                            R_world = (R_drone * R_cam_local).as_matrix()
                            pts_world = (R_world @ pts_cam.T).T + np.array(pos)

                            all_points.append(pts_world)
                            capture_count += 1

                            if capture_count % 10 == 0:
                                total_pts = sum(p.shape[0] for p in all_points)
                                elapsed = time.time() - start_time
                                print(f"[sim_map] Capture {capture_count}: "
                                      f"{total_pts:,} points total, "
                                      f"{elapsed:.0f}s elapsed", flush=True)

            if done:
                print("[sim_map] Orbit complete!", flush=True)
                # Run a few more frames
                for _ in range(30):
                    world.step(render=True)
                break

    except KeyboardInterrupt:
        print("[sim_map] Interrupted.", flush=True)

    # ── 8. Save PLY ───────────────────────────────────────────────────
    if all_points:
        cloud = np.concatenate(all_points, axis=0)
        print(f"[sim_map] Saving {cloud.shape[0]:,} points to PLY...", flush=True)

        ply_path = output_dir / "drone_map.ply"

        # Atomic write via tempfile
        fd, tmp_path = tempfile.mkstemp(dir=str(output_dir), suffix=".ply")
        try:
            with os.fdopen(fd, "w") as f:
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {cloud.shape[0]}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("end_header\n")
                for pt in cloud:
                    f.write(f"{pt[0]:.4f} {pt[1]:.4f} {pt[2]:.4f}\n")
            os.replace(tmp_path, str(ply_path))
            print(f"[sim_map] PLY saved: {ply_path} ({cloud.shape[0]:,} points)", flush=True)
        except Exception as e:
            print(f"[sim_map] Error saving PLY: {e}", flush=True)
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)
    else:
        print("[sim_map] No points captured!", flush=True)

    # ── 9. Cleanup ────────────────────────────────────────────────────
    elapsed_total = time.time() - start_time
    print(f"[sim_map] Done. Total time: {elapsed_total:.0f}s", flush=True)
    sim_app.close()


if __name__ == "__main__":
    main()

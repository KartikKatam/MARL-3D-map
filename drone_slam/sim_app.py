"""Main Isaac Sim standalone script for drone SLAM mapping.

IMPORTANT: SimulationApp must be created BEFORE any omni/pxr imports.

Launches a drone with stereo cameras on a circular orbit around a house scene,
publishes stereo images + camera_info + clock via OmniGraph ROS 2 bridge,
and signals /path_complete when the orbit finishes.

Run inside the Isaac Sim container:
    ./python.sh drone_slam/sim_app.py [--headless] [--config-dir config/]
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path


def main() -> None:
    # ── 1. Parse args BEFORE SimulationApp ──────────────────────────────
    parser = argparse.ArgumentParser(description="Drone SLAM simulation")
    parser.add_argument("--headless", action="store_true", help="Run without GUI")
    parser.add_argument(
        "--config-dir", default="config", help="Directory containing YAML configs"
    )
    args = parser.parse_args()
    config_dir = Path(args.config_dir)

    # ── 2. Create SimulationApp (must be first) ─────────────────────────
    from isaacsim import SimulationApp

    sim_app = SimulationApp({"headless": args.headless})

    # ── 3. NOW import omni/pxr/isaacsim modules ────────────────────────
    import omni.graph.core as og
    from isaacsim.core.api import World
    from omni.usd import get_context
    from pxr import Gf, UsdGeom

    from drone_slam.path_follower import load_flight_config, make_follower
    from drone_slam.scene_builder import build_scene, load_scene_config

    # ── 4. Create World ─────────────────────────────────────────────────
    world = World(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)

    # ── 5. Build scene ──────────────────────────────────────────────────
    scene_cfg = load_scene_config(config_dir / "scene.yaml")
    build_scene(scene_cfg)

    # ── 6. Create drone XForm + stereo cameras ──────────────────────────
    flight_cfg = load_flight_config(config_dir / "flight_path.yaml")
    cam_cfg = flight_cfg["camera"]
    baseline_half = cam_cfg["baseline"] / 2.0
    res_w, res_h = cam_cfg["resolution"]
    focal_mm = cam_cfg["focal_length_mm"]
    near_clip, far_clip = cam_cfg["clipping"]

    stage = get_context().get_stage()

    # Drone XForm
    drone_prim = stage.DefinePrim("/World/Drone", "Xform")
    drone_xform = UsdGeom.Xformable(drone_prim)
    translate_op = drone_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    orient_op = drone_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)

    # Stereo cameras as children of drone
    for cam_name, y_offset in [("CameraLeft", baseline_half), ("CameraRight", -baseline_half)]:
        cam_path = f"/World/Drone/{cam_name}"
        cam_prim = stage.DefinePrim(cam_path, "Camera")
        cam = UsdGeom.Camera(cam_prim)

        cam_xform = UsdGeom.Xformable(cam_prim)
        cam_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0.0, y_offset, 0.0)
        )
        # USD cameras look along -Z by default. Rotate -90° around Y to look
        # along +X (the drone's forward direction toward center after yaw).
        cam_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(0.7071067811865476, 0.0, -0.7071067811865476, 0.0)
        )

        cam.CreateFocalLengthAttr().Set(focal_mm)
        cam.CreateHorizontalApertureAttr().Set(focal_mm * res_w / res_w)  # simplified
        cam.CreateClippingRangeAttr().Set(Gf.Vec2f(near_clip, far_clip))

    # ── 7. OmniGraph ROS 2 bridge ──────────────────────────────────────
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/ROS2Bridge", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                # Left camera
                ("CreateRPLeft", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("PublishRGBLeft", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("PublishInfoLeft", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                # Right camera
                ("CreateRPRight", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("PublishRGBRight", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("PublishInfoRight", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                # Render products
                ("CreateRPLeft.inputs:cameraPrim", "/World/Drone/CameraLeft"),
                ("CreateRPLeft.inputs:width", res_w),
                ("CreateRPLeft.inputs:height", res_h),
                ("CreateRPRight.inputs:cameraPrim", "/World/Drone/CameraRight"),
                ("CreateRPRight.inputs:width", res_w),
                ("CreateRPRight.inputs:height", res_h),
                # Left RGB
                ("PublishRGBLeft.inputs:type", "rgb"),
                ("PublishRGBLeft.inputs:topicName", "front_stereo_camera/left/image_rect_color"),
                ("PublishRGBLeft.inputs:frameId", "camera_left"),
                # Left CameraInfo
                ("PublishInfoLeft.inputs:type", "camera_info"),
                ("PublishInfoLeft.inputs:topicName", "front_stereo_camera/left/camera_info"),
                ("PublishInfoLeft.inputs:frameId", "camera_left"),
                # Right RGB
                ("PublishRGBRight.inputs:type", "rgb"),
                ("PublishRGBRight.inputs:topicName", "front_stereo_camera/right/image_rect_color"),
                ("PublishRGBRight.inputs:frameId", "camera_right"),
                # Right CameraInfo
                ("PublishInfoRight.inputs:type", "camera_info"),
                ("PublishInfoRight.inputs:topicName", "front_stereo_camera/right/camera_info"),
                ("PublishInfoRight.inputs:frameId", "camera_right"),
            ],
            keys.CONNECT: [
                # Tick drives everything
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CreateRPLeft.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CreateRPRight.inputs:execIn"),
                # Render products → camera helpers
                ("CreateRPLeft.outputs:execOut", "PublishRGBLeft.inputs:execIn"),
                ("CreateRPLeft.outputs:execOut", "PublishInfoLeft.inputs:execIn"),
                ("CreateRPLeft.outputs:renderProductPath", "PublishRGBLeft.inputs:renderProductPath"),
                ("CreateRPLeft.outputs:renderProductPath", "PublishInfoLeft.inputs:renderProductPath"),
                ("CreateRPRight.outputs:execOut", "PublishRGBRight.inputs:execIn"),
                ("CreateRPRight.outputs:execOut", "PublishInfoRight.inputs:execIn"),
                ("CreateRPRight.outputs:renderProductPath", "PublishRGBRight.inputs:renderProductPath"),
                ("CreateRPRight.outputs:renderProductPath", "PublishInfoRight.inputs:renderProductPath"),
            ],
        },
    )

    # ── 8. ROS 2 path_complete publisher ────────────────────────────────
    import rclpy
    from std_msgs.msg import Bool

    rclpy.init()
    ros_node = rclpy.create_node("sim_app_signals")
    path_complete_pub = ros_node.create_publisher(Bool, "/path_complete", 10)

    # ── 9. Path follower ────────────────────────────────────────────────
    follower = make_follower(flight_cfg)
    physics_dt = 1.0 / 60.0
    path_complete_published = False
    path_complete_time: float | None = None

    def physics_step_callback(_step_size: float) -> None:
        nonlocal path_complete_published, path_complete_time

        pos, quat_xyzw, done = follower.step(physics_dt)

        # Update drone pose — USD uses (w,x,y,z) quaternion order
        translate_op.Set(Gf.Vec3d(*pos))
        orient_op.Set(Gf.Quatd(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]))

        if done and not path_complete_published:
            msg = Bool()
            msg.data = True
            path_complete_pub.publish(msg)
            path_complete_published = True
            path_complete_time = time.time()
            print("[sim_app] Path complete — published /path_complete")

    world.add_physics_callback("drone_orbit", physics_step_callback)

    # ── 10. Main loop ───────────────────────────────────────────────────
    world.reset()
    print("[sim_app] Simulation started. Drone orbiting...")

    try:
        while sim_app.is_running():
            world.step(render=True)
            rclpy.spin_once(ros_node, timeout_sec=0.0)

            # Exit 5 seconds after path completes
            if path_complete_time is not None and (time.time() - path_complete_time) > 5.0:
                print("[sim_app] Post-completion wait finished. Shutting down.")
                break
    except KeyboardInterrupt:
        print("[sim_app] Interrupted.")
    finally:
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sim_app.close()


if __name__ == "__main__":
    main()

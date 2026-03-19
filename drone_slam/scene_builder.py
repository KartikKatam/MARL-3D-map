"""Build a simple scene in Isaac Sim for drone SLAM mapping.

Creates ground, house (walls + roof cone), tree (trunk + canopy), and sidewalk
using USD prims with display colors. Designed to run inside the Isaac Sim
container with isaacsim.core available.

Must be imported AFTER SimulationApp is created.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from pxr import Gf, UsdGeom



def load_scene_config(path: str | Path) -> dict[str, Any]:
    """Load scene.yaml and return the raw dict."""
    with open(path) as f:
        return yaml.safe_load(f)


def _set_display_color(prim_path: str, color: list[float]) -> None:
    """Set primvars:displayColor on a UsdGeom prim."""
    from omni.usd import get_context

    stage = get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    gprim = UsdGeom.Gprim(prim)
    gprim.CreateDisplayColorAttr().Set([Gf.Vec3f(*color)])


def _create_cube(name: str, parent: str, position: list[float], size: list[float], color: list[float]) -> str:
    """Create a Cube prim scaled to act as a box."""
    prim_path = f"{parent}/{name}"

    from omni.usd import get_context

    stage = get_context().get_stage()
    stage.DefinePrim(prim_path, "Cube")
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3d(size[0] / 2, size[1] / 2, size[2] / 2))

    _set_display_color(prim_path, color)
    return prim_path


def _create_cone(name: str, parent: str, position: list[float], radius: float, height: float, color: list[float]) -> str:
    """Create a Cone prim for the roof."""
    prim_path = f"{parent}/{name}"

    from omni.usd import get_context

    stage = get_context().get_stage()
    stage.DefinePrim(prim_path, "Cone")
    prim = stage.GetPrimAtPath(prim_path)

    xform = UsdGeom.Xformable(prim)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3d(radius, radius, height / 2))

    _set_display_color(prim_path, color)
    return prim_path


def _create_cylinder(name: str, parent: str, position: list[float], radius: float, height: float, color: list[float]) -> str:
    """Create a Cylinder prim for the tree trunk."""
    prim_path = f"{parent}/{name}"

    from omni.usd import get_context

    stage = get_context().get_stage()
    stage.DefinePrim(prim_path, "Cylinder")
    prim = stage.GetPrimAtPath(prim_path)

    xform = UsdGeom.Xformable(prim)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3d(radius, radius, height / 2))

    _set_display_color(prim_path, color)
    return prim_path


def _create_sphere(name: str, parent: str, position: list[float], radius: float, color: list[float]) -> str:
    """Create a Sphere prim for the tree canopy."""
    prim_path = f"{parent}/{name}"

    from omni.usd import get_context

    stage = get_context().get_stage()
    stage.DefinePrim(prim_path, "Sphere")
    prim = stage.GetPrimAtPath(prim_path)

    xform = UsdGeom.Xformable(prim)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3d(radius, radius, radius))

    _set_display_color(prim_path, color)
    return prim_path


def build_scene(config: dict[str, Any], root: str = "/World") -> None:
    """Create all scene prims from config dict.

    Args:
        config: Parsed scene.yaml dict.
        root: USD root path for the scene.
    """
    from omni.usd import get_context

    stage = get_context().get_stage()

    scene_root = f"{root}/Scene"
    stage.DefinePrim(scene_root, "Xform")

    # Ground plane
    g = config["ground"]
    _create_cube("Ground", scene_root, g["position"], g["size"], g["color"])

    # House
    house_path = f"{scene_root}/House"
    stage.DefinePrim(house_path, "Xform")

    w = config["house"]["walls"]
    _create_cube("Walls", house_path, w["position"], w["size"], w["color"])

    r = config["house"]["roof"]
    _create_cone(
        "Roof", house_path,
        r["position"], r["radius"], r["height"], r["color"],
    )

    # Tree
    t = config["tree"]
    tree_x = t["offset_from_house"][0]
    tree_y = t["offset_from_house"][1]
    tree_path = f"{scene_root}/Tree"
    stage.DefinePrim(tree_path, "Xform")

    trunk = t["trunk"]
    _create_cylinder(
        "Trunk", tree_path,
        [tree_x, tree_y, trunk["position_z"]],
        trunk["radius"], trunk["height"], trunk["color"],
    )

    canopy = t["canopy"]
    _create_sphere(
        "Canopy", tree_path,
        [tree_x, tree_y, canopy["position_z"]],
        canopy["radius"], canopy["color"],
    )

    # Sidewalk
    s = config["sidewalk"]
    _create_cube("Sidewalk", scene_root, s["position"], s["size"], s["color"])


def main() -> None:
    """Standalone test: create SimulationApp, build scene, render a few frames."""
    import argparse

    parser = argparse.ArgumentParser(description="Test scene builder in Isaac Sim")
    parser.add_argument("--config", default="config/scene.yaml")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    # SimulationApp must be created before any omni imports
    from isaacsim import SimulationApp

    sim_app = SimulationApp({"headless": args.headless})

    from isaacsim.core.api import World

    world = World()
    world.scene.add_default_ground_plane()

    config = load_scene_config(args.config)
    build_scene(config)

    world.reset()
    for _ in range(120):  # ~2 seconds at 60Hz
        world.step(render=True)

    print("Scene built successfully. Closing.")
    sim_app.close()


if __name__ == "__main__":
    main()

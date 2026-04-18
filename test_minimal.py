"""Minimal test to find where sim_app.py hangs."""
from __future__ import annotations
import sys

print("[test] Step 1: Creating SimulationApp...", flush=True)
from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": True})
print("[test] Step 2: SimulationApp created", flush=True)

print("[test] Step 3: Importing omni.graph.core...", flush=True)
import omni.graph.core as og
print("[test] Step 4: omni.graph.core imported", flush=True)

print("[test] Step 5: Importing World...", flush=True)
from isaacsim.core.api import World
print("[test] Step 6: World imported", flush=True)

print("[test] Step 7: Importing pxr...", flush=True)
from pxr import Gf, UsdGeom
print("[test] Step 8: pxr imported", flush=True)

print("[test] Step 9: Importing omni.usd...", flush=True)
from omni.usd import get_context
print("[test] Step 10: omni.usd imported", flush=True)

print("[test] Step 11: Creating World...", flush=True)
world = World(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
print("[test] Step 12: World created", flush=True)

print("[test] Step 13: Getting stage...", flush=True)
stage = get_context().get_stage()
print(f"[test] Step 14: Stage: {stage}", flush=True)

print("[test] Step 15: Defining drone prim...", flush=True)
drone_prim = stage.DefinePrim("/World/Drone", "Xform")
print(f"[test] Step 16: Drone prim: {drone_prim}", flush=True)

print("[test] Step 17: Calling world.reset()...", flush=True)
world.reset()
print("[test] Step 18: world.reset() done", flush=True)

print("[test] Step 19: Stepping once...", flush=True)
world.step(render=True)
print("[test] Step 20: Step done. SUCCESS!", flush=True)

sim_app.close()
print("[test] DONE", flush=True)

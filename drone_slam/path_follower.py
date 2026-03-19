"""Circular orbit path follower for drone mapping.

Produces smooth analytical poses (position + quaternion) for a drone
orbiting a point at constant altitude. No spline interpolation needed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml
from scipy.spatial.transform import Rotation


@dataclass
class CircularOrbitFollower:
    """Follows a circular orbit, facing inward toward the center."""

    cx: float
    cy: float
    radius: float
    altitude: float
    omega: float  # angular velocity in rad/s (positive = CCW)
    theta: float = 0.0  # current angle in radians
    total_angle: float = 0.0  # accumulated angle for completion check
    _complete: bool = field(default=False, repr=False)

    def step(self, dt: float) -> tuple[list[float], list[float], bool]:
        """Advance one timestep.

        Returns:
            (position[3], quat_xyzw[4], complete)
        """
        if self._complete:
            # Hold final position after completion
            return self._pose_at(self.theta)

        self.theta += self.omega * dt
        self.total_angle += abs(self.omega * dt)

        if self.total_angle >= 2 * math.pi:
            self._complete = True

        return self._pose_at(self.theta)

    def _pose_at(self, theta: float) -> tuple[list[float], list[float], bool]:
        x = self.cx + self.radius * math.cos(theta)
        y = self.cy + self.radius * math.sin(theta)
        z = self.altitude

        # Face inward toward center
        yaw = math.atan2(self.cy - y, self.cx - x)
        quat_xyzw = Rotation.from_euler("z", yaw).as_quat().tolist()

        return [x, y, z], quat_xyzw, self._complete

    @property
    def is_complete(self) -> bool:
        return self._complete


def load_flight_config(path: str | Path) -> dict[str, Any]:
    """Load flight_path.yaml and return the raw dict."""
    with open(path) as f:
        return yaml.safe_load(f)


def make_follower(config: dict[str, Any]) -> CircularOrbitFollower:
    """Create a CircularOrbitFollower from a flight_path config dict."""
    orbit = config["orbit"]
    cx, cy = orbit["center"]
    radius = orbit["radius"]
    altitude = orbit["altitude"]
    speed = orbit["speed"]
    omega = speed / radius  # rad/s
    if orbit.get("direction", "ccw") == "cw":
        omega = -omega
    return CircularOrbitFollower(cx=cx, cy=cy, radius=radius, altitude=altitude, omega=omega)


def main() -> None:
    """Standalone test: print waypoints around the orbit."""
    import argparse

    parser = argparse.ArgumentParser(description="Test circular orbit path follower")
    parser.add_argument("--config", default="config/flight_path.yaml", help="Path to flight_path.yaml")
    args = parser.parse_args()

    config = load_flight_config(args.config)
    follower = make_follower(config)
    num_waypoints = config["orbit"].get("num_waypoints", 360)

    # Compute dt to produce exactly num_waypoints over one full orbit
    circumference = 2 * math.pi * follower.radius
    total_time = circumference / config["orbit"]["speed"]
    dt = total_time / num_waypoints

    print(f"Orbit: center=({follower.cx}, {follower.cy}), r={follower.radius}m, "
          f"alt={follower.altitude}m, omega={follower.omega:.4f} rad/s")
    print(f"Total time: {total_time:.1f}s, dt={dt:.4f}s, {num_waypoints} waypoints")
    print(f"{'step':>5} {'x':>8} {'y':>8} {'z':>6} {'qx':>7} {'qy':>7} {'qz':>7} {'qw':>7} {'done'}")

    for i in range(num_waypoints + 10):  # overshoot to show completion
        pos, quat, done = follower.step(dt)
        if i % 36 == 0 or done:  # print every 36th + completion
            print(f"{i:5d} {pos[0]:8.3f} {pos[1]:8.3f} {pos[2]:6.2f} "
                  f"{quat[0]:7.4f} {quat[1]:7.4f} {quat[2]:7.4f} {quat[3]:7.4f} {done}")
        if done and i > num_waypoints:
            break


if __name__ == "__main__":
    main()

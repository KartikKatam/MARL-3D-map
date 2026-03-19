"""ROS 2 node that saves cuVSLAM landmark cloud as PLY after orbit completes.

Subscribes to:
  /visual_slam/vis/landmarks_cloud (PointCloud2, BEST_EFFORT)
  /path_complete (Bool)

Writes ASCII PLY to output/maps/ after a configurable delay post-completion.
No open3d dependency — parses PointCloud2 manually with struct.
"""

from __future__ import annotations

import os
import struct
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class MapSaver(Node):
    def __init__(self) -> None:
        super().__init__("map_saver")

        self.declare_parameter("output_dir", "output/maps")
        self.declare_parameter("save_delay_sec", 3.0)

        self._output_dir = Path(self.get_parameter("output_dir").get_parameter_value().string_value)
        self._save_delay = self.get_parameter("save_delay_sec").get_parameter_value().double_value

        self._latest_cloud: PointCloud2 | None = None
        self._path_complete = False
        self._saved = False

        # cuVSLAM publishes with BEST_EFFORT/VOLATILE
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.create_subscription(
            PointCloud2,
            "/visual_slam/vis/landmarks_cloud",
            self._on_cloud,
            best_effort_qos,
        )

        self.create_subscription(
            Bool,
            "/path_complete",
            self._on_path_complete,
            10,
        )

        self.get_logger().info(f"MapSaver ready. Output dir: {self._output_dir}")

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg
        self.get_logger().info(
            f"Cloud received: {msg.width * msg.height} points",
            throttle_duration_sec=5.0,
        )

    def _on_path_complete(self, msg: Bool) -> None:
        if msg.data and not self._path_complete:
            self._path_complete = True
            self.get_logger().info(
                f"Path complete signal received. Saving in {self._save_delay}s..."
            )
            self.create_timer(self._save_delay, self._save_map)

    def _save_map(self) -> None:
        if self._saved:
            return
        self._saved = True

        if self._latest_cloud is None:
            self.get_logger().error("No cloud data received — cannot save map.")
            rclpy.shutdown()
            return

        points = self._parse_pointcloud2(self._latest_cloud)
        if not points:
            self.get_logger().error("Parsed 0 points from cloud — cannot save map.")
            rclpy.shutdown()
            return

        self._output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        ply_path = self._output_dir / f"map_{timestamp}.ply"

        self._write_ply(ply_path, points)
        self.get_logger().info(f"Saved {len(points)} points to {ply_path}")
        rclpy.shutdown()

    @staticmethod
    def _parse_pointcloud2(msg: PointCloud2) -> list[tuple[float, float, float]]:
        """Extract XYZ points from a PointCloud2 message.

        Handles both organized and unorganized clouds.
        Assumes x, y, z are the first three FLOAT32 fields.
        """
        # Find x, y, z field offsets
        field_map = {f.name: f.offset for f in msg.fields}
        if not all(k in field_map for k in ("x", "y", "z")):
            return []

        ox, oy, oz = field_map["x"], field_map["y"], field_map["z"]
        point_step = msg.point_step
        data = bytes(msg.data)
        num_points = msg.width * msg.height

        points: list[tuple[float, float, float]] = []
        for i in range(num_points):
            offset = i * point_step
            x = struct.unpack_from("<f", data, offset + ox)[0]
            y = struct.unpack_from("<f", data, offset + oy)[0]
            z = struct.unpack_from("<f", data, offset + oz)[0]
            # Skip NaN/inf points
            if all(abs(v) < 1e6 for v in (x, y, z)):
                points.append((x, y, z))

        return points

    @staticmethod
    def _write_ply(path: Path, points: list[tuple[float, float, float]]) -> None:
        """Write ASCII PLY file. No dependencies."""
        # Atomic write via tempfile
        import tempfile

        tmp_path = None
        try:
            fd, tmp_path = tempfile.mkstemp(
                suffix=".ply", dir=path.parent, prefix=".map_"
            )
            with os.fdopen(fd, "w") as f:
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {len(points)}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("end_header\n")
                for x, y, z in points:
                    f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
            os.replace(tmp_path, path)
        except Exception:
            if tmp_path and os.path.exists(tmp_path):
                os.unlink(tmp_path)
            raise


def main() -> None:
    rclpy.init()
    node = MapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

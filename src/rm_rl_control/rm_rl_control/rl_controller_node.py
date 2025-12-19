import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformException
from tf2_ros.transform_listener import TransformListener

try:
    # ROS2: provided by tf2_geometry_msgs for PoseStamped transforms
    from tf2_geometry_msgs import do_transform_pose  # type: ignore
except Exception:  # pragma: no cover
    try:  # pragma: no cover
        from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose  # type: ignore
    except Exception:  # pragma: no cover
        do_transform_pose = None


@dataclass
class Obs:
    goal_rel_xy: Tuple[float, float]  # in base_frame
    gravity_body: Tuple[float, float, float]  # unit-ish vector in base_frame
    scan_ranges: Sequence[float]


def _quat_conj(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def _quat_mul(
    a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]
) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_vec_by_quat_inv(
    q: Tuple[float, float, float, float], v: Tuple[float, float, float]
) -> Tuple[float, float, float]:
    """
    Compute v_body = R(q)^T * v_world using quaternion algebra.
    q is assumed to be (x, y, z, w).
    """
    vx, vy, vz = v
    vq = (vx, vy, vz, 0.0)
    q_conj = _quat_conj(q)
    # q^-1 == q* / |q|^2; we skip normalization here (Gazebo IMU is usually normalized).
    out = _quat_mul(_quat_mul(q_conj, vq), q)
    return (out[0], out[1], out[2])


class RlControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("rl_controller")

        self.declare_parameter("use_sim_time", True)
        self.declare_parameter("control_rate_hz", 10.0)

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/livox/imu")
        self.declare_parameter("goal_topic", "/rl_goal")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_chassis")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("goal_tolerance_m", 0.3)

        # Fallback (non-RL) controller gains
        self.declare_parameter("kp_v", 0.8)
        self.declare_parameter("kp_w", 2.0)

        self.declare_parameter("max_vx", 1.5)
        self.declare_parameter("max_vy", 1.5)
        self.declare_parameter("max_wz", 3.0)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.goal_tolerance_m = float(self.get_parameter("goal_tolerance_m").value)
        self.kp_v = float(self.get_parameter("kp_v").value)
        self.kp_w = float(self.get_parameter("kp_w").value)
        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_scan: Optional[LaserScan] = None
        self.last_imu: Optional[Imu] = None
        self.last_goal: Optional[PoseStamped] = None

        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.create_subscription(Imu, self.imu_topic, self._on_imu, 50)
        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        rate = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 1e-3), self._on_timer)

        self.get_logger().info(
            f"rl_controller started. scan={self.scan_topic} imu={self.imu_topic} "
            f"goal={self.goal_topic} cmd={self.cmd_vel_topic} base_frame={self.base_frame} "
            f"rate={rate}Hz"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def _on_imu(self, msg: Imu) -> None:
        self.last_imu = msg

    def _on_goal(self, msg: PoseStamped) -> None:
        self.last_goal = msg

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _transform_goal_to_base(self, goal: PoseStamped) -> Optional[PoseStamped]:
        # If already in base_frame (or frame empty), treat as relative pose.
        frame = goal.header.frame_id.strip()
        if frame == "" or frame == self.base_frame:
            if frame == "":
                goal.header.frame_id = self.base_frame
            return goal

        stamp = goal.header.stamp
        if int(stamp.sec) == 0 and int(stamp.nanosec) == 0:
            t = Time()
        else:
            t = Time.from_msg(stamp)

        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=self.base_frame,
                source_frame=frame,
                time=t,
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            # Fallback: try latest available transform
            try:
                tf = self.tf_buffer.lookup_transform(
                    target_frame=self.base_frame,
                    source_frame=frame,
                    time=Time(),
                    timeout=Duration(seconds=0.2),
                )
            except TransformException as ex:
                self.get_logger().warn(f"TF unavailable {frame}->{self.base_frame}: {ex}")
                return None

        if do_transform_pose is None:
            self.get_logger().error("tf2_geometry_msgs is unavailable; cannot transform goal PoseStamped.")
            return None

        out = do_transform_pose(goal, tf)
        out.header.frame_id = self.base_frame
        return out

    def _compute_gravity_body(self, imu: Imu) -> Tuple[float, float, float]:
        q = imu.orientation
        quat = (q.x, q.y, q.z, q.w)
        g_world = (0.0, 0.0, -1.0)
        gx, gy, gz = _rotate_vec_by_quat_inv(quat, g_world)

        n = math.sqrt(gx * gx + gy * gy + gz * gz)
        if n > 1e-6:
            return (gx / n, gy / n, gz / n)
        return (0.0, 0.0, -1.0)

    def _clip(self, v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    def _fallback_policy(self, obs: Obs) -> Tuple[float, float, float]:
        x, y = obs.goal_rel_xy
        dist = math.hypot(x, y)
        if dist < self.goal_tolerance_m:
            return (0.0, 0.0, 0.0)

        desired_yaw = math.atan2(y, x)
        vx = self.kp_v * x
        vy = self.kp_v * y
        wz = self.kp_w * desired_yaw

        vx = self._clip(vx, -self.max_vx, self.max_vx)
        vy = self._clip(vy, -self.max_vy, self.max_vy)
        wz = self._clip(wz, -self.max_wz, self.max_wz)
        return (vx, vy, wz)

    def _on_timer(self) -> None:
        if self.last_scan is None or self.last_imu is None or self.last_goal is None:
            self._stop()
            return

        goal_in_base = self._transform_goal_to_base(self.last_goal)
        if goal_in_base is None:
            self._stop()
            return

        x_rel = float(goal_in_base.pose.position.x)
        y_rel = float(goal_in_base.pose.position.y)

        gravity_body = self._compute_gravity_body(self.last_imu)
        obs = Obs(goal_rel_xy=(x_rel, y_rel), gravity_body=gravity_body, scan_ranges=self.last_scan.ranges)

        # TODO: Replace this with your RL policy inference:
        vx, vy, wz = self._fallback_policy(obs)

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        self.cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = RlControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


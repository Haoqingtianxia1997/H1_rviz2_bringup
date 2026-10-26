#!/usr/bin/env python3
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import JointState
from unitree_go.msg import LowState

def pick_attr(obj, names):
    for n in names:
        if hasattr(obj, n):
            return n
    return None

class LowStateToJointStates(Node):
    """
    Subscribe:  /lowstate   (unitree_go/msg/LowState)  [RELIABLE]
    Publish:    /joint_states (sensor_msgs/msg/JointState) [BEST_EFFORT]
    """
    def __init__(self):
        super().__init__("lowstate_to_joint_states")

        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("joint_map_yaml", "")

        lowstate_topic = self.get_parameter("lowstate_topic").value
        joint_states_topic = self.get_parameter("joint_states_topic").value
        joint_map_yaml = self.get_parameter("joint_map_yaml").value

        # ---- Load joint mapping ----
        # YAML format:
        # joints:
        #   - {name: "left_hip_yaw_joint",  index: 0}
        #   - {name: "left_hip_roll_joint", index: 1}
        self.joint_names = []
        self.motor_indices = []

        # ---- Joint position offsets ----
        # These offsets will be ADDED to the raw motor position before publishing.
        self.joint_offsets = {
            "left_shoulder_pitch_joint": 0.0,
            "left_shoulder_roll_joint":  0.0,
            "left_shoulder_yaw_joint":   0.0,
            "left_elbow_joint":          0.0,
            "right_shoulder_pitch_joint": 0.0,
            "right_shoulder_roll_joint": 0.0,
            "right_shoulder_yaw_joint":  0.0,
            "right_elbow_joint":         0.0,
        }

        if joint_map_yaml:
            with open(joint_map_yaml, "r") as f:
                data = yaml.safe_load(f)
            for item in data.get("joints", []):
                self.joint_names.append(str(item["name"]))
                self.motor_indices.append(int(item["index"]))
        else:
            self.get_logger().warn(
                "joint_map_yaml is empty. Publishing empty JointState names -> robot won't move in RViz. "
                "Provide a mapping YAML."
            )

        # ---- QoS: LowState publisher is often RELIABLE depth=1 (bare DDS app) ----
        lowstate_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- QoS: joint_states is sensor-like; robot_state_publisher subscribes BEST_EFFORT in your setup ----
        joint_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub = self.create_publisher(JointState, joint_states_topic, joint_qos)
        self.sub = self.create_subscription(LowState, lowstate_topic, self.cb, lowstate_qos)
        self.offset_sub = self.create_subscription(JointState, "/joint_offsets", self.offset_cb, 1)  # Optional dynamic offsets

        # Detect motor array field name once
        self.motor_array_field = None
        self.q_field = None
        self.dq_field = None

        self.get_logger().info(f"Subscribing {lowstate_topic} (RELIABLE) -> Publishing {joint_states_topic} (BEST_EFFORT)")
        if joint_map_yaml:
            self.get_logger().info(f"Loaded joint mapping: {len(self.joint_names)} joints from {joint_map_yaml}")

    def _init_schema(self, msg: LowState) -> bool:
        if self.motor_array_field is None:
            self.motor_array_field = pick_attr(msg, ["motor_state", "motorState", "motor", "motors"])
            if self.motor_array_field is None:
                self.get_logger().error(f"Cannot find motor array field in LowState. Fields={msg.get_fields_and_field_types()}")
                return False

            arr = getattr(msg, self.motor_array_field)
            if len(arr) == 0:
                self.get_logger().error("LowState motor array is empty.")
                return False

            m0 = arr[0]
            self.q_field = pick_attr(m0, ["q", "position"])
            self.dq_field = pick_attr(m0, ["dq", "velocity"])  # optional

            if self.q_field is None:
                self.get_logger().error("Cannot find q/position field in MotorState.")
                return False

            self.get_logger().info(
                f"Detected schema: motor_array_field='{self.motor_array_field}', q_field='{self.q_field}', dq_field='{self.dq_field}', motors={len(arr)}"
            )
        return True

    def offset_cb(self, msg: JointState):
        """
        Callback to dynamically update joint offsets from /joint_offsets topic.
        The incoming JointState message should contain joint names and their desired offsets in the position field.
        """
        for name, offset in zip(msg.name, msg.position):
            self.joint_offsets[name] = offset
            self.get_logger().info(f"Updated offset for '{name}': {offset}")

    def cb(self, msg: LowState):
        if not self._init_schema(msg):
            return

        if not self.joint_names:
            return

        arr = getattr(msg, self.motor_array_field)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names

        pos = []
        vel = []
        for name, idx in zip(self.joint_names, self.motor_indices):
            if idx < 0 or idx >= len(arr):
                pos.append(0.0)
                if self.dq_field:
                    vel.append(0.0)
                continue
            m = arr[idx]
            raw_pos = float(getattr(m, self.q_field))
            offset = self.joint_offsets.get(name, 0.0)
            pos.append(raw_pos + offset)
            if self.dq_field:
                vel.append(float(getattr(m, self.dq_field)))

        js.position = pos
        if self.dq_field and len(vel) == len(pos):
            js.velocity = vel

        self.pub.publish(js)

def main():
    rclpy.init()
    node = LowStateToJointStates()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
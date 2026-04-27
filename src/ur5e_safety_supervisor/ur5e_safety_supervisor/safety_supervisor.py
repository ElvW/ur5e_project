import csv
import os
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


class SafetySupervisor(Node):
    """Safety supervisor for UR5e robot arm."""

    def __init__(self):
        """Initialize the Safety Supervisor node."""
        super().__init__('safety_supervisor')

        self.safety_active = True

        # Workspace bounds
        self.min_x, self.max_x = -0.2, 0.7  # Front/Back
        self.min_y, self.max_y = -0.5, 0.5  # Left/Right
        # Up/Down (0.1 prevents hitting the table)
        self.min_z, self.max_z = 0.1, 1.1

        self.max_joint_vel = 1.5

        # Limits in Radians (+/- 180 degrees is roughly +/- 3.14)
        self.joint_limits = {
            'shoulder_pan_joint': [-3.14, 3.14],
            'shoulder_lift_joint': [-3.14, 0.0],  # Prevent lifting too far back
            'elbow_joint': [-3.14, 3.14],
            'wrist_1_joint': [-3.14, 3.14],
            'wrist_2_joint': [-3.14, 3.14],
            'wrist_3_joint': [-3.14, 3.14]
        }

        self.workspace_dir = '/ros2_ws'
        self.log_file = os.path.join(
            self.workspace_dir, 'safety_events_log.csv'
        )

        self.init_log()

        # 1. Listener for Joint States (for speed/limits)
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # 2. Setup TF Buffer (to calculate where tool0 is in 3D space)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. Timer to check safety rules (e.g., 20 times per second)
        self.timer = self.create_timer(0.05, self.check_safety)

        self.get_logger().info('Safety Supervisor active and monitoring...')

        # Wait for MoveIt to wake up before starting
        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Waiting for MoveIt Action Server...')
        self.move_client.wait_for_server()
        self.get_logger().info('MoveIt Server Connected!')

        # Track the active goal so we can kill it
        self.current_goal_handle = None

        self.get_logger().info('Safety Supervisor: Kill-switch initialized.')


    def init_log(self):
        """Create the log file and write headers if it doesn't exist."""
        if not os.path.exists(self.log_file):
            with open(self.log_file, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(
                    ['Timestamp', 'Event_Type', 'Details', 'Latency_ms',
                     'Recovery_Status']
                )

    def log_violation(self, v_type, details, latency=0.0):
        """Log a safety violation to CSV file."""
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        with open(self.log_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, v_type, details, f'{latency:.3f}'])
        self.get_logger().warn(f'💾 Logged {v_type} to CSV.')


    def joint_callback(self, msg):
        """Monitor joint states for speed and position violations."""
        # msg.name contains joint names
        # msg.velocity contains the actual speeds
        violation = False
        offending_joint = ''
        max_observed = 0.0

        for i in range(len(msg.name)):
            name = msg.name[i]
            pos = msg.position[i]
            # Use absolute value (direction doesn't matter)
            vel = abs(msg.velocity[i])

            if vel > self.max_joint_vel:
                violation = True
                self.get_logger().error(
                    f'🛑 SPEED VIOLATION: {name} moving at {vel:.2f} rad/s'
                )
                break

            if name in self.joint_limits:
                low, high = self.joint_limits[name]
                if not (low <= pos <= high):
                    violation = True
                    self.get_logger().error(
                        f'🛑 JOINT LIMIT BREACH: {name} at {pos:.2f} rad'
                    )
                    break

        if violation:
            detection_time = self.get_clock().now()
            event_time = rclpy.time.Time.from_msg(msg.header.stamp)
            # Convert to milliseconds
            latency = (detection_time - event_time).nanoseconds / 1e6
            self.get_logger().info(f'⏱️ Detection Latency: {latency:.3f} ms')
            self.log_violation('JOINT BREACH', f'{name} violation', latency)
            self.execute_hard_stop()


    def check_safety(self):

        try:

            # Look up the transform from base_link to tool0

            trans = self.tf_buffer.lookup_transform(

                'base_link_inertia',

                'wrist_3_link',

                rclpy.time.Time(),

                timeout=Duration(seconds=0.1))

           

            # Extract the current X, Y, Z

            curr_x = trans.transform.translation.x

            curr_y = trans.transform.translation.y

            curr_z = trans.transform.translation.z


            self.validate_bounds(curr_x, curr_y, curr_z)


        except Exception as e:

            # This happens if the robot hasn't started yet or TF isn't ready

            self.get_logger().warn(f"Waiting for TF: {str(e)}")


    def validate_bounds(self, x, y, z):

        violation = False

        msg = ""


        if not (self.min_x <= x <= self.max_x):

            violation = True

            msg += f"X violation ({x:.2f}) "

       

        if not (self.min_y <= y <= self.max_y):

            violation = True

            msg += f"Y violation ({y:.2f}) "


        if not (self.min_z <= z <= self.max_z):

            violation = True

            msg += f"Z violation ({z:.2f}) "


        if violation and self.safety_active:
            self.get_logger().error(f"⚠️ SAFETY BREACH: {msg}")
            self.log_violation("WORKSPACE_BREACH", msg, 0.0)
            self.safety_active = False
            self.execute_hard_stop()
        else:
            self.safety_active = True

    def execute_hard_stop(self):
        """Execute emergency hard stop by canceling all active goals."""
        # This is the "Intervention"
        if self.move_client.server_is_ready():
            # Send a cancel request to the Action Server for any active goals
            self.get_logger().warn('Canceling all active MoveIt trajectories!')

            # The most direct 'Hard Stop' in simulation is sending
            # a cancel request to the current goal handle
            cancel_future = self.move_client.cancel_all_goals_async()
            cancel_future.add_done_callback(self.stop_confirmed)

    def stop_confirmed(self, future):
        """Callback to confirm emergency stop."""
        self.get_logger().info('✅ Emergency Stop Confirmed. Robot is locked.')


def main():
    """Main entry point for the Safety Supervisor node."""
    rclpy.init()
    node = SafetySupervisor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

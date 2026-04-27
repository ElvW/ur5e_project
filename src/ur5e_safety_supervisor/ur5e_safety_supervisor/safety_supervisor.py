import csv
import os
from datetime import datetime
from time import sleep

import rclpy
from action_msgs.srv import CancelGoal
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

        self.prev_positions = {}
        self.prev_time = None

        # Workspace bounds
        self.min_x, self.max_x = -0.2, 0.6  # Keep the arm mostly in front of its base
        self.min_y, self.max_y = -0.5, 0.5  # Prevent wide lateral swings
        self.min_z, self.max_z = 0.1, 0.9   # 0.1m floor prevents smashing into the table

        # Speed limit: 1.0 rad/s (~57 deg/s) is a standard collaborative speed
        self.max_joint_vel = 1.0

        # Limits in Radians (+/- 180 degrees is roughly +/- 3.14)
        self.joint_limits = {
            'shoulder_pan_joint': [-3.14, 3.14],
            'shoulder_lift_joint': [-3.14, 0.0],  # 0.0 prevents bending backwards into its own base
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
        # DIRECT Service Clients (Bypasses the Action Client to force a stop on the drivers)
        self.cancel_moveit = self.create_client(
            CancelGoal, '/move_action/_action/cancel_goal')
        
        self.cancel_scaled_driver = self.create_client(
            CancelGoal, '/scaled_joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal')
            
        self.cancel_standard_driver = self.create_client(
            CancelGoal, '/joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal')
        
        self.get_logger().info('Waiting for MoveIt Action Server...')
        self.move_client.wait_for_server()
        self.get_logger().info('MoveIt Server Connected!')

        # Track the active goal so we can kill it
        self.current_goal_handle = None

        self.get_logger().info("Waiting for everything to be ready...")

        sleep(5)

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
        now = self.get_clock().now()
        
        # Initialize history on the very first message
        if self.prev_time is None:
            self.prev_time = now
            for i, name in enumerate(msg.name):
                self.prev_positions[name] = msg.position[i]
            return

        # Calculate time elapsed in seconds
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0001:  # Prevent division by zero
            return

        violation = False
        offending_joint = ""

        for i in range(len(msg.name)):
            name = msg.name[i]
            pos = msg.position[i]
            
            # 1. Calculate TRUE velocity manually (Δx / Δt)
            if name in self.prev_positions:
                calculated_vel = abs(pos - self.prev_positions[name]) / dt
            else:
                calculated_vel = 0.0
            
            # Update history for the next loop
            self.prev_positions[name] = pos

            # 2. Check Speed Limit
            if calculated_vel > self.max_joint_vel:
                violation = True
                offending_joint = f"{name} (speed: {calculated_vel:.2f} rad/s)"
                break

            # 3. Check Position Limit
            if name in self.joint_limits:
                low, high = self.joint_limits[name]
                if not (low <= pos <= high):
                    violation = True
                    offending_joint = f"{name} (limit: {pos:.2f} rad)"
                    break

        # Update the clock for the next cycle
        self.prev_time = now

        if violation and self.safety_active:
            event_time = rclpy.time.Time.from_msg(msg.header.stamp)
            latency_ms = abs((now - event_time).nanoseconds) / 1e6
            
            self.get_logger().error(f"🛑 BREACH: {offending_joint}")
            self.get_logger().info(f"⏱️ Detection Latency: {latency_ms:.3f} ms")
            
            self.log_violation("JOINT_BREACH", offending_joint, latency_ms)
            self.execute_hard_stop()
            self.safety_active = False


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
            self.execute_hard_stop()

    def execute_hard_stop(self):
        """Execute emergency hard stop by killing the Planner AND the Drivers."""
        self.get_logger().warn('🛑 EMERGENCY STOP: Pulling the hardware brakes!')
        
        request = CancelGoal.Request()
        
        # 1. Stop the Primary Driver (Usually the one moving the UR5e)
        if self.cancel_scaled_driver.wait_for_service(timeout_sec=0.1):
            self.cancel_scaled_driver.call_async(request)
            
        # 2. Stop the Fallback Driver
        if self.cancel_standard_driver.wait_for_service(timeout_sec=0.1):
            self.cancel_standard_driver.call_async(request)
            
        # 3. Stop MoveIt (So it knows the plan was aborted)
        if self.cancel_moveit.wait_for_service(timeout_sec=0.1):
            self.cancel_moveit.call_async(request)

        self.get_logger().info("✅ Brakes applied to all controllers.")


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

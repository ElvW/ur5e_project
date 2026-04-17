#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import yaml
import os
import time
import subprocess
import signal
import math

# ROS 2 and MoveIt messages
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import WorkspaceParameters, PositionIKRequest

# 3. In raw ROS 2, we have to put the pose inside a "Constraint" envelope
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import BoundingVolume

class ExperimentRunner(Node):
    def __init__(self):
        super().__init__('experiment_runner')
        self.get_logger().info("Starting A1 Experiment Automation with MoveIt...")
        
        # 1. Setup Paths
        self.config_path = '/ros2_ws/src/ur5e_experiment_automation/config/experiments.yaml'
        self.bag_dir = '/ros2_ws/src/ur5e_project/bags'
        os.makedirs(self.bag_dir, exist_ok=True)
        
        # 2. Setup MoveIt Action Client
        # This creates a direct pipeline to the MoveIt brain
        self.move_action_client = ActionClient(self, MoveGroup, 'recognize_poses') 
        # Note: Depending on your specific UR simulation setup, the action server name might be 'move_action'
        self.move_action_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.scenarios = self.load_config()
        self.total_repetitions = 10
        self.bag_process = None
        
        # Wait for MoveIt to wake up before starting
        self.get_logger().info("Waiting for MoveIt Action Server...")
        self.move_action_client.wait_for_server()
        self.get_logger().info("MoveIt Server Connected!")

        self.run_experiments()

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Converts human-readable Roll/Pitch/Yaw into a MoveIt Quaternion"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def load_config(self):
        if not os.path.exists(self.config_path):
            self.get_logger().error(f"YAML file not found at {self.config_path}")
            return {}
        with open(self.config_path, 'r') as file:
            return yaml.safe_load(file).get('scenarios', {})

    def start_recording(self, bag_name):
        """Adım 3: Starts the background camera"""
        bag_path = os.path.join(self.bag_dir, bag_name)
        if os.path.exists(bag_path):
            self.get_logger().warn(f"Bag {bag_name} exists! Skipping to prevent crash.")
            return

        topics = "/joint_states /tf /tf_static /joint_trajectory_controller/joint_trajectory /rosout"
        command = f"ros2 bag record -o {bag_path} {topics}"
        
        self.get_logger().info(f"[*] RECORDING START: {bag_name}")
        self.bag_process = subprocess.Popen(command, shell=True, executable='/bin/bash', preexec_fn=os.setsid)

    def stop_recording(self):
        """Adım 3: Saves the recording safely"""
        if self.bag_process:
            self.get_logger().info("[*] RECORDING STOP: Saving file...\n")
            os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
            self.bag_process.wait()
            self.bag_process = None
            time.sleep(1.0)

    def execute_move(self, target_pose, timeout):
        """The actual MoveIt Muscles"""
        self.get_logger().info(f"Commanding physical move to: {target_pose}")
        
        x, y, z, roll, pitch, yaw = target_pose
        
        # 1. Build the MoveIt Goal Request
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.allowed_planning_time = float(timeout)
        
        # 2. Setup the target pose
        target = Pose()
        target.position = Point(x=float(x), y=float(y), z=float(z))
        target.orientation = self.euler_to_quaternion(roll, pitch, yaw)

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose = target

        # Set up position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "tool0"  # The end of the robot arm
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
        # Create a tiny bounding box for the target
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.SPHERE
        shape.dimensions = [0.01]  # 1 cm tolerance
        pos_constraint.constraint_region.primitives.append(shape)
        pos_constraint.constraint_region.primitive_poses.append(target)
        pos_constraint.weight = 1.0

        # Package it up
        constraint = Constraints()
        constraint.position_constraints.append(pos_constraint)
        goal_msg.request.goal_constraints.append(constraint)

        # 4. SEND THE GOAL TO MOVEIT
        self.get_logger().info("Sending trajectory to MoveIt...")
        send_goal_future = self.move_action_client.send_goal_async(goal_msg)
        
        # Pause the script until MoveIt accepts the math
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("❌ MoveIt rejected the trajectory! (Coordinate might be inside the table)")
            return False
            
        self.get_logger().info("Trajectory accepted. Robot is moving...")
        
        # Pause the script until the physical robot finishes moving
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        self.get_logger().info("-> SUCCESS: Robot arm reached coordinate.\n")
        return True

    def run_experiments(self):
        for rep in range(1, self.total_repetitions + 1):
            self.get_logger().info(f"========== REPETITION {rep}/{self.total_repetitions} ==========")
            
            for scenario_name, params in self.scenarios.items():
                self.get_logger().info(f"--- Running Scenario: {scenario_name} ---")
                
                bag_name = f"exp_{scenario_name}_rep{rep}"
                self.start_recording(bag_name)
                
                timeout = params.get('timeout', 10.0)
                if 'target_pose' in params:
                    self.execute_move(params['target_pose'], timeout)
                elif 'waypoints' in params:
                    for wp in params['waypoints']:
                        self.execute_move(wp, timeout)
                elif 'pick_approach' in params:
                    self.execute_move(params['pick_approach'], timeout)
                    self.execute_move(params['pick_grasp'], timeout)
                    self.execute_move(params['place_approach'], timeout)
                    self.execute_move(params['place_drop'], timeout)
                
                self.stop_recording()
                    
            self.get_logger().info(f"========== END REPETITION {rep} ==========\n")

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunner()
    # Let it spin to handle MoveIt action callbacks
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
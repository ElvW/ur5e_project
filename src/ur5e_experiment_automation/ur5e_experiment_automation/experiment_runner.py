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
from datetime import datetime
import threading

# ROS 2 and MoveIt messages
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import WorkspaceParameters, PositionIKRequest

# 3. In raw ROS 2, we have to put the pose inside a "Constraint" envelope
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import BoundingVolume
from moveit_msgs.msg import CollisionObject

class ExperimentRunner(Node):
    def __init__(self):
        super().__init__('experiment_runner')
        self.get_logger().info("Starting Experiment Automation with MoveIt...")
        
        # 1. Setup Paths
        self.config_path = '/ros2_ws/src/ur5e_experiment_automation/config/experiments.yaml'
        self.bag_dir = f"/ros2_ws/recordings/bags_{now.strftime('%Y-%m-%d_%H-%M-%S')}"
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

        # Create a publisher to talk to the MoveIt environment
        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        time.sleep(1.0) # Give the publisher a second to wake up
        self.add_virtual_floor()

    def view_movements(self):
        """Prints out the loaded YAML scenarios safely without moving the arm"""
        print("\n" + "-"*40)
        print(" LOADED MOVEMENTS (FROM YAML)")
        print("-"*40)
        for name, params in self.scenarios.items():
            print(f"Scenario: {name}")
            if 'target_pose' in params:
                print(f"  Type: Point-to-Point")
                print(f"  Target: {params['target_pose']}")
            elif 'waypoints' in params:
                print(f"  Type: Waypoints ({len(params['waypoints'])} points)")
                for i, wp in enumerate(params['waypoints']):
                    print(f"    {i+1}: {wp}")
            elif 'pick_approach' in params:
                print(f"  Type: Pick and Place")
                print(f"    Approach: {params['pick_approach']}")
                print(f"    Grasp:    {params['pick_grasp']}")
                print(f"    Drop:     {params['place_drop']}")
        print("-"*40 + "\n")

    def main_menu(self):
        """The interactive Command Line UI"""
        
        print(datetime.now())
        # Brief pause to let the ROS 2 boot-up text finish printing
        time.sleep(1.0) 
        
        while rclpy.ok():
            print("\n" + "="*45)
            print(" 🤖 EXPERIMENT AUTOMATION MENU ")
            print("="*45)
            print("[1] Run all experiments (with recording)")
            print("[2] View loaded movements (List Coordinates)")
            print("[3] Exit")
            print("="*45)
            
            choice = input("Enter your choice (1/2/3): ")
            
            if choice == '1':
                self.run_experiments()
            elif choice == '2':
                self.view_movements()
            elif choice == '3':
                print("\nShutting down safely. Goodbye!")
                # Sends a safe Ctrl+C signal to the main ROS 2 thread
                os.kill(os.getpid(), signal.SIGINT) 
                break
            else:
                print("\nInvalid choice. Please enter 1, 2, or 3.")

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
        # 3: Starts the background camera
        bag_path = os.path.join(self.bag_dir, bag_name)
        if os.path.exists(bag_path):
            self.get_logger().warn(f"Bag {bag_name} exists! Skipping to prevent crash.")
            return

        topics = "/joint_states /tf /tf_static /joint_trajectory_controller/joint_trajectory /rosout"
        command = f"ros2 bag record -o {bag_path} {topics}"
        
        self.get_logger().info(f"[*] RECORDING START: {bag_name}")
        self.bag_process = subprocess.Popen(command, shell=True, executable='/bin/bash', preexec_fn=os.setsid)

        # Wait for the camera to boot up
        time.sleep(2.0)

    def stop_recording(self):
        """Adım 3: Saves the recording safely"""
        if self.bag_process:
            self.get_logger().info("[*] RECORDING STOP: Saving file...\n")
            os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
            self.bag_process.wait()
            self.bag_process = None
            time.sleep(5.0)

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
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.1)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("❌ MoveIt rejected the trajectory! (Coordinate might be inside the table)")
            return False
            
        self.get_logger().info("Trajectory accepted. Robot is moving...")
        
        # Pause the script until the physical robot finishes moving
        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.1)
        
        self.get_logger().info("-> SUCCESS: Robot arm reached coordinate.\n")
        return True

    def reset_joints(self):
        """Forces the 6 motors back to a safe, upright 'Ready' position"""
        self.get_logger().info("Untangling robot: Returning to absolute Joint Home...")
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.allowed_planning_time = 10.0
        
        # The official UR5e joint names
        joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Safe "upright" joint positions in radians [0, -90, 90, -90, -90, 0]
        home_joints = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
        
        # Build the Joint Constraints
        constraint = Constraints()
        for i in range(6):
            jc = JointConstraint()
            jc.joint_name = joint_names[i]
            jc.position = home_joints[i]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraint.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(constraint)
        
        # Send the Joint command to MoveIt
        send_goal_future = self.move_action_client.send_goal_async(goal_msg)
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.1)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("❌ Failed to untangle joints!")
            return False
            
        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.1)
        
        self.get_logger().info("-> SUCCESS: Motors completely reset.\n")
        time.sleep(1.0) # Brief pause to let physics settle
        return True
    
    def add_virtual_floor(self):
        """Spawns an invisible floor in MoveIt so the elbow stops crashing"""
        self.get_logger().info("Spawning virtual concrete floor in MoveIt...")
        
        # Create the floor object
        floor = CollisionObject()
        floor.id = "safety_floor"
        floor.header.frame_id = "base_link"
        
        # Define it as a large flat box (2m x 2m wide, 10cm thick)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0, 2.0, 0.1]
        
        # Place it so the top surface is exactly at Z = 0 (right under the robot base)
        box_pose = Pose()
        box_pose.position.x = 0.0
        box_pose.position.y = 0.0
        box_pose.position.z = -0.051 # Just barely below the metal base to avoid self-collision
        
        # Package it up
        floor.primitives.append(box)
        floor.primitive_poses.append(box_pose)
        floor.operation = CollisionObject.ADD
        
        # Publish it to the MoveIt scene
        self.collision_pub.publish(floor)
        self.get_logger().info("Virtual floor active! The elbow is now safe.")

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
                    
            self.reset_joints()
                
            self.get_logger().info(f"========== END REPETITION {rep} ==========\n")

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunner()

    ui_thread = threading.Thread(target=node.main_menu, daemon=True)
    ui_thread.start()

    # Let it spin to handle MoveIt action callbacks
    try:    
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
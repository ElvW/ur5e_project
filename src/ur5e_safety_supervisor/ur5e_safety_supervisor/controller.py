import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus  # Imports the standard status codes

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PlanningOptions

class RobotController(Node):
    """A script to command the UR5e and listen for the final outcome."""
    
    def __init__(self):
        super().__init__('robot_controller')
        self.move_client = ActionClient(self, MoveGroup, 'move_action')

    def move_to_danger_zone(self):
        self.get_logger().info('Waiting for MoveIt Action Server...')
        self.move_client.wait_for_server()

        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'ur_manipulator'
        req.num_planning_attempts = 3
        req.allowed_planning_time = 5.0
        
        # 100% Speed to trigger the speed violation!
        req.max_velocity_scaling_factor = 1.0  
        req.max_acceleration_scaling_factor = 1.0

        # Massive 180-degree swing to keep the robot moving long enough to be stopped
        joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        target_positions = [3.14, -1.57, 1.57, -1.57, -1.57, 0.0]

        constraints = Constraints()
        for name, pos in zip(joint_names, target_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req.goal_constraints.append(constraints)
        goal_msg.request = req
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False

        # Send the goal and attach a callback for when the server responds
        self.get_logger().info('🚀 Sending goal...')
        send_goal_future = self.move_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when MoveIt decides to Accept or Reject the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by MoveIt.')
            return

        self.get_logger().info('Goal accepted! Robot is moving...')
        
        # Ask the server to notify us when the movement is completely finished
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Called when the movement is finished, aborted, or canceled."""
        result = future.result()
        status = result.status

        # Check the official ROS 2 Goal Status Enum
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error('❌ Action SUCCEEDED. The supervisor failed to stop it!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('✅ Action CANCELED! The Safety Supervisor successfully stopped the robot!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('⚠️ Action ABORTED! (MoveIt hit a collision internally)')
        else:
            self.get_logger().info(f'Action finished with unknown status code: {status}')
            
        # Shut down the controller automatically
        rclpy.shutdown()

def main():
    rclpy.init()
    node = RobotController()
    node.move_to_danger_zone()
    rclpy.spin(node) # Keeps the node alive to listen for callbacks

if __name__ == '__main__':
    main()
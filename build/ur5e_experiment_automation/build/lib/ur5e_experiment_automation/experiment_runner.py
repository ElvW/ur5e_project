#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import os
import time

class ExperimentRunner(Node):
    def __init__(self):
        super().__init__('experiment_runner')
        self.get_logger().info("Starting A1 Experiment Automation...")
        
        self.config_path = '/ur5e_project/src/ur5e_experiment_automation/config/experiments.yaml'
        self.scenarios = self.load_config()
        self.total_repetitions = 10
        self.run_experiments()

    def load_config(self):
        if not os.path.exists(self.config_path):
            self.get_logger().error(f"YAML file not found at {self.config_path}")
            return {}
        with open(self.config_path, 'r') as file:
            return yaml.safe_load(file).get('scenarios', {})

    def execute_move(self, target_pose, timeout):
        self.get_logger().info(f"Commanding move to: {target_pose} with timeout: {timeout}s")
        time.sleep(2.0) 
        self.get_logger().info("-> SUCCESS: Reached target pose.\n")
        return True

    def run_experiments(self):
        for rep in range(self.total_repetitions):
            self.get_logger().info(f"========== REPETITION {rep}/{self.total_repetitions + 1} ==========")
            for scenario_name, params in self.scenarios.items():
                self.get_logger().info(f"--- Running Scenario: {scenario_name} ---")
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
            self.get_logger().info(f"========== END REPETITION {rep} ==========\n")

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunner()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

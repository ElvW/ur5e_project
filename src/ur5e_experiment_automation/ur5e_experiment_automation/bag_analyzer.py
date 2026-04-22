import os
import sqlite3
import csv
import math
import rclpy
import yaml
from datetime import datetime 
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from rclpy.time import Time

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

class BagAnalyzer:
    def __init__(self):
        # Find the bags folder (adjust path if your bags are saved somewhere else)
        self.workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
        self.bag_dir = os.path.join(self.workspace_dir, 'recordings')

        self.config_path = os.path.join(self.workspace_dir, 'src', 'ur5e_experiment_automation', 'config', 'experiments.yaml')
        
        if os.path.exists(self.config_path):
            with open(self.config_path, 'r') as file:
                self.config = yaml.safe_load(file)
            print(f"Loaded experiment targets from YAML.")
        else:
            print(f"Warning: Could not find YAML at {self.config_path}")
            self.config = {}
        
        if not os.path.exists(self.bag_dir):
            print(f"Error: Could not find bags directory at {self.bag_dir}")
            return
            
        print(f"Analyzer initialized. Looking for databases in: {self.bag_dir}")

    def analyze_all_bags(self):
        all_results = []

        for folder_name in os.listdir(self.bag_dir):
            folder_path = os.path.join(self.bag_dir, folder_name)
            # ROS 2 creates a folder for each bag. Inside is the actual .db3 database.
            if os.path.isdir(folder_path):
                raw_folders = os.listdir(folder_path)

                sorted_folders = sorted(raw_folders, key=lambda x: int(x.split("rep")[-1]) if 'rep' in x else 0)

                for repetition in sorted_folders: 
                    rep_folder = os.path.join(folder_path, repetition)
                    db_file = os.path.join(rep_folder, f"{repetition}_0.db3")

                    if os.path.exists(db_file):
                        print(f"\nAnalyzing: {repetition}")

                        exp_name = repetition.split('_rep')[0] 
                        exp_name = exp_name.split('exp_')[-1]
            
                        metrics = self.extract_metrics(db_file, exp_name)

                        if metrics is not None:
                            metrics['Repetition'] = repetition
                            all_results.append(metrics)
            
            if all_results:
                now = datetime.now()
                results_dir = os.path.join(self.workspace_dir, 'results')

                os.makedirs(results_dir, exist_ok=True)

                now_dir = os.path.join(results_dir, now.strftime("%Y-%m-%d_%H-%M-%S"))

                os.makedirs(now_dir, exist_ok=True)

                csv_path = os.path.join(now_dir, f'experiment_results.csv')
                
                # Open a new CSV file
                with open(csv_path, 'w', newline='') as csvfile:
                    # Define the column headers
                    fieldnames = ['Repetition', 'Experiment', 'Duration (s)', 'Path Length (m)', 'Final X', 'Final Y', 'Final Z', 'Target Error (m)', 'Success']
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    
                    # Write the header row, then write all the data!
                    writer.writeheader()
                    for row in all_results:
                        writer.writerow(row)
                        
                print(f"\nSUCCESS! All data perfectly saved to: {csv_path}")
            
                # --- NEW: AUTOMATED REPORT & GRAPH GENERATION ---
                if plt is not None:
                    print("Generating automated report and graph...")
                    
                    report_path = os.path.join(now_dir, 'experiment_report.md')
                    with open(report_path, 'w') as f:
                        f.write("# Automated Experiment Report\n\n")
                        
                        # 1. GROUP THE DATA BY SCENARIO
                        experiments = {}
                        for row in all_results:
                            exp_name = row['Experiment']
                            if exp_name not in experiments:
                                experiments[exp_name] = []
                            experiments[exp_name].append(row)
                            
                        # 2. LOOP THROUGH EACH SCENARIO TO CREATE SEPARATE GRAPHS AND TABLES
                        for exp_name, exp_data in experiments.items():
                            f.write(f"## Scenario: {exp_name.upper()}\n\n")
                            
                            # Extract data ONLY for this specific scenario
                            reps = [row['Repetition'].split('_rep')[-1] for row in exp_data]
                            path_lengths = [row['Path Length (m)'] for row in exp_data]
                            durations = [row['Duration (s)'] for row in exp_data]
                            
                            # Draw a dual-axis graph for this scenario
                            fig, ax1 = plt.subplots(figsize=(10, 5))
                            
                            color = 'tab:blue'
                            ax1.set_xlabel('Repetition Number')
                            ax1.set_ylabel('Path Length (meters)', color=color)
                            ax1.bar(reps, path_lengths, color=color, alpha=0.6, label='Path Length')
                            ax1.tick_params(axis='y', labelcolor=color)
                            
                            ax2 = ax1.twinx()
                            color = 'tab:red'
                            ax2.set_ylabel('Duration (seconds)', color=color)
                            ax2.plot(reps, durations, color=color, marker='o', linewidth=2, label='Duration')
                            ax2.tick_params(axis='y', labelcolor=color)
                            
                            plt.title(f'Performance: {exp_name}')
                            fig.tight_layout()
                            
                            # Save the graph with a unique filename!
                            graph_filename = f'performance_graph_{exp_name}.png'
                            graph_path = os.path.join(now_dir, graph_filename)
                            plt.savefig(graph_path)
                            plt.close()
                            
                            # Write this scenario's section to the Markdown (.md) Report
                            f.write("### Performance Graph\n")
                            f.write(f"![Performance Graph](./{graph_filename})\n\n")
                            
                            f.write("### Results Table\n")
                            f.write("| Repetition | Duration | Path Length | Target Error | Success |\n")
                            f.write("|---|---|---|---|---|\n")
                            
                            for row in exp_data:
                                success_str = "Success" if row['Success'] else "Fail"
                                f.write(f"| {row['Repetition']} | {row['Duration (s)']}s | {row['Path Length (m)']}m | {row['Target Error (m)']}m | **{success_str}** |\n")
                            
                            # Add a dividing line between scenarios
                            f.write("\n---\n\n") 
                            
                    print(f"SUCCESS! Automated report saved to: {report_path}")
                    all_results = []

    def extract_metrics(self, db_path, exp_name):
        """Reads the SQLite database to calculate metrics"""
        try:
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()
            
            # 1. CALCULATE DURATION (Süre)
            # ROS 2 saves timestamps in nanoseconds. We grab the very first and very last message time.
            cursor.execute("SELECT min(timestamp), max(timestamp) FROM messages")
            timestamps = cursor.fetchone()
            
            if timestamps[0] and timestamps[1]:
                # Divide by 1,000,000,000 to convert nanoseconds to standard seconds
                start_time = timestamps[0] / 1e9 
                end_time = timestamps[1] / 1e9
                duration = end_time - start_time
                print(f"  -> Duration: {duration:.2f} seconds")
            else:
                print("  -> Error: Bag is empty or corrupted!")
                return
                
            # 2. IDENTIFY TOPICS
            # This looks up the ID numbers for the /joint_states and /tf topics
            cursor.execute("SELECT id, name FROM topics")
            topics = cursor.fetchall()
            topic_map = {name: topic_id for topic_id, name in topics}

            # 3. EXTRACT KINEMATICS AND CALCULATE MATH
            if '/tf' in topic_map:
                tf_id = topic_map['/tf']
                cursor.execute("SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (tf_id,))
                tf_msg_type = get_message('tf2_msgs/msg/TFMessage')
                
                tf_buffer = Buffer()
                trajectory_points = []
                error_printed = False
                
                # COMBINED LOOP: Play the tape and take a picture at the exact same time
                for row in cursor.fetchall():
                    binary_data = row[0]
                    msg = deserialize_message(binary_data, tf_msg_type)
                    
                    for transform in msg.transforms:
                        # 1. Feed the bone movement into the math engine
                        tf_buffer.set_transform(transform, "offline_bag")

                        # 2. The instant the wrist moves, take a 3D snapshot!
                        if transform.child_frame_id == 'wrist_3_link':
                            try:
                                # rclpy.time.Time() means "Give me the latest valid coordinates right now"
                                pose = tf_buffer.lookup_transform('base_link_inertia', 'wrist_3_link', rclpy.time.Time())
                                
                                x = - pose.transform.translation.x
                                y = - pose.transform.translation.y
                                z = pose.transform.translation.z
                                trajectory_points.append((x, y, z))
                            except Exception as e:
                                # It is normal for the first few frames to fail before the base_link data arrives
                                if not error_printed:
                                    # print(f"    (Debug: First frame skipped because: {e})")
                                    error_printed = True
                                pass
                
                # C. APPLY THE FORMULAS
                if len(trajectory_points) > 1:
                    path_length = 0.0
                    for i in range(len(trajectory_points) - 1):
                        p1 = trajectory_points[i]
                        p2 = trajectory_points[i+1]
                        
                        # 3D Euclidean Distance (Absolute Room Coordinates!)
                        distance = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)
                        path_length += distance
                    
                    print(f"  -> Path Length: {path_length:.4f} meters")
                    
                    final_pos = trajectory_points[-1]
                    print(f"  -> Final Absolute Position: X: {final_pos[0]:.4f}, Y: {final_pos[1]:.4f}, Z: {final_pos[2]:.4f}")
                else:
                    print("  -> ERROR: Still no data. base_link might not be the correct root name!")

                # D. HEDEF HATASI (Target Error) & BAŞARIM (Success)
                if exp_name in self.config['scenarios']:
                    if exp_name in self.config['scenarios']:
                        scenario_data = self.config['scenarios'][exp_name]
                        
                        # 1. DYNAMICALLY FIND THE FINAL TARGET ARRAY
                        if exp_name == 'home_to_pose':
                            target_array = scenario_data['target_pose']
                            
                        elif exp_name == 'waypoint':
                            # The final destination is the very last [-1] waypoint in the list
                            target_array = scenario_data['waypoints'][-1]
                            
                        elif exp_name == 'pick_place':
                            # The final destination is the final drop-off point
                            target_array = scenario_data['place_drop']
                            
                        else:
                            print(f"  -> Warning: Unknown scenario '{exp_name}'. Cannot calculate error.")
                            return None
                        
                        # 2. EXTRACT X, Y, Z
                        target_x = target_array[0]
                        target_y = target_array[1]
                        target_z = target_array[2]
                    
                    # Calculate the 3D Euclidean Distance between Actual and Target
                    target_error = math.sqrt((final_pos[0] - target_x)**2 + (final_pos[1] - target_y)**2 + (final_pos[2] - target_z)**2)
                    print(f"  -> Target Error: {target_error:.6f} meters")
                    
                    # Calculate Başarım (Success). Example: True if error is less than 1 cm (0.01 meters)
                    success = target_error < 0.01
                    print(f"  -> Success: {success}")

                    return {
                            'Experiment': exp_name,
                            'Duration (s)': round(duration, 4),
                            'Path Length (m)': round(path_length, 4),
                            'Final X': round(final_pos[0], 4),
                            'Final Y': round(final_pos[1], 4),
                            'Final Z': round(final_pos[2], 4),
                            'Target Error (m)': round(target_error, 6),
                            'Success': success
                }
                else:
                    print(f"  -> Warning: '{exp_name}' not found in YAML. Cannot calculate error.")
                    return None

            conn.close()
            
        except Exception as e:
            print(f"  -> Failed to read database: {e}")

def main(args=None):
    analyzer = BagAnalyzer()
    analyzer.analyze_all_bags()

if __name__ == '__main__':
    main()
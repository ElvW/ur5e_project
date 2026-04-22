import os
import sqlite3
import csv
import math
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from rclpy.time import Time

class BagAnalyzer:
    def __init__(self):
        # Find the bags folder (adjust path if your bags are saved somewhere else)
        self.workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
        self.bag_dir = os.path.join(self.workspace_dir, 'recordings')
        
        if not os.path.exists(self.bag_dir):
            print(f"Error: Could not find bags directory at {self.bag_dir}")
            return
            
        print(f"Analyzer initialized. Looking for databases in: {self.bag_dir}")

    def analyze_all_bags(self):
        """Loops through every .db3 file and extracts the math"""
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
                        self.extract_metrics(db_file)

    def extract_metrics(self, db_path):
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
                                
                                x = pose.transform.translation.x
                                y = pose.transform.translation.y
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

            conn.close()
            
        except Exception as e:
            print(f"  -> Failed to read database: {e}")

def main(args=None):
    analyzer = BagAnalyzer()
    analyzer.analyze_all_bags()

if __name__ == '__main__':
    main()
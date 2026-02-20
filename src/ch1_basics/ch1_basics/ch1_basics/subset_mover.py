import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class SubsetMover(Node):
    def __init__(self):
        super().__init__('subset_mover')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # 1. Define the full list of joint names based on your prefix
        self.joint_names = [
            'east_joint_a1', 'east_joint_a2', 'east_joint_a3', 
            'east_joint_a4', 'east_joint_a5', 'east_joint_a6', 'east_joint_e1',
            'west_joint_a1', 'west_joint_a2', 'west_joint_a3', 
            'west_joint_a4', 'west_joint_a5', 'west_joint_a6', 'west_joint_e1'
        ]
        
        # 2. Initialize all joints to 0.0
        self.current_positions = [0.0] * 14
        self.iteration = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # 3. Only modify the "subset" you care about (e.g., A1 and A3)
        # We leave A2, A4, A5, and A6 at 0.0 as initialized
        self.current_positions[0] = math.sin(self.iteration) * 1.0  # Joint A1
        self.current_positions[2] = math.cos(self.iteration) * 0.5  # Joint A3
        
        msg.position = self.current_positions
        
        self.publisher_.publish(msg)
        self.iteration += 0.05

def main(args=None):
    rclpy.init(args=args)
    node = SubsetMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class ToolAttacher(Node):
    def __init__(self):
        super().__init__('tool_attacher')
        # We publish to the Planning Scene to tell MoveIt about the tool
        self.publisher_ = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        # Check every 1 second if MoveIt is listening
        self.timer = self.create_timer(1.0, self.publish_tool)
        self.get_logger().info('Tool Attacher initialized. Waiting for MoveGroup...')

    def publish_tool(self):
        # 1. Ensure MoveIt is active before sending
        if self.publisher_.get_subscription_count() > 0:
            
            # 2. Define the Tool Geometry
            tool = CollisionObject()
            # This MUST match the last link name in your RCCN URDF (e.g., 'east_link_6')
            tool.header.frame_id = "east_tool0" 
            tool.id = "fabrication_extruder"
            
            shape = SolidPrimitive()
            shape.type = SolidPrimitive.CYLINDER
            shape.dimensions = [0.3, 0.05] # 30cm long, 5cm diameter
            
            pose = Pose()
            pose.position.z = 0.15 # Offset so it protrudes from the flange
            
            tool.primitives.append(shape)
            tool.primitive_poses.append(pose)
            
            # 3. Create the Attachment "Glue"
            attached_tool = AttachedCollisionObject()
            attached_tool.link_name = "east_tool0"
            attached_tool.object = tool
            # 'touch_links' prevents the robot from 'colliding' with its own tool
            attached_tool.touch_links = ["east_link_6", "east_tool0"] 

            # 4. Wrap in the Scene Message
            scene_msg = PlanningScene()
            scene_msg.robot_state.attached_collision_objects.append(attached_tool)
            scene_msg.is_diff = True
            
            self.publisher_.publish(scene_msg)
            self.get_logger().info('✅ Extruder successfully attached to flange. Disabling timer.')
            
            # 5. Stop publishing to prevent UI flickering
            self.timer.cancel()
        else:
            self.get_logger().warn('Waiting for MoveGroup/RViz subscription on /planning_scene...')

def main():
    rclpy.init()
    node = ToolAttacher()
    rclpy.spin(node)
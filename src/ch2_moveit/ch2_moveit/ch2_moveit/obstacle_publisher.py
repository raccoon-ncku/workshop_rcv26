import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')
        # Use a high reliability QoS for scene updates
        self.publisher_ = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        # We use a one-shot timer to give the system time to connect
        self.timer = self.create_timer(1.0, self.publish_obstacle)
        self.get_logger().info('Obstacle Publisher initialized. Waiting to sync...')

    def publish_obstacle(self):
        # 1. Check if anyone is actually listening (Avoids shouting into the void)
        if self.publisher_.get_subscription_count() > 0:
            
            obstacle = CollisionObject()
            obstacle.header.frame_id = "kuka_robot_cell_world"
            obstacle.id = "workshop_table"
            
            shape = SolidPrimitive()
            shape.type = SolidPrimitive.BOX
            shape.dimensions = [1.0, 2.0, 0.1]
            
            pose = Pose()
            pose.position.x = 1.0 
            pose.position.y = 0.0
            pose.position.z = 0.4
            
            obstacle.primitives.append(shape)
            obstacle.primitive_poses.append(pose)
            obstacle.operation = CollisionObject.ADD

            scene_msg = PlanningScene()
            scene_msg.world.collision_objects.append(obstacle)
            scene_msg.is_diff = True 
            
            self.publisher_.publish(scene_msg)
            self.get_logger().info('✅ Table published to Planning Scene. Shutting down timer to prevent flickering.')
            
            # 2. Kill the timer! We only need to do this once.
            self.timer.cancel()
        else:
            self.get_logger().warn('Waiting for MoveGroup/RViz to subscribe to /planning_scene...')

def main():
    rclpy.init()
    node = ObstaclePublisher()
    rclpy.spin(node)
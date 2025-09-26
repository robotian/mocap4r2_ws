import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mocap4r2_msgs.msg import RigidBodies  # Adjust package and type as needed

class MocapPosePublisher(Node):
    def __init__(self):
        super().__init__('rigidbody_pose_publisher')
        # Declare and get namespace parameter
        self.declare_parameter('namespace', '')
        self.ns = self.get_parameter('namespace').get_parameter_value().string_value
        self.sub = self.create_subscription(
            RigidBodies,
            'rigid_bodies',
            # f'{self.ns}/rigid_bodies' if self.ns else '/rigid_bodies',
            self.rigidbodies_callback,
            10
        )
        self.rb_publishers = {}
        self.counter = 0

    def rigidbodies_callback(self, msg):
        if self.counter % 500 == 0:
            self.get_logger().info(f'Received {len(msg.rigidbodies)} rigid bodies')
            self.counter = 0
        self.counter += 1
        
        for rb in msg.rigidbodies:
            # topic_name = f'{self.ns}/rigidbody_{rb.rigid_body_name}/pose' if self.ns else f'/rigidbody_{rb.rigid_body_name}/pose'
            topic_name = f'rigidbody_{rb.rigid_body_name}/pose'
            if topic_name not in self.rb_publishers:
                self.rb_publishers[topic_name] = self.create_publisher(PoseWithCovarianceStamped, topic_name, 10)
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header = msg.header  # Carry over timestamp/frame
            pose_msg.pose.pose.position = rb.pose.position
            pose_msg.pose.pose.orientation = rb.pose.orientation
            pose_msg.pose.covariance = [0.0]*36  # Placeholder covariance
            pose_msg.pose.covariance[0] = 0.0001  # Example covariance values
            pose_msg.pose.covariance[7] = 0.0001
            pose_msg.pose.covariance[14] = 0.0001
            pose_msg.pose.covariance[21] = 0.0003
            pose_msg.pose.covariance[28] = 0.0003
            pose_msg.pose.covariance[35] = 0.0003
            # pose_msg.pose.covariance = [
            #     0.01, 0,    0,    0,       0,       0,      # x
            #     0,    0.01, 0,    0,       0,       0,      # y
            #     0,    0,    0.04, 0,       0,       0,      # z
            #     0,    0,    0,    0.0003,  0,       0,      # roll
            #     0,    0,    0,    0,       0.0003,  0,      # pitch
            #     0,    0,    0,    0,       0,       0.0062  # yaw
            # ]
            
            self.rb_publishers[topic_name].publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MocapPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

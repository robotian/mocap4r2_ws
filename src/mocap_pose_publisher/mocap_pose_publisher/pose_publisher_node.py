import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from mocap4r2_msgs.msg import RigidBodies  # Adjust package and type as needed
import math
import time
from geometry_msgs.msg import Quaternion
import numpy as np
from collections import deque

class MocapPosePublisher(Node):
    def __init__(self):
        super().__init__('rigidbody_pose_publisher')
        # Declare and get namespace parameter
        self.declare_parameter('namespace', '')
        self.ns = self.get_parameter('namespace').get_parameter_value().string_value
        # number of recent twist estimates to average (default 5)
        self.declare_parameter('twist_window', 5)
        self.twist_window = int(self.get_parameter('twist_window').get_parameter_value().integer_value)
        self.sub = self.create_subscription(
            RigidBodies,
            'rigid_bodies',
            # f'{self.ns}/rigid_bodies' if self.ns else '/rigid_bodies',
            self.rigidbodies_callback,
            10
        )
        self.rb_publishers = {}
        self.rb_odom_publishers = {}
        # store previous state for finite-difference velocity estimation
        # map: rigid_body_name -> {'pos': (x,y,z), 'quat': (x,y,z,w), 'time': float}
        self._prev_state = {}
        # per-rigidbody buffers for recent twist estimates
        # name -> {'lin_buf': deque, 'ang_buf': deque}
        self._twist_buffers = {}
        self.counter = 0

    def _quat_to_np(self, q: Quaternion):
        return np.array([q.x, q.y, q.z, q.w], dtype=float)

    def _quat_inverse(self, q):
        # q assumed [x,y,z,w]
        return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)

    def _quat_mul(self, a, b):
        # Hamilton product (x,y,z,w)
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        x = aw*bx + ax*bw + ay*bz - az*by
        y = aw*by - ax*bz + ay*bw + az*bx
        z = aw*bz + ax*by - ay*bx + az*bw
        w = aw*bw - ax*bx - ay*by - az*bz
        return np.array([x, y, z, w], dtype=float)

    def _quat_to_angular_vel(self, q_prev, q_cur, dt):
        # q_prev, q_cur are arrays [x,y,z,w]. Compute angular velocity vector (rad/s)
        if dt <= 0:
            return np.zeros(3)
        dq = self._quat_mul(q_cur, self._quat_inverse(q_prev))
        # Ensure scalar part in [-1,1]
        w = np.clip(dq[3], -1.0, 1.0)
        angle = 2.0 * math.acos(w)
        s = math.sqrt(max(0.0, 1.0 - w*w))
        if s < 1e-8:
            # angle ~ 0, direction arbitrary
            return np.zeros(3)
        axis = dq[0:3] / s
        return axis * (angle / dt)

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
            odom_topic = f'rigidbody_{rb.rigid_body_name}/odom'
            if odom_topic not in self.rb_odom_publishers:
                self.rb_odom_publishers[odom_topic] = self.create_publisher(Odometry, odom_topic, 10)
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
            # Create and publish Odometry message derived from the pose
            odom_msg = Odometry()
            odom_msg.header = msg.header
            # Use rigid body name as child_frame_id
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose.position = rb.pose.position
            odom_msg.pose.pose.orientation = rb.pose.orientation
            # Copy covariance from PoseWithCovarianceStamped into Odometry.pose.covariance
            odom_msg.pose.covariance = pose_msg.pose.covariance
            # Estimate twist via finite-differences using previous pose
            now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            name = rb.rigid_body_name
            lin_v = np.zeros(3)
            ang_v = np.zeros(3)
            if name in self._prev_state:
                prev = self._prev_state[name]
                dt = now - prev['time'] if ('time' in prev and prev['time'] is not None) else 0.0
                if dt > 1e-9:
                    cur_pos = np.array([rb.pose.position.x, rb.pose.position.y, rb.pose.position.z], dtype=float)
                    prev_pos = np.array(prev['pos'], dtype=float)
                    lin_v = (cur_pos - prev_pos) / dt
                    q_prev = prev['quat']
                    q_cur = self._quat_to_np(rb.pose.orientation)
                    ang_v = self._quat_to_angular_vel(q_prev, q_cur, dt)
            # store current state
            self._prev_state[name] = {
                'pos': (rb.pose.position.x, rb.pose.position.y, rb.pose.position.z),
                'quat': self._quat_to_np(rb.pose.orientation),
                'time': now
            }

            # push latest estimate into per-rigidbody buffers and compute simple mean over last N samples
            if name not in self._twist_buffers:
                self._twist_buffers[name] = {'lin_buf': deque(maxlen=self.twist_window), 'ang_buf': deque(maxlen=self.twist_window)}
            buf = self._twist_buffers[name]
            buf['lin_buf'].append(np.array(lin_v, dtype=float))
            buf['ang_buf'].append(np.array(ang_v, dtype=float))
            # compute mean across available samples
            if len(buf['lin_buf']) > 0:
                filt_lin = np.mean(np.stack(buf['lin_buf'], axis=0), axis=0)
            else:
                filt_lin = np.zeros(3)
            if len(buf['ang_buf']) > 0:
                filt_ang = np.mean(np.stack(buf['ang_buf'], axis=0), axis=0)
            else:
                filt_ang = np.zeros(3)

            odom_msg.twist.twist.linear.x = float(filt_lin[0])
            odom_msg.twist.twist.linear.y = float(filt_lin[1])
            odom_msg.twist.twist.linear.z = float(filt_lin[2])
            odom_msg.twist.twist.angular.x = float(filt_ang[0])
            odom_msg.twist.twist.angular.y = float(filt_ang[1])
            odom_msg.twist.twist.angular.z = float(filt_ang[2])
            # Set a placeholder covariance for twist (unknown)
            odom_msg.twist.covariance = [0.0]*36
            self.rb_odom_publishers[odom_topic].publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MocapPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

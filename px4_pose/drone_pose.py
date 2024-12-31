import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from px4_msgs.msg import VehicleOdometry
import tf2_ros
import time

class DronePose(Node):
    def __init__(self):
        super().__init__('drone_pose')
        # Parameters
        self.base_frame = self.declare_parameter('base_frame', '/map').value
        self.child_frame = self.declare_parameter('child_frame', '/base_link').value
        self.pub_pose = self.declare_parameter('pub.pose', True).value
        self.pub_vel = self.declare_parameter('pub.vel', True).value
        self.pub_path = self.declare_parameter('pub.path', True).value


        qos = rclpy.qos.QoSProfile(depth=5, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.vehicle_odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.position_cb,
            qos)
        self.vehicle_path_pub = self.create_publisher(Path, '/vehicle_path', 10)
        self.vehicle_pose_pub = self.create_publisher(PoseStamped, '/vehicle_pose', 10)
        self.vehicle_vel_pub = self.create_publisher(Marker, '/vehicle_velocity', 10)
        self.path_msg = Path()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info('Init PX4 position.')

    def position_cb(self, odom_msg):
        start_time = time.time()
        pose_msg = PoseStamped()
        arrow_velocity_msg = Marker()
        current_time = self.get_clock().now()

        if self.pub_pose:
            pose_msg.header.frame_id = self.base_frame
            pose_msg.header.stamp = current_time.to_msg()
            pose_msg.pose.orientation.w = float(odom_msg.q[0])
            pose_msg.pose.orientation.x = float(odom_msg.q[1])
            pose_msg.pose.orientation.y = -float(odom_msg.q[2])
            pose_msg.pose.orientation.z = -float(odom_msg.q[3])
            pose_msg.pose.position.x = float(odom_msg.position[0])
            pose_msg.pose.position.y = -float(odom_msg.position[1])
            pose_msg.pose.position.z = -float(odom_msg.position[2])
            self.vehicle_pose_pub.publish(pose_msg)

        if self.pub_vel:
            arrow_velocity_msg.action = 0
            arrow_velocity_msg.header.frame_id = self.base_frame
            arrow_velocity_msg.header.stamp = current_time.to_msg()
            arrow_velocity_msg.ns = 'arrow'
            arrow_velocity_msg.type = 0
            arrow_velocity_msg.scale.x = 0.1
            arrow_velocity_msg.scale.y = 0.2
            arrow_velocity_msg.scale.z = 0.1
            arrow_velocity_msg.color.r = 0.5
            arrow_velocity_msg.color.g = 0.5
            arrow_velocity_msg.color.b = 0.0
            arrow_velocity_msg.color.a = 1.0
            dt = 0.2
            
            tail = Point()
            head = Point()
            tail.x = float(odom_msg.position[0])
            tail.y = -float(odom_msg.position[1])
            tail.z = -float(odom_msg.position[2])
            head.x = float(odom_msg.position[0]) + dt * float(odom_msg.velocity[0])
            head.y = -float(odom_msg.position[1]) + dt * -float(odom_msg.velocity[1])
            head.z = -float(odom_msg.position[2]) + dt * -float(odom_msg.velocity[2])
            arrow_velocity_msg.points = [tail, head]
            self.vehicle_vel_pub.publish(arrow_velocity_msg)

        if self.pub_path:
            self.path_msg.header = pose_msg.header
            self.path_msg.poses.append(pose_msg)
            self.vehicle_path_pub.publish(self.path_msg)

        tf = TransformStamped()
        tf.header.stamp = current_time.to_msg()
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.child_frame
        tf.transform.translation.x = float(odom_msg.position[0])
        tf.transform.translation.y = -float(odom_msg.position[1])
        tf.transform.translation.z = -float(odom_msg.position[2])
        tf.transform.rotation.x = float(odom_msg.q[1])
        tf.transform.rotation.y = -float(odom_msg.q[2])
        tf.transform.rotation.z = -float(odom_msg.q[3])
        tf.transform.rotation.w = float(odom_msg.q[0])
        self.tf_broadcaster.sendTransform(tf)

        end_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = DronePose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

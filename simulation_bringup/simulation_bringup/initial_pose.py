#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(Odometry, '/init_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_initial_pose)
        self.declare_parameter('frame_id', 'lidar')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.num_published = 0

    def publish_initial_pose(self):
        initial_pose = Odometry()
        initial_pose.header = Header()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = self.get_parameter('frame_id').value
        # initial_pose.pose.pose.position.x = 5.806057414
        # initial_pose.pose.pose.position.y = 5.52551664
        # initial_pose.pose.pose.position.z = -1.326369883
        # initial_pose.pose.pose.orientation.x = 0.0002744296454
        # initial_pose.pose.pose.orientation.y = 0.002159387464
        # initial_pose.pose.pose.orientation.z = 0.4439909921
        # initial_pose.pose.pose.orientation.w = 0.8960286048
        
        pose = Pose()
        # pose.position.x = 5.806057414
        # pose.position.y = 5.52551664
        # pose.position.z = 1.326369883
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        # pose.orientation.x = 0.0002744296454
        # pose.orientation.y = 0.002159387464
        # pose.orientation.z = 0.4439909921 #- 0.7071068
        # pose.orientation.w = 0.8960286048 #+ 0.7071068

        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        # pose.orientation.z = -0.7071068
        # pose.orientation.w = 0.7071068

        # transform_stamped = self.tf_buffer.lookup_transform('map', initial_pose.header.frame_id, initial_pose.header.stamp)
        # pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform_stamped)

        initial_pose.pose.pose = pose

        self.publisher.publish(initial_pose)
        self.num_published += 1

        if self.num_published > 5:

            self.destroy_node()
        # self.get_logger().warn('Published initial pose on /initial_pose')

def main(args=None):
    rclpy.init(args=args)
    initial_pose_publisher = InitialPosePublisher()
    rclpy.spin(initial_pose_publisher)
    initial_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

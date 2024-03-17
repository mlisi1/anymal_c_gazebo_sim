#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Vector3Stamped


class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('TrajectoryStreamer')
        self.declare_parameter('odom_topic', '/odom_rf2o')
        self.declare_parameter('path_topic', '/path_rf2o')
        self.declare_parameter('vectorStamped', False)

        if self.get_parameter('vectorStamped').value:
            self.subscription = self.create_subscription(
            Vector3Stamped,
            self.get_parameter('odom_topic').value,
            self.odometry_callback,
            10)
        else:
            self.subscription = self.create_subscription(
                Odometry,
                self.get_parameter('odom_topic').value,
                self.odometry_callback,
                10)
        self.path_publisher = self.create_publisher(Path, self.get_parameter('path_topic').value, 100)
        self.path = Path()
        self.declare_parameter('frame', 'path')
        self.declare_parameter('synchronize', True)
        self.declare_parameter('filepath', '')
        # self.filepath = '/home/elechim/test_msg.txt'

    def write_path_to_file(self):
        if not self.get_parameter('filepath').value == '':
            with open(self.get_parameter('filepath').value, 'w') as file:
                # Write header information if available
                if self.path.header:
                    file.write(f"Header:\n{self.path.header}\n\n")

                # Write each pose in the path
                for pose_stamped in self.path.poses:
                    file.write(f"Pose:\n{pose_stamped}\n\n")
    

    def odometry_callback(self, msg):
        # Extract pose information from the Odometry message
        now = self.get_clock().now().to_msg()
        pose_stamped = PoseStamped()

        if self.get_parameter('vectorStamped').value:

            pose_stamped.pose.position.x = msg.vector.x
            pose_stamped.pose.position.y = msg.vector.y
            pose_stamped.pose.position.z = msg.vector.z

        else:
            pose_stamped.pose = msg.pose.pose
            pose_stamped.header = msg.header
            pose_stamped.header.stamp = now




        # Accumulate the pose information
        if len(self.path.poses) == 0:
            self.path.poses = [pose_stamped]
        else:
            self.path.poses.append(pose_stamped)

        # Optionally, you can perform further processing or visualization here


        # Publish the updated path
        self.path.header.frame_id = self.get_parameter('frame').value
        if self.get_parameter('synchronize').value:
            self.path.header.stamp = msg.header.stamp
        else:
            self.path.header.stamp = now
        if not self.path.header.frame_id == '':
            self.path_publisher.publish(self.path)
            self.write_path_to_file()

def main(args=None):
    rclpy.init(args=args)

    odometry_subscriber = OdometrySubscriber()

    rclpy.spin(odometry_subscriber)

    odometry_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

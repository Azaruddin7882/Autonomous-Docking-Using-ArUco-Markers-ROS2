import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

class ArUcoMarkerDetection(Node):
    def __init__(self):
        super().__init__('aruco_marker_detection')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(PoseStamped, 'aruco_marker_pose', 10)
        self.bridge = CvBridge()
        self.get_logger().info('ArUco Marker Detection Node has been started.')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        dictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)

        if ids is not None:
            self.get_logger().info(f'Detected markers with IDs: {ids.flatten()}')
            for i in range(len(ids)):
                aruco.drawDetectedMarkers(cv_image, corners, ids)

                marker_pose = PoseStamped()
                marker_pose.header.frame_id = 'camera_link'
                marker_pose.header.stamp = self.get_clock().now().to_msg()
                marker_pose.pose.position.x = corners[i][0][0][0]
                marker_pose.pose.position.y = corners[i][0][0][1]
                marker_pose.pose.position.z = 0.0

                self.publisher.publish(marker_pose)
                self.get_logger().info(f'Marker {ids[i][0]} pose: x={marker_pose.pose.position.x}, y={marker_pose.pose.position.y}, z={marker_pose.pose.position.z}')
        else:
            self.get_logger().info('No markers detected.')

        cv2.imshow('Detected Markers', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoMarkerDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

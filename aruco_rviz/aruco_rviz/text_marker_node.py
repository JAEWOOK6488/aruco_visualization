import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point

class TextMarkerTFListener(Node):
    def __init__(self):
        super().__init__('text_marker_tf_listener')

        self.publisher_ = self.create_publisher(Marker, '/visualization_marker', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.publish_tf_as_text)

    def publish_tf_as_text(self):
        try:
            transform = self.tf_buffer.lookup_transform('camera_color_optical_frame', 'aruco_marker', rclpy.time.Time())
            
            position = transform.transform.translation
            orientation = transform.transform.rotation

            text_data = f"Position: ({position.x}, {position.y}, {position.z})\n"
            text_data += f"Orientation: ({orientation.x}, {orientation.y}, {orientation.z}, {orientation.w})"
            
            marker = Marker()

            marker.header.frame_id = "camera_color_optical_frame"
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = "text"
            marker.id = 0

            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD

            marker.pose.position.x = position.x
            marker.pose.position.y = position.y
            marker.pose.position.z = position.z
            marker.pose.orientation.w = orientation.w 

            marker.text = text_data

            marker.scale.z = 0.1  # Text scale

            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            self.publisher_.publish(marker)

        except Exception as e:
            self.get_logger().warn(f"Could not get TF transform. Reason: {e}")

def main(args=None):
    rclpy.init(args=args)

    node = TextMarkerTFListener()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

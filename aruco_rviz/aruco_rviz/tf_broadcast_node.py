import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import tf2_ros

class ArucoTFBroadcaster(Node):

    def __init__(self):
        super().__init__('aruco_tf_broadcaster')
        
        # Subscribe to the Aruco marker's position
        self.subscription = self.create_subscription(
            PoseStamped,
            '/marker_pose',
            self.aruco_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.br = tf2_ros.TransformBroadcaster(self)
        
    def aruco_callback(self, data):
        print(data.pose.position)
        t = TransformStamped()

        # Set the timestamp
        t.header.stamp = data.header.stamp

        # Set the parent and child frame IDs
        t.header.frame_id = "camera_color_optical_frame"
        t.child_frame_id = data.header.frame_id

        # For this example, we'll just set a fixed translation and rotation
        t.transform.translation.x = data.pose.position.x*0.01
        t.transform.translation.y = data.pose.position.y*0.01
        t.transform.translation.z = data.pose.position.z*0.01
        t.transform.rotation.x = data.pose.orientation.x
        t.transform.rotation.y = data.pose.orientation.y
        t.transform.rotation.z = data.pose.orientation.z
        t.transform.rotation.w = data.pose.orientation.w

        # Broadcast the transform using sendTransform
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    node = ArucoTFBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

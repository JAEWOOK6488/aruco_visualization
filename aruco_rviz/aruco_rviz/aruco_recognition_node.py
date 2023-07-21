import cv_bridge
import cv2
from cv2 import aruco
import rclpy as rp
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

from ament_index_python.packages import get_package_share_directory

class MarkerDetector(Node):
    def __init__(self):
        super().__init__('marker_detector')
        self.distance = 0.0
        self.x = 0.0
        self.y = 0.0
        self.calibration_path = get_package_share_directory('aruco_rviz') + '/config/MultiMatrix.npz'
        self.calib_data = np.load(self.calibration_path)
        self.cam_mat = self.calib_data["camMatrix"]
        self.dist_coef = self.calib_data["distCoef"]
        self.r_vectors = self.calib_data["rVector"]
        self.t_vectors = self.calib_data["tVector"]

        self.MARKER_SIZE = 5  # centimeters
        self.marker_dict = aruco.Dictionary_get(aruco.DICT_6X6_100)
        self.param_markers = aruco.DetectorParameters_create()

        self.cv_bridge_color = cv_bridge.CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.callback_marker,
            1)
        # self.publisher = self.create_publisher(MarkerDetection, '/marker_det_result', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/marker_pose', 10)

    def rotation_matrix_to_quaternion(self, R):
        """ 회전 행렬에서 쿼터니언으로 변환 """
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (R[2, 1] - R[1, 2]) * s
            qy = (R[0, 2] - R[2, 0]) * s
            qz = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

        return np.array([qw, qx, qy, qz])

    def callback_marker(self, msg):
        cv_image = self.cv_bridge_color.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        gray_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, self.marker_dict, parameters=self.param_markers
        )
        if marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef
            )
            # 마커에 대한 rVec을 회전 행렬로 변환
            R_mtx, _ = cv2.Rodrigues(rVec[0])

            # 회전 행렬에서 쿼터니언으로 변환
            quaternion = self.rotation_matrix_to_quaternion(R_mtx)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'aruco_marker'
            pose_msg.pose.position.x = tVec[0][0][0]
            pose_msg.pose.position.y = tVec[0][0][1]
            pose_msg.pose.position.z = tVec[0][0][2]
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]
            self.pose_publisher.publish(pose_msg)

            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv2.polylines(
                    cv_image, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

                self.distance = np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )
                # Draw the pose of the marker
                point = cv2.drawFrameAxes(cv_image, self.cam_mat, self.dist_coef, rVec[i], tVec[i], 4, 4)
                cv2.putText(
                    cv_image,
                    f"id: {ids[0]} Dist: {round(self.distance, 2)}",
                    top_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    cv_image,
                    f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                    bottom_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                self.x, self.y = round(tVec[i][0][0],1), round(tVec[i][0][1],1)
                # print(ids, "  ", corners)
        rgb_cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        cv2.imshow("frame", rgb_cv_image)
        cv2.waitKey(50)

def main(args=None):
    rp.init(args=args)
    node = MarkerDetector()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
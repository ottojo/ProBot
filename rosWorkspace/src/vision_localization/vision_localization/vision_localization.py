import math

import cv2 as cv
import geometry_msgs.msg
import image_geometry
import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformBroadcaster

# import tf_transformations

arucoParams = cv.aruco.DetectorParameters_create()
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)


def inverseRTVec(rvec, tvec):
    """
    Takes a transformation x' = rotate(x) + t specified by rvec and tvec
    :param rvec: Rotation in vector form (use Rodrigues to convert between/to rotation matrix)
    :param tvec: Translation t
    :return: Rotation rotate2 and transformation t2 for transformation x = invrotate(x' - t) = rotate2(x') + t2
    """
    assert (tvec.shape == (3,))
    R, _ = cv.Rodrigues(rvec)
    R = cv.transpose(R)  # == invert
    invTvec = np.dot(R, -tvec)
    invRvec, _ = cv.Rodrigues(R)
    return invRvec, invTvec


def transformBetweenMarkers(baseRvec, baseTvec, markerRvec, markerTvec, v):
    """
    :param baseRvec: Rotation for base marker coordinate system
    :param baseTvec: Translation for base marker coordinate system
    :param markerRvec: Rotation for second marker coordinate system
    :param markerTvec: Translation for second marker coordinate system
    :param v: Vector to be transformed
    """
    # print(f"transforming {v} from marker system at {markerTvec} to base system at {baseTvec}")
    # Transform v into camera coordinates
    # print(f"inverse translation for marker system is {invT}")
    r1, _ = cv.Rodrigues(markerRvec)
    # print(f"inverse rotation matrix for marker system is {r1}")
    vInCamera = np.dot(r1, v) + markerTvec
    # print(f"v in camera coordinates is {vInCamera}")

    # Transform into base coordinate system
    invBaseR, invBaseT = inverseRTVec(baseRvec, baseTvec)
    r2, _ = cv.Rodrigues(invBaseR)
    vInBase = np.dot(r2, vInCamera) + invBaseT
    return vInBase


def rot_vec_to_quaternion(rotvec):
    angle = cv.norm(rotvec)
    axis = rotvec.squeeze() / angle
    angle_2 = angle / 2
    q = geometry_msgs.msg.Quaternion()
    q.x = axis[0] * math.sin(angle_2)
    q.y = axis[1] * math.sin(angle_2)
    q.z = axis[2] * math.sin(angle_2)
    q.w = math.cos(angle_2)
    return q


def transform_from_rvec_tvec(rvec, tvec, parent_id, child_id, stamp):
    t = TransformStamped()
    # Read message content and assign it to
    # corresponding tf variables
    t.header.stamp = stamp
    t.header.frame_id = parent_id
    t.child_frame_id = child_id

    # Turtle only exists in 2D, thus we get x and y translation
    # coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = tvec[0]
    t.transform.translation.y = tvec[1]
    t.transform.translation.z = tvec[2]

    t.transform.rotation = rot_vec_to_quaternion(rvec)
    return t


class LocalizationNode(Node):
    def on_image(self, image_message):
        frame = self.br.imgmsg_to_cv2(image_message, "bgr8")
        corners, ids, _ = cv.aruco.detectMarkers(frame, dictionary, parameters=arucoParams)
        if ids is not None:
            cv.aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.03348, self.camera_model.intrinsicMatrix(),
                                                                 self.camera_model.distortionCoeffs())
            for r, t in zip(rvecs, tvecs):
                cv.drawFrameAxes(frame, self.camera_model.intrinsicMatrix(), self.camera_model.distortionCoeffs(), r, t,
                                 0.1)

            ids: np.ndarray = ids.squeeze()
            first_index = np.where(ids == 11)
            first_index = None if first_index[0].size == 0 else first_index[0][0]
            second_index = np.where(ids == 12)
            second_index = None if second_index[0].size == 0 else second_index[0][0]

            if first_index is not None and second_index is not None:
                first_tvec = tvecs[first_index].reshape((3,))
                first_rvec = rvecs[first_index]
                second_tvec = tvecs[second_index].reshape((3,))
                second_rvec = rvecs[second_index]
                x_in_first_marker = transformBetweenMarkers(first_rvec, first_tvec, second_rvec, second_tvec,
                                                            np.array([0, 0, 0]))
                print(f"x_in_first_marker={x_in_first_marker}")

                stamp = self.get_clock().now().to_msg()

                camr, camt = inverseRTVec(first_rvec, first_tvec)

                # Send the transformation
                self.tf_broadcaster.sendTransform(
                    [transform_from_rvec_tvec(second_rvec, second_tvec, "tracking_camera", "target", stamp),
                     transform_from_rvec_tvec(first_rvec, first_tvec, "tracking_camera", "map", stamp)])

        cv.imshow("frame", frame)
        if cv.waitKey(20) & 0xFF == ord('q'):
            exit(0)

    def on_camera_info(self, camerainfo_message):
        self.get_logger().info("New CameraInfo")
        self.camera_model.fromCameraInfo(camerainfo_message)

    def __init__(self):
        super(LocalizationNode, self).__init__("vision_localization")
        self.cameraSubscription = self.create_subscription(Image, "tracking_camera/image_raw", self.on_image, 10)
        self.cameraInfoSubscription = self.create_subscription(CameraInfo, "tracking_camera/camera_info",
                                                               self.on_camera_info, 10)
        self.br = CvBridge()
        self.camera_model = image_geometry.PinholeCameraModel()
        self.tf_broadcaster = TransformBroadcaster(self)


def main(args=None):
    rclpy.init(args=args)
    localization_node = LocalizationNode()
    rclpy.spin(localization_node)
    localization_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

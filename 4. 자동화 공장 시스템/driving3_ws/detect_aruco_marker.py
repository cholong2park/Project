import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# 카메라 매트릭스와 왜곡 계수 설정
mtx = np.array([[743.23694266, 0., 607.42392992],
                [0., 746.17628819, 328.91387719],
                [0., 0., 1.]])
dist = np.array([[0.03555538, -0.05886207, -0.00330347, -0.00567832, -0.14910992]])

# ArUco 딕셔너리 및 파라미터 설정
desired_aruco_dictionary = "DICT_5X5_100"
ARUCO_DICT = {
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100
}
this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])
this_aruco_parameters = cv2.aruco.DetectorParameters()

# 마커 크기 (실제 크기, 단위: 미터)
marker_size = 0.105  # 10.5cm

# 기준 마커 ID 정의
REFERENCE_MARKER_ID = 23
TARGET_MARKER_ID = 21


class ArucoMarkerPublisher(Node):
    def __init__(self):
        super().__init__('aruco_marker_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'aruco_marker_pose', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.cap = cv2.VideoCapture('/dev/video4')  # 기본 웹캠 장치 사용
        if not self.cap.isOpened():
            self.get_logger().error("웹캠을 열 수 없습니다.")
            exit()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("프레임을 가져올 수 없습니다.")
            return

        # ArUco 마커 탐지
        detector = cv2.aruco.ArucoDetector(this_aruco_dictionary, this_aruco_parameters)
        corners, ids, rejected = detector.detectMarkers(frame)

        if ids is not None:
            ids = ids.flatten()
            rvecs = []
            tvecs = []

            # 각 마커에 대해 자세한 정보 계산
            for i, marker_corners in enumerate(corners):
                retval, rvec, tvec = cv2.solvePnP(
                    objectPoints=self.create_aruco_object_points(marker_size),
                    imagePoints=marker_corners,
                    cameraMatrix=mtx,
                    distCoeffs=dist,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )
                if retval:
                    rvecs.append(rvec)
                    tvecs.append(tvec)

            # 기준 마커의 rvec과 tvec 찾기
            reference_rvec = None
            reference_tvec = None
            for i, marker_id in enumerate(ids):
                if marker_id == REFERENCE_MARKER_ID:
                    reference_rvec = rvecs[i]
                    reference_tvec = tvecs[i]
                    break

            if reference_rvec is None or reference_tvec is None:
                self.get_logger().info(f"Reference marker ID {REFERENCE_MARKER_ID} not found.")
                return

            # 기준 마커의 회전 행렬 계산
            R_ref, _ = cv2.Rodrigues(reference_rvec)
            T_ref = reference_tvec.reshape(3, 1)

            for i, marker_id in enumerate(ids):
                if marker_id == REFERENCE_MARKER_ID:
                    # 기준 마커는 카메라 좌표계 축만 그림
                    cv2.drawFrameAxes(frame, mtx, dist, reference_rvec, reference_tvec, 0.1)
                    continue

                # 현재 마커의 rvec과 tvec
                rvec = rvecs[i]
                tvec = tvecs[i]

                # 현재 마커의 회전 행렬 계산
                R_marker, _ = cv2.Rodrigues(rvec)
                T_marker = tvec.reshape(3, 1)

                # 상대 좌표 계산
                R_relative = np.dot(R_ref.T, R_marker)
                T_relative = np.dot(R_ref.T, T_marker - T_ref)

                # 상대 rvec 계산
                rvec_relative, _ = cv2.Rodrigues(R_relative)

                # 출력 (상대 좌표)
                print(f"Marker ID: {marker_id}")
                print(f"Relative Position (tvec): {T_relative.ravel()}")
                print(f"Relative Rotation (rvec): {rvec_relative.ravel()}")

                # 카메라 좌표계 기준 축 그리기
                cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, 0.1)
                if marker_id == TARGET_MARKER_ID:
                    rvecT = rvec_relative.ravel()
                    tvecT = T_relative.ravel()
                    
                    msg = Float32MultiArray()
                    msg.data = [tvecT[0], tvecT[1], tvecT[2], rvecT[0], rvecT[1], rvecT[2]]
                    
                    self.publisher_.publish(msg)
            # 탐지된 마커 표시
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        cv2.imshow('ArUco Marker Detector', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()

    def create_aruco_object_points(self, marker_size):
        """Create 3D object points for an ArUco marker."""
        half_size = marker_size / 2.0
        return np.array([
            [-half_size, half_size, 0.0],
            [half_size, half_size, 0.0],
            [half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0]
        ], dtype=np.float32)


def main(args=None):
    rclpy.init(args=args)
    aruco_marker_publisher = ArucoMarkerPublisher()
    rclpy.spin(aruco_marker_publisher)
    aruco_marker_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

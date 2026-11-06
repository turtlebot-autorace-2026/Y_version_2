import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import pickle
import os

from autorace.camera.device import open_capture

class UndistortedVideoPublisher(Node):
    def __init__(self):
        super().__init__('undistorted_video_publisher')

        # Publisher
        self.pub = self.create_publisher(CompressedImage, 'camera', 10)
        self.raw_pub = self.create_publisher(CompressedImage, 'camera_raw', 10)
        self.wide_raw_pub = self.create_publisher(CompressedImage, 'camera_raw_16_9', 10)

        # Load camera calibration
        # copied from turtlebot-autorace-2026/camera_ros (cf15991); calibration now in share/autorace/config
        from ament_index_python.packages import get_package_share_directory
        calib_file = os.path.join(get_package_share_directory('autorace'), 'config', 'camera_calibration.pkl')
        if not os.path.exists(calib_file):
            self.get_logger().error(f"Calibration file not found: {calib_file}")
            return

        with open(calib_file, 'rb') as f:
            calib_data = pickle.load(f)
        self.camera_matrix = calib_data['camera_matrix']
        self.dist_coeffs = calib_data['dist_coeffs']

        # Open camera: a stable /dev/v4l name (config/camera_params.yaml), not a bare index that can
        # swap with the front camera after a reboot / re-plug
        device = str(self.declare_parameter('device', '0').value)
        self.cap, opened = open_capture(device, cv2)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open lane camera %s." % opened)
            return
        self.get_logger().info('lane camera: %s' % opened)

        # Capture one native 16:9 stream and derive the 4:3 view from it.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Prepare undistortion map
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab a frame for calibration mapping.")
            return
        h, w = frame.shape[:2]
        if (w, h) != (1280, 720):
            self.cap.release()
            raise RuntimeError(
                f'Camera returned {w}x{h}; native 1280x720 capture is required.'
            )

        # alpha=0 -> black area removal
        self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 0, (w, h)
        )

        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix, self.dist_coeffs, None, self.new_camera_matrix, (w, h), cv2.CV_16SC2
        )

        # Timer ~30 FPS
        self.timer = self.create_timer(0.03, self.publish_frame)
        self.get_logger().info(
            'Publishing 1280x720 original frames on camera_raw_16_9, '
            '640x480 center-cropped frames on camera_raw, '
            'and undistorted frames on camera.'
        )

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame.")
            return

        stamp = self.get_clock().now().to_msg()

        # Keep the native 16:9 frame intact; center-crop a separate 4:3 view.
        self.publish_compressed(self.wide_raw_pub, frame, stamp)
        raw_4_3 = cv2.resize(frame[:, 160:1120], (640, 480), interpolation=cv2.INTER_AREA)
        self.publish_compressed(self.raw_pub, raw_4_3, stamp)

        # Undistort
        undistorted = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)
        # Crop to ROI to remove black borders
        x, y, w_crop, h_crop = self.roi
        undistorted = undistorted[y:y+h_crop, x:x+w_crop]
        undistorted = cv2.resize(undistorted, (640,480))
        
        '''
        # Show undistorted video
        cv2.imshow('Undistorted Video', undistorted)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return
        '''
        
        self.publish_compressed(self.pub, undistorted, stamp)

    def publish_compressed(self, publisher, frame, stamp):
        """Publish a JPEG frame with the timestamp shared by all three views."""
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            self.get_logger().warn(f'Failed to encode frame for {publisher.topic_name}.')
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.format = 'jpeg'
        msg.data = buffer.tobytes()
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UndistortedVideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


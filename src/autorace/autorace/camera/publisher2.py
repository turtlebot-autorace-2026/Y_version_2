import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

from autorace.camera.device import open_capture


class NormalVideoPublisher(Node):
    def __init__(self):
        super().__init__('normal_video_publisher')
        self.pub = self.create_publisher(CompressedImage, 'camera2', 10)

        # front camera (signs, traffic light, barrier): a stable /dev/v4l name from
        # config/camera_params.yaml, not the bare index 2 that can swap with the lane camera
        device = str(self.declare_parameter('device', '2').value)
        self.cap, opened = open_capture(device, cv2)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera2 %s." % opened)
            return
        self.get_logger().info('front camera (camera2): %s' % opened)

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab a frame for calibration mapping.")
            return

        # Timer ~30 FPS
        self.timer = self.create_timer(0.03, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame.")
            return
        frame = cv2.resize(frame, (640,480))

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            self.get_logger().warn("Failed to encode frame.")
            return
        msg.data = np.array(buffer).tobytes()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NormalVideoPublisher()
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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw_real_d435i', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(8)  # カメラデバイス番号
        self.br = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().info("カメラを開けませんでした")
            exit()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("フレームを読み込めませんでした")
            return

        msg = self.br.cv2_to_imgmsg(frame, 'bgr8')

        # 現在時刻をROS 2のClockから取得してタイムスタンプを設定
        msg.header.stamp = self.get_clock().now().to_msg()  # 現在時刻を設定

        msg.header.frame_id = "camera_frame"  # フレームIDを設定
        self.publisher_.publish(msg)

    def destroy(self):
        self.cap.release()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

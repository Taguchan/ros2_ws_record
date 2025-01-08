import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from message_filters import Subscriber, TimeSynchronizer

class SyncNode(Node):
    def __init__(self):
        super().__init__('sync_node')
        
        # 各トピックのサブスクライバーを作成
        self.image_sub_1 = Subscriber(self, Image, '/camera/image_raw')
        self.image_sub_2 = Subscriber(self, Image, '/camera/image_raw_real_d435i')
        self.image_sub_3 = Subscriber(self, Image, '/camera/image_raw_real_d515')
        self.emotion_result_sub = Subscriber(self, String, '/emotion_result')
        self.emotion_image_sub = Subscriber(self, Image, '/emotion_image')

        # 時間同期を使って複数のトピックを同期
        self.sync = TimeSynchronizer(
            [self.image_sub_1, self.image_sub_2, self.image_sub_3, self.emotion_result_sub, self.emotion_image_sub],
            10  # バッファサイズ
        )
        self.sync.registerCallback(self.sync_callback)

        # 新しいトピックにパブリッシュ
        self.publisher = self.create_publisher(String, '/synchronized_data', 10)

        # CvBridgeの初期化
        self.bridge = CvBridge()

    def sync_callback(self, image_msg_1, image_msg_2, image_msg_3, emotion_result_msg, emotion_image_msg):
        # 各メッセージを適切に処理し、1つのメッセージにまとめる
        synchronized_data = f"Emotion: {emotion_result_msg.data}"

        # 画像データの処理
        # 画像をリストや辞書にまとめてもよい
        images = {
            'camera_1': self.bridge.imgmsg_to_cv2(image_msg_1, 'bgr8'),
            'camera_2': self.bridge.imgmsg_to_cv2(image_msg_2, 'bgr8'),
            'camera_3': self.bridge.imgmsg_to_cv2(image_msg_3, 'bgr8'),
            'emotion_image': self.bridge.imgmsg_to_cv2(emotion_image_msg, 'bgr8')
        }

        # 画像の処理（例えば、表示や保存など）
        # ここでは画像を処理せず、例としてログに出力
        self.get_logger().info(synchronized_data)

        # 新しいトピックに同期データをパブリッシュ
        self.publisher.publish(String(data=synchronized_data))

def main(args=None):
    rclpy.init(args=args)
    sync_node = SyncNode()
    rclpy.spin(sync_node)
    sync_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

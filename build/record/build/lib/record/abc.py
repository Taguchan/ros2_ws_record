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
        self.emotion_image_sub = Subscriber(self, Image, '/emotion_image')

        # 時間同期を使って複数の画像トピックを同期
        self.sync = TimeSynchronizer(
            [self.image_sub_1, self.image_sub_2, self.image_sub_3, self.emotion_image_sub],
            10  # バッファサイズ
        )
        self.sync.registerCallback(self.sync_callback)

        # 結果の`String`メッセージ用のサブスクライバーを作成
        self.emotion_result_sub = self.create_subscription(String, '/emotion_result', self.emotion_result_callback, 10)

        # 新しいトピックにパブリッシュ
        self.publisher = self.create_publisher(String, '/synchronized_data', 10)

        # CvBridgeの初期化
        self.bridge = CvBridge()

        # データ保存用
        self.emotion_result_data = ""

    def sync_callback(self, image_msg_1, image_msg_2, image_msg_3, emotion_image_msg):
        # 画像データを処理（ここでは省略）
        images = {
            'camera_1': self.bridge.imgmsg_to_cv2(image_msg_1, 'bgr8'),
            'camera_2': self.bridge.imgmsg_to_cv2(image_msg_2, 'bgr8'),
            'camera_3': self.bridge.imgmsg_to_cv2(image_msg_3, 'bgr8'),
            'emotion_image': self.bridge.imgmsg_to_cv2(emotion_image_msg, 'bgr8')
        }

        # 結果のパブリッシュ（`emotion_result_data`は外部で設定された結果）
        synchronized_data = f"Emotion: {self.emotion_result_data}"
        self.get_logger().info(synchronized_data)

        # 新しいトピックに同期データをパブリッシュ
        self.publisher.publish(String(data=synchronized_data))

    def emotion_result_callback(self, msg):
        # `/emotion_result`トピックから受け取ったデータを保存
        self.emotion_result_data = msg.data

def main(args=None):
    rclpy.init(args=args)
    sync_node = SyncNode()
    rclpy.spin(sync_node)
    sync_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__('video_publisher_node')

        # ELPカメラ（表情認識済み画像）の購読
        self.subscription_elp = self.create_subscription(
            Image, 'camera/image_with_fer', self.elp_callback, 10)
        
        # OAK-D LiteのRGB画像の購読（複数カメラ対応）
        self.subscription_oakd_1 = self.create_subscription(
            Image, '/sensor/depthai/rgb_image_1', self.oakd_callback_1, 10)
        self.subscription_oakd_2 = self.create_subscription(
            Image, '/sensor/depthai/rgb_image_2', self.oakd_callback_2, 10)
        
        # 統合画像をパブリッシュするトピック
        self.publisher_ = self.create_publisher(Image, '/output/video_stream', 10)
        
        self.br = CvBridge()

        # 各フレームを格納する変数
        self.elp_frame = None
        self.oakd_frame_1 = None
        self.oakd_frame_2 = None

    def elp_callback(self, data):
        # ELPカメラのフレームを保存
        self.elp_frame = self.br.imgmsg_to_cv2(data, 'bgr8')
        self.combine_and_publish()

    def oakd_callback_1(self, data):
        # OAK-D Liteの1つ目のカメラのフレームを保存
        self.oakd_frame_1 = self.br.imgmsg_to_cv2(data, 'bgr8')
        self.combine_and_publish()

    def oakd_callback_2(self, data):
        # OAK-D Liteの2つ目のカメラのフレームを保存
        self.oakd_frame_2 = self.br.imgmsg_to_cv2(data, 'bgr8')
        self.combine_and_publish()

    def combine_and_publish(self):
        # 全てのフレームが揃っている場合に映像を統合
        if self.elp_frame is not None and self.oakd_frame_1 is not None and self.oakd_frame_2 is not None:
            # ELPカメラの画像を大きくする（例えば2倍に拡大）
            elp_resized = cv2.resize(self.elp_frame, (self.elp_frame.shape[1] * 2, self.elp_frame.shape[0] * 2))
            
            # 各OAK-D Liteの画像をそのままのサイズで使用
            oakd_height_1, oakd_width_1 = self.oakd_frame_1.shape[:2]
            oakd_height_2, oakd_width_2 = self.oakd_frame_2.shape[:2]
            
            # 統合フレームのサイズを決定（左に大きなELP、右側に上下にOAK-D Liteを並べる）
            combined_height = max(elp_resized.shape[0], oakd_height_1 + oakd_height_2)
            combined_width = elp_resized.shape[1] + max(oakd_width_1, oakd_width_2)

            # 黒背景で統合フレームを作成
            combined_frame = np.zeros((combined_height, combined_width, 3), dtype=np.uint8)

            # 左側に拡大したELPカメラの画像を配置
            combined_frame[:elp_resized.shape[0], :elp_resized.shape[1]] = elp_resized

            # 右側にOAK-D Liteの画像を上下に配置
            combined_frame[:oakd_height_1, elp_resized.shape[1]:elp_resized.shape[1] + oakd_width_1] = self.oakd_frame_1
            combined_frame[oakd_height_1:oakd_height_1 + oakd_height_2, elp_resized.shape[1]:elp_resized.shape[1] + oakd_width_2] = self.oakd_frame_2
            
            # パブリッシュ用のメッセージに変換してトピックにパブリッシュ
            ros_image_msg = self.br.cv2_to_imgmsg(combined_frame, encoding="bgr8")
            self.publisher_.publish(ros_image_msg)

def main(args=None):
    rclpy.init(args=args)
    video_publisher_node = VideoPublisherNode()
    rclpy.spin(video_publisher_node)
    video_publisher_node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

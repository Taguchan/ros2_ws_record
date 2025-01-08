import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from fer import FER

class EmotionRecognitionNode(Node):
    def __init__(self):
        super().__init__('emotion_recognition_node')
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Image, 'camera/image_with_fer', 20)
        self.br = CvBridge()
        self.emotion_detector = FER(mtcnn=True)

    def listener_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data, 'bgr8')

        # 表情認識処理
        result = self.emotion_detector.detect_emotions(frame)

        if result:
            # 最初に検出された顔だけを処理
            face = result[0]
            (x, y, w, h) = face["box"]
            emotions = face["emotions"]
            emotion, score = max(emotions.items(), key=lambda item: item[1])

            # 検出された顔に矩形を描画
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 1)
            # 表情とスコアを表示
            cv2.putText(frame, f'{emotion}: {score:.2f}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            # 表情とスコアをターミナルに出力
            self.get_logger().info(f'Emotion: {emotion}, Score: {score:.2f}')
        else:
            self.get_logger().info('No faces detected')

        # パブリッシュ用のメッセージに変換
        msg = self.br.cv2_to_imgmsg(frame, 'bgr8')
        msg.header.frame_id = "camera_frame"  # フレームIDを設定
        self.publisher_.publish(msg)

        # # フレームを表示
        # cv2.imshow("Emotion Recognition", frame)
        # if cv2.waitKey(1) == ord('q'):
        #     rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    emotion_recognition_node = EmotionRecognitionNode()
    rclpy.spin(emotion_recognition_node)
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

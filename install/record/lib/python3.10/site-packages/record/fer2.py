import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from fer import FER
import os
import pandas as pd

class FERNode(Node):
    def __init__(self):
        super().__init__('fer_node')

        # サブスクライバー：カメラ画像を購読
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # カメラ画像のトピック名
            self.image_callback,
            10)
        self.subscription  # 未使用警告を避けるために保持

        # パブリッシャー：表情認識結果と画像をパブリッシュ
        self.result_publisher = self.create_publisher(String, '/emotion_result', 10)
        self.image_publisher = self.create_publisher(Image, '/emotion_image', 10)

        # OpenCVとFERライブラリの初期化
        self.bridge = CvBridge()
        self.face_detector = FER(mtcnn=True)

        # データ保存用リスト
        self.emotion_data = []

        self.get_logger().info('FER Node with Data Saving started.')

    def image_callback(self, msg):
        try:
            # ROS ImageメッセージをOpenCV形式に変換
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 表情認識を実行（顔ごとの情報を取得）
            results = self.face_detector.detect_emotions(frame)

            overall_result_str = ""
            frame_data = {"frame": len(self.emotion_data) + 1}

            # 各顔について処理
            for result in results:
                # バウンディングボックス座標
                box = result["box"]  # [x, y, width, height]
                x, y, w, h = box

                # 最も優勢な表情を取得
                emotions = result["emotions"]
                top_emotion = max(emotions, key=emotions.get)
                score = emotions[top_emotion]

                # 感情データを保存
                for emotion, value in emotions.items():
                    frame_data[emotion] = value

                # バウンディングボックスを描画
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 0), 2)

                # 表情結果を描画
                result_str = f"{top_emotion}: {score:.2f}"
                overall_result_str += result_str + "; "
                cv2.putText(frame, result_str, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

            # 表情認識結果がない場合の処理
            if not results:
                overall_result_str = "No faces detected or no emotion detected"

            # データをリストに追加
            self.emotion_data.append(frame_data)

            # 結果をパブリッシュ
            self.result_publisher.publish(String(data=overall_result_str))
            self.get_logger().info(overall_result_str)

            # 画像にタイムスタンプを設定
            emotion_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            emotion_image_msg.header.stamp = self.get_clock().now().to_msg()  # 現在時刻を設定
            emotion_image_msg.header.frame_id = "camera_frame"  # フレームIDを設定

            # 画像をROSトピックにパブリッシュ
            self.image_publisher.publish(emotion_image_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def save_data(self):
        # ディレクトリの作成（存在しない場合）
        output_dir = "output"
        os.makedirs(output_dir, exist_ok=True)
        # プロセス終了時にCSVファイルに保存
        df = pd.DataFrame(self.emotion_data)
        df.to_csv("output/emotion_data.csv", index=False)
        self.get_logger().info("Emotion data saved to output/emotion_data.csv")

def main(args=None):
    rclpy.init(args=args)
    node = FERNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_data()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

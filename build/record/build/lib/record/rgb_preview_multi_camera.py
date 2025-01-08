import rclpy
from rclpy.executors import MultiThreadedExecutor
import cv2
import depthai as dai
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RgbPreview(Node):
    def __init__(self, device_id, topic_name):
        super().__init__('rgb_preview_' + device_id)
        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        self.device_id = device_id

        # パイプラインを作成
        self.pipeline = dai.Pipeline()

        # ソースと出力を定義
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        xoutRgb = self.pipeline.create(dai.node.XLinkOut)

        # 出力ストリーム名を設定
        xoutRgb.setStreamName("video")  # ストリーム名を「video」に設定

        # プロパティ設定
        # setPreviewSize()を削除して、フル解像度での出力を使用
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)  # 1080pの解像度に設定
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        # リンク設定
        camRgb.video.link(xoutRgb.input)

        # 特定のデバイスIDを使ってデバイスに接続し、パイプラインを開始
        self.device = dai.Device(self.pipeline, dai.DeviceInfo(self.device_id))
        print(f"Connected to device: {self.device_id}")
        print('Connected cameras:', self.device.getConnectedCameraFeatures())
        print('Usb speed:', self.device.getUsbSpeed().name)

        # 上で定義した出力からRGBフレームを取得するためのキューを設定
        self.qRgb = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)

        # ROS パブリッシャー設定
        self.bridge = CvBridge()

        # 画像をパブリッシュするためのタイマーを設定
        timer_period = 0.00001  # 秒単位のタイマー間隔
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # 出力キューからRGBデータを取得
        inRgb = self.qRgb.tryGet()
        if inRgb:
            # 取得したフレームをOpenCV形式に変換
            cv_frame = inRgb.getCvFrame()

            # フレームをROSイメージメッセージに変換
            ros_image_msg = self.bridge.cv2_to_imgmsg(cv_frame, encoding="bgr8")
            # 変換したメッセージをパブリッシュ
            self.publisher_.publish(ros_image_msg)


def main(args=None):
    # rclpyを初期化
    rclpy.init(args=args)

    # 利用可能なデバイスを取得
    available_devices = dai.Device.getAllAvailableDevices()
    if not available_devices:
        print("No OAK-D Lite devices connected.")
        return

    # デバイスIDのリストを作成
    device_ids = [device.getMxId() for device in available_devices]

    # 各デバイスに対して一意のトピックを持つRgbPreviewインスタンスを開始
    nodes = []
    for i, device_id in enumerate(device_ids):
        topic_name = f'/sensor/depthai/rgb_image_{i+1}'
        node = RgbPreview(device_id, topic_name)
        nodes.append(node)

    # 複数のノードを処理するためにMultiThreadedExecutorを使用
    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    # 全てのノードを並行して実行
    executor.spin()

    # クリーンアップ
    for node in nodes:
        node.destroy_node()

    # rclpyをシャットダウン
    rclpy.shutdown()


if __name__ == '__main__':
    main()

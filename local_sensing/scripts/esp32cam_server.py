#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import requests
import numpy as np
from urllib.parse import urljoin
from threading import Thread
import cv2

HOST = "http://192.168.2.101"
STREAM_PORT = "81"
CAMERA_PORT = "80"
ENDPOINT_STREAM = "/stream"
ENDPOINT_RESOLUTION = "/resolution"
ENDPOINT_STATUS = "/status"
ENDPOINT_CMD = "/control"
url_stream = urljoin(f"{HOST}:{STREAM_PORT}", ENDPOINT_STREAM)
url_resolution = urljoin(f"{HOST}:{CAMERA_PORT}", ENDPOINT_RESOLUTION)
url_status = urljoin(f"{HOST}:{CAMERA_PORT}", ENDPOINT_STATUS)
url_cmd = urljoin(f"{HOST}:{CAMERA_PORT}", ENDPOINT_CMD)

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.bridge = CvBridge()
        self.declare_parameter('visualize', False)
        self.visualize = self.get_parameter('visualize').value
        
        # Set up camera parameters
        self.setup_camera()
        
        # Start streaming thread
        self.thread = Thread(target=self.stream_video)
        self.thread.daemon = True
        self.thread.start()

    def setup_camera(self):
        self.set_camera_parameter("framesize", 0)
        self.set_camera_parameter("quality", 10)
        self.get_status()

    def get_status(self):
        response = requests.get(url_status)
        if response.ok:
            print("esp32cam status:", response.json())
        else:
            print('Failed to get the status. Status code:', response.status_code)

    def set_resolution(self):
        params = {
        'sx': 0,     # 開始 X 座標
        'sy': 0,     # 開始 Y 座標
        'ex': 1000,   # 終了 X 座標
        'ey': 1000,   # 終了 Y 座標
        'offx': 0,   # X 軸のオフセット
        'offy': 0,   # Y 軸のオフセット
        'tx': 100,   # 全体の横幅
        'ty': 100,   # 全体の高さ
        'ox': 100,   # 出力の横幅
        'oy': 100,   # 出力の高さ
        'scale': 0,  # スケーリングの有無
        'binning': 0 # ビニングの有無
        }
        response = requests.get(url_resolution, params=params)

        if response.ok:
            print('Successfully set the resolution.')
        else:
            print('Failed to set resolution. Status code:', response.status_code)

    def set_camera_parameter(self, variable, value):
        # URLパラメーターを設定
        params = {'var': variable, 'val': str(value)}
        # GETリクエストを送信
        response = requests.get(url_cmd, params=params)
        if response.status_code == 200:
            print(f"Successfully set {variable} to {value}")
        else:
            print(f"Failed to set parameter. Status Code: {response.status_code}, Response: {response.text}")

    def get_frames(self, stream_response):
        # Function to extract MJPEG frames
        content = bytes()
        for chunk in stream_response.iter_content(chunk_size=1024):
            content += chunk
            while True:
                start = content.find(b'\xff\xd8')
                end = content.find(b'\xff\xd9', start)
                if start != -1 and end != -1:
                    jpg = content[start:end+2]
                    content = content[end+2:]
                    yield jpg
                else:
                    break

    def stream_video(self):
        # Add your video streaming code here
        response = requests.get(url_stream, stream=True)
        try:
            for frame_data in self.get_frames(response):
                frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    self.publisher_.publish(msg)
                    if self.visualize:
                        cv2.imshow('frame', frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                else:
                    self.get_logger().info("Received an incomplete frame.")
        finally:
            response.close()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.thread.join()
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
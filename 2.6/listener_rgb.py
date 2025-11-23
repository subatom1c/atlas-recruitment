import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import numpy as np
import cv2

Gst.init(None)

class VideoProcessor(Node):
    def __init__(self):
        super().__init__('video_processor')

        # Subscribe to raw_rgb_matrix topic
        self.subscription = self.create_subscription(
            UInt8MultiArray, 
            'raw_rgb_matrix',
            self.process_frame,
            10
        )

        # Setup sending pipeline
        self.sending_pipeline = Gst.parse_launch(
            "appsrc name=source is-live=true block=true format=time "
            "caps=video/x-raw,format=BGR,width=640,height=480,framerate=30/1 "
            "! videoconvert ! x264enc tune=zerolatency bitrate=500 ! "
            "rtph264pay config-interval=1 pt=96 ! udpsink host=127.0.0.1 port=5000"
        )
        self.appsrc = self.sending_pipeline.get_by_name("source")
        self.sending_pipeline.set_state(Gst.State.PLAYING)
        self.frame_number = 0

    def process_frame(self, msg):
        # Convert ROS2 message data into a NumPy array
        data_frame = np.array(msg.data, dtype=np.uint8)
        # Assuming shape is 480x640x3; otherwise reshape appropriately
        data_frame = data_frame.reshape((480, 640, 3))

        # OpenCV processing
        frame_flipped = cv2.flip(data_frame, 0)

        # Build GstBuffer
        gst_buffer = Gst.Buffer.new_allocate(None, frame_flipped.nbytes, None)
        gst_buffer.fill(0, frame_flipped.tobytes())
        gst_buffer.pts = self.frame_number * (Gst.SECOND // 30)
        gst_buffer.dts = gst_buffer.pts
        gst_buffer.duration = Gst.SECOND // 30
        self.frame_number += 1

        # Push buffer to GStreamer
        ret = self.appsrc.emit("push-buffer", gst_buffer)
        if ret != Gst.FlowReturn.OK:
            self.get_logger().error(f"Error pushing buffer: {ret}")


def main():
    rclpy.init()
    node = VideoProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.sending_pipeline.set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
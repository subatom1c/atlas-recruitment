# Capture Video -> Process it -> Send it to receiving Computer

# 3 different pipelines
# GStreamer reading content
# OpenCV processing GStreamer content
# Gstreamer sending content to receiving computer

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject 
import time
import numpy
import cv2

Gst.init(None)

# Created capturing pipeline with appsink for OpenCV (BGR color format for OpenCV)
capturing_pipeline = Gst.parse_launch(
    "v4l2src ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink name=sink"
)

# Fetch sink object from pipeline
appsink = capturing_pipeline.get_by_name("sink")
appsink.set_property("emit_signals", True)  # Allows to pull samples
appsink.set_property("sync", False)         # Prevents blocking

capturing_pipeline.set_state(Gst.State.PLAYING)

# Create sending pipeline
sending_pipeline = Gst.parse_launch(
    "appsrc name=src is-live=true block=true format=time "
    "caps=video/x-raw,format=BGR,width=640,height=480,framerate=30/1 "
    "! videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency bitrate=500 ! "
    "rtph264pay config-interval=1 pt=96 ! udpsink host=127.0.0.1 port=5000"
)

appsrc = sending_pipeline.get_by_name("src")
sending_pipeline.set_state(Gst.State.PLAYING)

frame_number = 0

try:
    while True:
        
        # GstSample
        sample = appsink.emit("pull-sample")
        if sample is None:
            continue

        # Strip GstBuffer from GstSample
        buf = sample.get_buffer()

        # Making the frame accessible
        accessible, mapped_data = buf.map(Gst.MapFlags.READ)
        if not accessible:
            continue

        # NumPy Array is compatible with OpenCV
        data_frame = numpy.frombuffer(mapped_data.data, dtype=numpy.uint8)

        # Fetch previous frame caps
        caps = sample.get_caps()
        structure = caps.get_structure(0)
        width = structure.get_value("width")
        height = structure.get_value("height")
        format = structure.get_value("format")       

        # Reshape the Array to match previous frame
        data_frame = data_frame.reshape((height, width, 3))    

        # Example OpenCV processing
        frame_flipped = cv2.flip(data_frame, 0)

        # Build the GstBuffer
        gst_buffer = Gst.Buffer.new_wrapped(frame_flipped.tobytes())


        gst_buffer.pts = frame_number * (Gst.SECOND // 30)
        gst_buffer.dts = gst_buffer.pts
        gst_buffer.duration = Gst.SECOND // 30
        frame_number += 1

        ret = appsrc.emit("push-buffer", gst_buffer)
        if ret != Gst.FlowReturn.OK:
            print(f"Error pushing buffer: {ret}")

        # Let GStreamer know you're done with this frame
        buf.unmap(mapped_data)

# Handle sigint        
except KeyboardInterrupt:
    print("\nStopping...")
    capturing_pipeline.set_state(Gst.State.NULL)
    sending_pipeline.set_state(Gst.State.NULL)
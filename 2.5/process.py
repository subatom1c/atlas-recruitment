# Processes both RTSP streams
    # Fetch video from both streams
    # Regulate using threads
    # Process video
    # Send to receiver 

import cv2
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject 
import numpy
Gst.init(None)
import threading


def process_sample(sample, frame_number):
    print("Processing")
    # Get buffer from sample
    buf = sample.get_buffer()

    # Make frame accessible
    accessible, mapped_data = buf.map(Gst.MapFlags.READ)
    if not accessible:
        return
    
    # Conver to Numpy array
    data_frame = numpy.frombuffer(mapped_data.data, dtype=numpy.uint8)

    # Fetch frame caps
    caps = sample.get_caps()
    structure = caps.get_structure(0)
    width = structure.get_value("width")
    height = structure.get_value("height")
    format = structure.get_value("format")       

    data_frame = data_frame.reshape((height, width, 3))    

    # Flip the image vertically
    frame_flipped = cv2.flip(data_frame, 0)

    # Insert frame into buffer
    gst_buffer = Gst.Buffer.new_allocate(None, frame_flipped.nbytes, None)
    gst_buffer.fill(0, frame_flipped.tobytes())

    gst_buffer.pts = frame_number * (Gst.SECOND // 30)
    gst_buffer.dts = gst_buffer.pts
    gst_buffer.duration = Gst.SECOND // 30

    # Release frame
    buf.unmap(mapped_data)
    return gst_buffer

def process_stream(receiving_sink, sending_src):
    frame_number = 0
    try: 
        while True:
            print("Waiting for sample")
            sample = receiving_sink.emit("pull-sample")
            print("Got sample")
            buffer = process_sample(sample)

            ret = sending_src.emit("push-buffer", buffer)
            if ret != Gst.FlowReturn.OK:
                print(f"Error pushing buffer: {ret}")

            frame_number += 1
    except KeyboardInterrupt:
        print("\nStopping...")
        capturing_pipeline.set_state(Gst.State.NULL)
        sending_pipeline.set_state(Gst.State.NULL)   

def main():

    # RTSP server URLs
    rtsp_url_1 = "rtsp://127.0.0.1:8554/test"
    rtsp_url_2 = "rtsp://127.0.0.1:8555/test"

    # Appsink addresses
    appsink_addr_1 = "host=127.0.0.1 port=5000"
    appsink_addr_2 = "host=127.0.0.1 port=5001"

    # RTSP sources
    rtsp_source_1 = (
        "rtspsrc location=" + rtsp_url_1 + " latency=0 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
        "video/x-raw,format=I420 ! appsink name=receiving_sink emit-signals=true max-buffers=1 drop=true"
    )
    rtsp_source_2 = (
        "rtspsrc location=" + rtsp_url_2 + " latency=0 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
        "video/x-raw,format=I420 ! appsink name=receiving_sink emit-signals=true max-buffers=1 drop=true"
    )

    # Appsink
    rtp_output_1 = (
        "appsrc name=sending_src ! videoconvert ! x264enc tune=zerolatency ! "
        "rtph264pay pt=96 ! udpsink " + appsink_addr_1
    )
    rtp_output_2 = (
        "appsrc name=sending_src ! videoconvert ! x264enc tune=zerolatency ! "
        "rtph264pay pt=96 ! udpsink " + appsink_addr_2
    )
    
    # Build pipelines 
    # Change names to just receiver and sender maybe, kinda confusing rn
    receiving_pipeline_1 = Gst.parse_launch(rtsp_source_1)
    sending_pipeline_1 = Gst.parse_launch(rtp_output_1)
    receiving_pipeline_2 = Gst.parse_launch(rtsp_source_2)
    sending_pipeline_2 = Gst.parse_launch(rtp_output_2)

    receiving_sink_1 = receiving_pipeline.get_by_name("receiving_sink_1")
    sending_src_1 = sending_pipeline.get_by_name("sending_src_1")
    receiving_sink_2 = receiving_pipeline.get_by_name("receiving_sink_2")
    sending_src_2 = sending_pipeline.get_by_name("sending_src_2")

    receiving_pipeline_1.set_state(Gst.State.PLAYING)
    sending_pipeline_1.set_state(Gst.State.PLAYING)

    receiving_pipeline_2.set_state(Gst.State.PLAYING)
    sending_pipeline_2.set_state(Gst.State.PLAYING)

    frame_number_1 = 0
    frame_number_2 = 0




if __name__ == "__main__":
    main()
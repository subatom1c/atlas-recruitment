# Computer to process the received video 
    # Fetch frames from RTSP server
    # Process frames
    # Send them to receiver computer 

import cv2
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject 
import numpy
Gst.init(None)


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


def main():

    # RTSP server URL
    rtsp_url = "rtsp://127.0.0.1:8554/test"

    # Appsink address
    appsink_addr = "host=127.0.0.1 port=5000"


    # RTSP source
    rtsp_source = (
        "rtspsrc location=" + rtsp_url + " latency=0 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
        "video/x-raw,format=I420 ! appsink name=receiving_sink emit-signals=true max-buffers=1 drop=true"
    )

    # Appsink
    rtp_output = (
        "appsrc name=sending_src ! videoconvert ! x264enc tune=zerolatency ! "
        "rtph264pay pt=96 ! udpsink " + appsink_addr
    )
    
    # Build pipelines
    receiving_pipeline = Gst.parse_launch(rtsp_source)
    sending_pipeline = Gst.parse_launch(rtp_output)

    receiving_sink = receiving_pipeline.get_by_name("receiving_sink")
    sending_src = sending_pipeline.get_by_name("sending_src")

    frame_number = 0

    receiving_pipeline.set_state(Gst.State.PLAYING)
    sending_pipeline.set_state(Gst.State.PLAYING)
             
    try: 
        while True:
            print("Waiting for sample")
            sample = receiving_sink.emit("pull-sample")
            print("Got sample")
            buffer = process_sample(sample, frame_number)

            ret = sending_src.emit("push-buffer", buffer)
            if ret != Gst.FlowReturn.OK:
                print(f"Error pushing buffer: {ret}")

            frame_number += 1
    except KeyboardInterrupt:
        print("\nStopping...")
        receiving_pipeline.set_state(Gst.State.NULL)
        sending_pipeline.set_state(Gst.State.NULL)            


if __name__ == "__main__":
    main()
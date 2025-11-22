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

# RTSP server URLs
rtsp_url_1 = "rtsp://127.0.0.1:8554/test"
rtsp_url_2 = "rtsp://127.0.0.1:8555/test"

# Appsink addresses
appsink_addr_1 = "host=127.0.0.1 port=5000"
appsink_addr_2 = "host=127.0.0.1 port=5001"


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

def process_stream(appsink, appsource):
    frame_number = 0
    while True:
        print("Waiting for sample")
        sample = appsink.emit("pull-sample")
        print("Got sample")
        buffer = process_sample(sample, frame_number)

        ret = appsource.emit("push-buffer", buffer)
        if ret != Gst.FlowReturn.OK:
            print(f"Error pushing buffer: {ret}")

        frame_number += 1

def initialize_pipelines(urls, sink_addresses):
    appsources = []
    appsinks = []
    source_pipelines = []
    sink_pipelines = []

    for i in range(2):
        source_str = (
        "rtspsrc location=" + urls[i] + " latency=0 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
        "video/x-raw,format=I420 ! appsink name=source emit-signals=true max-buffers=1 drop=true"
        )

        sink_str = (
        "appsrc name=sink ! videoconvert ! x264enc tune=zerolatency ! "
        "rtph264pay pt=96 ! udpsink " + sinks_addresses[i]
        )          

        source_pipeline = Gst.parse_launch(source_str)
        sink_pipeline = Gst.parse_launch(sink_str)
        
        appsource = source_pipeline.get_by_name("source")
        appsink = sink_pipeline.get_by_name("sink")

        source_pipeline.set_state(Gst.State.PLAYING)
        sink_pipeline.set_state(Gst.State.PLAYING)

        appsinks.append(appsink)
        appsources.append(appsource)
        source_pipelines.append(source_pipeline)
        sink_pipelines.append(sink_pipeline)

    return source_pipelines, sink_pipelines, appsources, appsinks


def main():

    urls = [rtsp_url_1, rtsp_url_2]
    sink_addresses = [appsink_addr_1, appsink_addr_2]
    
    source_pipelines, sink_pipelines, appsources, appsinks = initialize_pipelines(urls, sink_addresses)

    threads = []
    for i in range(2):
        thread = threading.Thread(target=process_stream, args=(appsources[i], appsinks[i]))
        thread.start()
        threads.append(thread)

    try:
        for thread in threads:
            thread.join()
    except KeyboardInterrupt:
        print("Stopping all pipelines...")
        for p in source_pipelines + sink_pipelines:
            p.set_state(Gst.State.NULL)

if __name__ == "__main__":
    main()
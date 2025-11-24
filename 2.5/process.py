import cv2
import gi
import sys
import threading
import numpy

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject 

Gst.init(None)

# --- CONFIGURATION ---
WIDTH = 1280
HEIGHT = 720
# Strict caps for the application side (OpenCV expects this exact size)
CAPS_STR = f"video/x-raw,format=BGR,width={WIDTH},height={HEIGHT},framerate=30/1"

# RTSP server URLs
rtsp_url_1 = "rtsp://192.168.1.150:8554/mystream"
rtsp_url_2 = "rtsp://192.168.1.150:8555/mystream"

# Appsink addresses
appsink_addr_1 = "host=192.168.1.150 port=5000"
appsink_addr_2 = "host=192.168.1.150 port=5001"


def process_sample(sample, stream_id):

    buf = sample.get_buffer()

    caps = sample.get_caps()
    structure = caps.get_structure(0)
    width = structure.get_value("width")
    height = structure.get_value("height")       

    accessible, mapped_data = buf.map(Gst.MapFlags.READ)
    if not accessible:
        return None
    
    data_frame = numpy.ndarray(
        shape=(height, width, 3),
        dtype=numpy.uint8,
        buffer=mapped_data.data
    )

    if stream_id == 0:
        # Stream 1: Flip
        frame_processed = cv2.flip(data_frame, 0)
    else:
        # Stream 2: Blur
        frame_processed = cv2.blur(data_frame, (20, 20))

    gst_buffer = Gst.Buffer.new_allocate(None, frame_processed.nbytes, None)
    gst_buffer.fill(0, frame_processed.tobytes())

    buf.unmap(mapped_data)
    
    return gst_buffer


def process_stream(gst_receiver, gst_sender, pipeline_source, stream_id):
    print(f"Stream {stream_id}: Thread started. Waiting for connection...")
    
    while True:
        # try-pull-sample with a 500ms timeout
        sample = gst_receiver.emit("try-pull-sample", 500 * 1000 * 1000)
        
        if sample:
            buffer = process_sample(sample, stream_id)

            if buffer:
                ret = gst_sender.emit("push-buffer", buffer)
                if ret != Gst.FlowReturn.OK:
                    print(f"Stream {stream_id}: Error pushing buffer: {ret}")


def initialize_pipelines(urls, sink_addresses):
    appsources = [] 
    appsinks = []   
    source_pipelines = []
    sink_pipelines = []

    for i in range(len(urls)):
        
        source_str = (
            "rtspsrc location=" + urls[i] + " latency=200 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "videoconvert ! videoscale ! videorate ! "
            f"{CAPS_STR} ! appsink name=source emit-signals=true max-buffers=1 drop=true"
        )

        # OUTPUT
        sink_str = (
            f"appsrc name=sink caps={CAPS_STR} stream-type=0 format=3 do-timestamp=true ! "
            "videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30 ! "
            "rtph264pay pt=96 ! udpsink " + sink_addresses[i] + " sync=false"
        )          

        print(f"DEBUG: Initializing Stream {i} Input: {source_str}")
        
        source_pipeline = Gst.parse_launch(source_str)
        sink_pipeline = Gst.parse_launch(sink_str)
        
        appsink_element = source_pipeline.get_by_name("source")
        appsrc_element = sink_pipeline.get_by_name("sink")

        source_pipeline.set_state(Gst.State.PLAYING)
        sink_pipeline.set_state(Gst.State.PLAYING)

        appsinks.append(appsink_element)     
        appsources.append(appsrc_element)    
        source_pipelines.append(source_pipeline)
        sink_pipelines.append(sink_pipeline)

    return source_pipelines, sink_pipelines, appsources, appsinks


def main():
    urls = [rtsp_url_1, rtsp_url_2]
    sink_addresses = [appsink_addr_1, appsink_addr_2]
    
    source_pipelines, sink_pipelines, senders, receivers = initialize_pipelines(urls, sink_addresses)

    threads = []
    for i in range(len(urls)):
        thread = threading.Thread(target=process_stream, args=(receivers[i], senders[i], source_pipelines[i], i))
        thread.start()
        threads.append(thread)

    try:
        for thread in threads:
            thread.join()
    except KeyboardInterrupt:
        print("\nStopping all pipelines...")
        for p in source_pipelines + sink_pipelines:
            p.set_state(Gst.State.NULL)

if __name__ == "__main__":
    main()
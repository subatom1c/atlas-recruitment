import cv2
import gi
import sys
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject 
import numpy

Gst.init(None)

def process_sample(sample):
    # Get buffer from sample
    buf = sample.get_buffer()
    
    # Fetch frame caps to ensure we know the size dynamically
    caps = sample.get_caps()
    structure = caps.get_structure(0)
    width = structure.get_value("width")
    height = structure.get_value("height")             

    # Make frame accessible
    accessible, mapped_data = buf.map(Gst.MapFlags.READ)
    if not accessible:
        return None
    
    # Convert to Numpy array
    data_frame = numpy.ndarray(
        shape=(height, width, 3),
        dtype=numpy.uint8,
        buffer=mapped_data.data
    )

    # Flip the image vertically
    frame_processed = cv2.flip(data_frame, 0)

    # Create new GStreamer buffer
    gst_buffer = Gst.Buffer.new_allocate(None, frame_processed.nbytes, None)
    gst_buffer.fill(0, frame_processed.tobytes())
    
    # Release original frame
    buf.unmap(mapped_data)
    
    return gst_buffer

def main():
    # RTSP Camera IP
    rtsp_url = "rtsp://192.168.1.150:8554/mystream"

    # DESTINATION IP
    target_ip = "192.168.1.150" 
    target_port = "5000"

    # Matches the camera resolution
    width = 1280
    height = 720
    raw_video_caps = f"video/x-raw,format=BGR,width={width},height={height},framerate=30/1"

    # Input Pipeline
    rtsp_source = (
        f"rtspsrc location={rtsp_url} latency=200 protocols=tcp ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
        f"video/x-raw,format=BGR ! appsink name=receiving_sink emit-signals=true max-buffers=1 drop=true"
    )

    # Output Pipeline
    rtp_output = (
        f"appsrc name=sending_src caps={raw_video_caps} stream-type=0 format=3 do-timestamp=true ! "
        "videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30 ! "
        f"rtph264pay config-interval=1 pt=96 ! udpsink host={target_ip} port={target_port} sync=false"
    )
    
    print("Starting Pipelines...")
    receiving_pipeline = Gst.parse_launch(rtsp_source)
    sending_pipeline = Gst.parse_launch(rtp_output)

    receiving_sink = receiving_pipeline.get_by_name("receiving_sink")
    sending_src = sending_pipeline.get_by_name("sending_src")

    receiving_pipeline.set_state(Gst.State.PLAYING)
    sending_pipeline.set_state(Gst.State.PLAYING)
             
    try: 
        while True:
            # This blocks until a frame is available
            sample = receiving_sink.emit("pull-sample")
            
            if sample:
                buffer = process_sample(sample)
                
                if buffer:
                    # Push buffer into output pipeline
                    ret = sending_src.emit("push-buffer", buffer)
                    if ret != Gst.FlowReturn.OK:
                        print(f"Error pushing buffer: {ret}")
            else:
                print("Timeout or no sample received.")

    except KeyboardInterrupt:
        print("\nStopping...")
        receiving_pipeline.set_state(Gst.State.NULL)
        sending_pipeline.set_state(Gst.State.NULL)            

if __name__ == "__main__":
    main()
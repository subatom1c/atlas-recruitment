# Computer that receives the video 
    # GStreamer pipeline that reads the video and shows (very simple)

gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink
# GStreamer threaded pipelines

# Stream 1
gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp, encoding-name=H264, payload=96" ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink

# Stream 2
gst-launch-1.0 udpsrc port=5001 caps="application/x-rtp, encoding-name=H264, payload=96" ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink

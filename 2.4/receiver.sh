# Computer that receives the video 
    # GStreamer pipeline that reads the video and shows (very simple)

gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink

# In windows
"C:\Program Files\gstreamer\1.0\mingw_x86_64\bin\gst-launch-1.0.exe" -v ^
udpsrc port=5000 caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ! ^
rtph264depay ! avdec_h264 ! queue ! videoconvert ! queue ! autovideosink
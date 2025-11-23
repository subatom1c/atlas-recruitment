# Test-launch wraps RTP stream in RTSP stream
# rtsp://10.0.2.15:8554/test is the rtsp server

# To run from 2.4 dir
# gst-rtsp-server/examples/test-launch "( videotestsrc pattern=balls ! videoconvert ! x264enc tune=zerolatency ! rtph264pay name=pay0 pt=96 )"


test-launch "( videotestsrc pattern=balls ! videoconvert ! x264enc tune=zerolatency ! rtph264pay name=pay0 pt=96 )"

# Using in windows 
"C:\Program Files\gstreamer\1.0\mingw_x86_64\bin\gst-launch-1.0.exe" -v ^
udpsrc port=5000 caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ! ^
rtph264depay ! avdec_h264 ! queue ! videoconvert ! queue ! autovideosink
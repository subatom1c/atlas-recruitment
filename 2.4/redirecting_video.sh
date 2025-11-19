# Test-launch wraps RTP stream in RTSP stream
# rtsp://10.0.2.15:8554/test is the rtsp server

# To run from 2.4 dir
# gst-rtsp-server/examples/test-launch "( videotestsrc pattern=balls ! videoconvert ! x264enc tune=zerolatency ! rtph264pay name=pay0 pt=96 )"


test-launch "( videotestsrc pattern=balls ! videoconvert ! x264enc tune=zerolatency ! rtph264pay name=pay0 pt=96 )"

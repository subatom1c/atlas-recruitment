# Test-launch wraps RTP stream in RTSP stream
# rtsp://10.0.2.15:8554/test is the rtsp server

# To run from 2.4 dir
# gst-rtsp-server/examples/test-launch "( videotestsrc pattern=balls ! videoconvert ! x264enc tune=zerolatency ! rtph264pay name=pay0 pt=96 )"


test-launch "( videotestsrc pattern=balls ! videoconvert ! x264enc tune=zerolatency ! rtph264pay name=pay0 pt=96 )"

# In windows

# Creating a RTSP server
mediamtx.exe

# Sending OBS virtual camera to RTSP server
ffmpeg.exe -f dshow -i video="OBS Virtual Camera" -pix_fmt yuv420p -vcodec libx264 -preset ultrafast -tune zerolatency -framerate 30 -g 30 -f rtsp rtsp://127.0.0.1:8554/mystream

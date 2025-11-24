# Create 2 different RTSP streams on ports 8554 and 8555

test-launch --port=8554 "( videotestsrc pattern=smpte ! videoconvert ! x264enc tune=zerolatency ! rtph264pay name=pay0 pt=96 )"
test-launch --port=8555 "( videotestsrc pattern=smpte ! videoconvert ! x264enc tune=zerolatency ! rtph264pay name=pay0 pt=96 )"

# In windows

# Creating a RTSP server
mediamtx.exe

# Sending OBS virtual camera to RTSP server
ffmpeg.exe -f dshow -i video="OBS Virtual Camera" -pix_fmt yuv420p -vcodec libx264 -preset ultrafast -tune zerolatency -framerate 30 -g 30 -f rtsp rtsp://127.0.0.1:8554/mystream

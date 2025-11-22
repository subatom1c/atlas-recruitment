# Create 2 different RTSP streams on ports 8554 and 8555

test-launch --port=8554 "( videotestsrc pattern=smpte ! videoconvert ! x264enc tune=zerolatency ! rtph264pay name=pay0 pt=96 )"
test-launch --port=8555 "( videotestsrc pattern=smpte ! videoconvert ! x264enc tune=zerolatency ! rtph264pay name=pay0 pt=96 )"

# Use GStreamer to listen to the socket and reproduce automatically video

# udpsrc port=5000 -> We listen all network interfaces on port 5000

# We need to tell GStreamer the specifications of the packets we're receiving
    # application/x-rtp  -> We're using RTP protocol
    # encoding-name=H264 -> Enconded with H.264
    # payload=96         -> Payload(codec identifier) = 96(dynamic) since it's not a fixed codec

# Convert RTP packets to H.264 byte stream
    # rtph264depay -> Basically assembling messages into a stream since RTP breaks down stream into messages

# avdec_h264 -> Decode H.264 enconded byte stream to video frames

# videoconvert -> Convert into compatible format with sink

# Display the frames

gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink
import gi
gi.require_version('Gst', '1.0') # Specify version

# Gst - GStreamer Python API
# GObject - Object for GStreamer
from gi.repository import Gst, GObject 
import time

# Initialize GStreamer library 
Gst.init(None)

# Create pipeline
pipeline = Gst.parse_launch("v4l2src ! videoconvert ! autovideosink")

# Start the video
pipeline.set_state(Gst.State.PLAYING)

time.sleep(5)

# Close the pipeline
pipeline.set_state(Gst.State.NULL)
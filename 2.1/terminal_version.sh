#!/bin/bash
# gst-launch-1.0: command to create and run a video pipeline
# v4l2src: webcam source
# videoconvert: converts formats to make sure source and sink are compatible 
# autovideosink: displays the video output
 
gst-launch-1.0 v4l2src ! videoconvert ! autovideosink
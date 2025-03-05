### Launch Gstreamer microphone
roslaunch audio_capture capture.launch

gst-launch-1.0 alsasrc ! audioconvert ! audioresample ! alsasink  (standalone no ROS)

### add library to ~/.bashrc
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"path to person-tracking-drone"/lib
# reload:
sudo modprobe -r uvcvideo
sudo modprobe uvcvideo

# check:
v4l2-ctl --list-devices

#!/bin/bash

echo ""
echo "Installing udev files"
echo ""

sudo cp `rospack find elp_stereo_camera`/udev/90-elp-stereo-camera.rules /etc/udev/rules.d


echo ""
echo "Restarting udev"
echo ""

sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

echo ""
echo "Check /dev/elp_stereo"
echo ""

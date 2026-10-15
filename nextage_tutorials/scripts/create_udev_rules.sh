#!/bin/bash

echo ""
echo "This scripts copies udev rules for dual hands to /etc/udev/rules.d"
echo ""

sudo cp `rospack find nextage_tutorials`/udev/99-elp-camera.rules /etc/udev/rules.d
sudo cp `rospack find nextage_tutorials`/udev/91-force_sensors.rules /etc/udev/rules.d
echo ""
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart

sudo udevadm control --reload-rules
sudo udevadm trigger

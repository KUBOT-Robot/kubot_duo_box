#!/bin/bash
#Install KUBOT DUO BOX controller package and initialize the environment.

tput setaf 2
echo "Build controller udev rulse..."
tput sgr0

echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0777", SYMLINK+="kubot_duo_box"' >/etc/udev/rules.d/kubot_duo_box.rules
status=$?
if [ $status -eq 0 ]; then
   tput setaf 2
   echo "Build controller udev rulse successfully"
   tput sgr0
else
   tput setaf 1
   echo "Failed to build controller udev rulse!"
   tput sgr0
   exit 1
fi

tput setaf 2
echo "Reload udev rules..."
tput sgr0

sudo udevadm control --reload
sudo udevadm trigger

tput setaf 2
echo "Finish to build KUBOT DUO BOX controller initialize the environment..."
tput sgr0

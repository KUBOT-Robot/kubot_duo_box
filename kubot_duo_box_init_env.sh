#!/bin/bash
#Install KUBOT LED control package and initialize the environment.

tput setaf 2
echo "Build KUBOT LED udev rulse..."
tput sgr0

echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0777", SYMLINK+="kubot_led"' >/etc/udev/rules.d/kubot_led.rules
status=$?
if [ $status -eq 0 ]; then
   tput setaf 2
   echo "Build LED udev rulse successfully"
   tput sgr0
else
   tput setaf 1
   echo "Failed to build LED udev rulse!"
   tput sgr0
   exit 1
fi

tput setaf 2
echo "Reload udev rules..."
tput sgr0

sudo udevadm control --reload
sudo udevadm trigger

tput setaf 2
echo "Finish to build KUBOT LED initialize the environment..."
tput sgr0

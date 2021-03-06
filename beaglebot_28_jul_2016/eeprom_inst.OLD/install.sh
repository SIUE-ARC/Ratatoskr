#!/bin/bash

#fist copy the dtbo to the FW folder
cp ../BB-PRUPID-MOTOR-00A0.dtbo /lib/firmware/

#next write the eeprom on the board
cat ./PRUMOTOR.data > /sys/bus/i2c/devices/1-0054/eeprom 

#make sure that the dtbo script is executable
chmod +x ./dtbo

#place it in the initramfs-tools/hooks folder so it will be ran when doing the update script
cp ./dtbo /etc/initramfs-tools/hooks/

#make a backup of the bootloader just incase
cp /boot/initrd.img-??????-bone?? /boot/initrd.img.bak

#run the update script
/opt/scripts/tools/developers/update_initrd.sh

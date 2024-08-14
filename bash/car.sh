echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777",SYMLINK+="car_usb"' >/etc/udev/rules.d/car_usb.rules

service udev reload
sleep 2
service udev restart



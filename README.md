# Teknic Clearpath-SC ROS Workspace
Under development
# Exar Kernel Driver Install
This specific driver works only with Teknic's communication HUB. It will fix the overlapping issue with the cdc_acm driver.
``` bash 
cd Exar_USB_Serial_Kernel_Driver
make
sudo cp xr_usb_serial_common.ko /lib/modules/`uname -r`
sudo depmod -a
sudo modprobe xr_usb_serial_common
```
# ROS INSTALL
``` bash 
catkin_make
source devel/setup.bash
```

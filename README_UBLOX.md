Using ground truth coordinates for precise GPS synchronization with u-blox lea-m8f requires some further setup:

1. To get the desired behaviour for the USB connection with the u-blox lea-m8f, settings for the ttyACM0 connection need to be modified:
    - /etc/udev/rules.d/99-ttyacms.rules needs to be modified. Add line:
        ATTRS{idVendor}=="1546" ATTRS{idProduct}=="01a8", ENV{ID_MM_DEVICE_IGNORE}="1"
    - execute: udevadm control --reload-rules
    - for detailed explanation check http://linux-tips.org/t/prevent-modem-manager-to-capture-usb-serial-devices/284/2

2. If you want to use u-center on a linux device, mind that
    - u-center version 8.16 is required if you are using the wine version provided by apt
    - A symbolic link for ttyACM0 needs to be created. In ~/.wine/dosdevices run: ln -s /dev/ttyACM0 com1

3. The bitstream for the configuration is composed as follows:


0xB5 0x62    0x06      0x3D     28              2                       0               0x01
|sync chars|CLASS=CFG|ID=TMODE2|Length in bytes|0. Byte: timeMode=fixed|1.Byte reserved|2.Byte:TMODEFlags

Lat                   ...  Lon   Alt             Acc                    |...
4.BYte: Latitude*1e-7|...|8.BYte|12.Byte Alt(cm)|16.Byte: Accuracy in mm|...rest for survey in

29.Byte CK_A 30.Byte CK_B

For detailed information check: u-blox 8 / u-blox M8 Receiver Description Including Protocol Specification p 140-144, 217


#!/bin/sh
IP_SENSOR_1=192.168.100.110 
IP_SENSOR_2=192.168.100.101 
IP_SENSOR_3=192.168.100.106
IP_FUSION_CENTER=192.168.100.102

killall python
gnome-terminal --tab -e "bash -c \"./fusion_center.py; exec bash\"" --title "Fusion center" \
--tab -e "ssh $IP_SENSOR_1 \"killall python; cd /home/$USER/work/sdr/gr-localization/apps; ./receiver.py -i 1 --fusion-center $IP_FUSION_CENTER \"" --title "Receiver 1" \
--tab -e "ssh $IP_SENSOR_2 \"killall python; cd /home/$USER/work/sdr/gr-localization/apps; ./receiver.py -i 2 --fusion-center $IP_FUSION_CENTER \"" --title "Receiver 2" \
--tab -e "ssh $IP_SENSOR_3 \"killall python; cd /home/$USER/work/sdr/gr-localization/apps; ./receiver.py -i 3 --fusion-center $IP_FUSION_CENTER \"" --title "Receiver 3" \
--tab -e "bash -c \"./gui.py; exec bash\"" --title "GUI" \

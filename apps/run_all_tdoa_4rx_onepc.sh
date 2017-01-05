#!/bin/sh

killall python
sleep 1
gnome-terminal \
--tab -e "bash -c \"./fusion_center.py; exec bash\"" --title "Fusion center" \
--tab -e "bash -c \"./receiver.py -i 1 --coordinates-wgs84 50.77931,6.062955 -s F571DD --ntp-server; exec bash\"" --title "Receiver 1" \
--tab -e "bash -c \"./receiver.py -i 2 --coordinates-wgs84 50.77931,6.062955 -s F571B0; exec bash\"" --title "Receiver 2" \
--tab -e "bash -c \"./receiver.py -i 3 --coordinates-wgs84 50.77931,6.062955 -s F571F2; exec bash\"" --title "Receiver 3" \
--tab -e "bash -c \"./receiver.py -i 4 --coordinates-wgs84 50.77931,6.062955 -s F571ED; exec bash\"" --title "Receiver 4"
pwd
./gui.py

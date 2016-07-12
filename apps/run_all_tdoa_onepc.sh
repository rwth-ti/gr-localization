#!/bin/sh

killall python
gnome-terminal --tab -e "bash -c \"./fusion_center.py; exec bash\"" --title "Fusion center" \
--tab -e "bash -c \"./receiver.py -i 1 -s F571DD --ntp-server; exec bash\"" --title "Receiver 1" \
--tab -e "bash -c \"./receiver.py -i 2 -s F571F2; exec bash\"" --title "Receiver 2" \
--tab -e "bash -c \"./receiver.py -i 3 -s F571B0; exec bash\"" --title "Receiver 3"
pwd
./gui.py

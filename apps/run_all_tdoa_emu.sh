#!/bin/sh

killall python
gnome-terminal --tab -e "bash -c \"./fusion_center.py; exec bash\"" --title "Fusion center" \
--tab -e "bash -c \"./receiver_emu.py -i 1 --coordinates-wgs84 50.77933114,6.06295739 --coordinates-m 45,30 --movement-file ../movements/spline.csv; exec bash\"" --title "Receiver 1" \
--tab -e "bash -c \"./receiver_emu.py -i 2 --coordinates-wgs84 50.77943114,6.06295739 --coordinates-m 250,30 --movement-file ../movements/spline.csv; exec bash\"" --title "Receiver 2" \
--tab -e "bash -c \"./receiver_emu.py -i 3 --coordinates-wgs84 50.77933114,6.06296739 --coordinates-m 120,150 --movement-file ../movements/spline.csv; exec bash\"" --title "Receiver 3"
pwd
./gui.py

#!/bin/sh

killall python
gnome-terminal --tab -e "bash -c \"./fusion_center.py; exec bash\"" --title "Fusion center" \
--tab -e "bash -c \"./receiver_emu.py -i 1 --coordinates-m 45,30 --movement-file ../movements/spline_paper.csv; exec bash\"" --title "Receiver 1" \
--tab -e "bash -c \"./receiver_emu.py -i 2 --coordinates-m 250,30 --movement-file ../movements/spline_paper.csv; exec bash\"" --title "Receiver 2" \
--tab -e "bash -c \"./receiver_emu.py -i 3 --coordinates-m 120,150 --movement-file ../movements/spline_paper.csv; exec bash\"" --title "Receiver 3"
pwd
./gui.py

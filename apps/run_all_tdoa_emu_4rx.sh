#!/bin/sh

killall python
gnome-terminal --tab -e "bash -c \"./fusion_center.py; exec bash\"" --title "Fusion center" \
--tab -e "bash -c \"./receiver_emu.py -i 1 --coordinates-m 30,80 --movement-file ../movements/spline_paper.csv; exec bash\"" --title "Receiver 1" \
--tab -e "bash -c \"./receiver_emu.py -i 2 --coordinates-m 200,80 --movement-file ../movements/spline_paper.csv; exec bash\"" --title "Receiver 2" \
--tab -e "bash -c \"./receiver_emu.py -i 3 --coordinates-m 120,150 --movement-file ../movements/spline_paper.csv; exec bash\"" --title "Receiver 3" \
--tab -e "bash -c \"./receiver_emu.py -i 4 --coordinates-m 120,20 --movement-file ../movements/spline_paper.csv; exec bash\"" --title "Receiver 4"
pwd
./gui.py

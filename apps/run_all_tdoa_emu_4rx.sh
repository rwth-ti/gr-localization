#!/bin/sh

killall python
gnome-terminal --tab -e "bash -c \"./fusion_center.py; exec bash\"" --title "Fusion center" \
--tab -e "bash -c \"./receiver_emu.py -i 1 --coordinates-m 30,80 --coordinates-wgs84 50.77890891753905,6.061026275464247 --movement-file ../movements/spline_paper.csv; exec bash\"" --title "Receiver 1" \
--tab -e "bash -c \"./receiver_emu.py -i 2 --coordinates-m 200,80 --coordinates-wgs84 50.77890763166232,6.063436569767161 --movement-file ../movements/spline_paper.csv; exec bash\"" --title "Receiver 2" \
--tab -e "bash -c \"./receiver_emu.py -i 3 --coordinates-m 120,150 --coordinates-wgs84 50.77953748977171,6.062303149728518 --movement-file ../movements/spline_paper.csv; exec bash\"" --title "Receiver 3" \
--tab -e "bash -c \"./receiver_emu.py -i 4 --coordinates-m 120,20 --coordinates-wgs84 50.77836888854142,6.06230159702876 --movement-file ../movements/spline_paper.csv; exec bash\"" --title "Receiver 4"
pwd
./gui.py


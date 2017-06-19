#!/bin/sh

killall python
gnome-terminal --tab -e "bash -c \"./fusion_center.py --samp-rate 100e6 --num-samples 300; exec bash\"" --title "Fusion center" 
gnome-terminal --tab -e "bash -c \"./receiver_emu.py -i 1 --coordinates-m 118.59,105.15 --coordinates-wgs84 50.77913433,6.06228259 --movement-file ../movements/selfloc_average.csv; exec bash\"" --title "Receiver 1" 
sleep 1s 
gnome-terminal --tab -e "bash -c \"./receiver_emu.py -i 2 --coordinates-m 115.07,71.62 --coordinates-wgs84 50.77883295,6.06223227 --movement-file ../movements/selfloc_average.csv; exec bash\"" --title "Receiver 2"
sleep 1s 
gnome-terminal --tab -e "bash -c \"./receiver_emu.py -i 3 --coordinates-m 136.57,65.76 --coordinates-wgs84 50.77878010,6.06253711 --movement-file ../movements/selfloc_average.csv; exec bash\"" --title "Receiver 3"
sleep 1s 
gnome-terminal --tab -e "bash -c \"./receiver_emu.py -i 4 --coordinates-m 151.45,93.28 --coordinates-wgs84 50.77902740,6.06274836 --movement-file ../movements/selfloc_average.csv; exec bash\"" --title "Receiver 4"
pwd
./gui.py

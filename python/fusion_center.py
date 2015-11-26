#!/usr/bin/env python

###############################################################################
# Imports
###############################################################################
from __future__ import print_function
from optparse import OptionParser
from gnuradio.eng_option import eng_option
import sys
import os
from gnuradio import zeromq
import signal
import numpy as np
import receiver_interface
import time
import threading
import json
import rpc_manager as rpc_manager_local
from mpl_toolkits.basemap import Basemap
import copy
import chan94_algorithm
import grid_based_algorithm

class fusion_center():
    def __init__(self, options):
        # give Ctrl+C back to system
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        # socket addresses
        rpc_adr = "tcp://*:6665"

        self.samples_to_receive = int(options.num_samples)
        self.gain = float(options.gain)
        self.frequency = float(options.frequency)
        self.samp_rate = float(options.samp_rate)
        self.bw = float(options.bandwidth)
        self.lo_offset = float(options.lo_offset)
        self.antenna = options.antenna

        self.receivers = {}
        self.guis = {}

        self.estimated_positions = {}

        self.correlation_receivers = ["",""]

        self.delay_history = []
        self.store_results = False
        self.results_file = ""
        self.run_loop = False
        self.localizing = False

        self.bbox = 6.0580,50.7775,6.0690,50.7810
        self.basemap = Basemap(llcrnrlon=self.bbox[0], llcrnrlat=self.bbox[1],
                      urcrnrlon=self.bbox[2], urcrnrlat=self.bbox[3],
                      projection='merc')

        # ZeroMQ
        self.probe_manager = zeromq.probe_manager()
        self.rpc_manager = rpc_manager_local.rpc_manager()
        self.rpc_manager.set_reply_socket(rpc_adr)
        self.rpc_manager.add_interface("register_gui",self.register_gui)
        self.rpc_manager.add_interface("sync_position",self.sync_position)
        self.rpc_manager.add_interface("register_receiver",self.register_receiver)
        self.rpc_manager.add_interface("forward_chat",self.forward_chat)
        self.rpc_manager.add_interface("start_receivers",self.start_receivers)
        self.rpc_manager.add_interface("reset_receivers",self.reset_receivers)
        self.rpc_manager.add_interface("update_receivers",self.update_receivers)
        self.rpc_manager.add_interface("get_gui_gps_position",self.get_gui_gps_position)
        self.rpc_manager.add_interface("localize",self.localize)
        self.rpc_manager.add_interface("localize_loop",self.localize_loop)
        self.rpc_manager.add_interface("start_correlation",self.start_correlation)
        self.rpc_manager.add_interface("start_correlation_loop",self.start_correlation_loop)
        self.rpc_manager.add_interface("stop_loop",self.stop_loop)
        self.rpc_manager.add_interface("set_frequency",self.set_frequency)
        self.rpc_manager.add_interface("set_lo_offset",self.set_lo_offset)
        self.rpc_manager.add_interface("set_samples_to_receive",self.set_samples_to_receive)
        self.rpc_manager.add_interface("set_gain",self.set_gain)
        self.rpc_manager.add_interface("set_samp_rate",self.set_samp_rate)
        self.rpc_manager.add_interface("set_bw",self.set_bw)
        self.rpc_manager.add_interface("set_antenna",self.set_antenna)
        self.rpc_manager.add_interface("set_selected_position",self.set_selected_position)
        self.rpc_manager.start_watcher()

        threading.Thread(target = self.poll_gps_position).start()

    def forward_chat(self, chat):
        for gui in self.guis.values():
            gui.rpc_manager.request("new_chat",[chat])

    def sync_position(self, serial, coordinates):
        self.receivers[serial].coordinates = coordinates
        for gui in self.guis.values():
            gui.rpc_manager.request("sync_position",[serial, coordinates])

    def poll_gps_position(self):
        while True:
            for receiver in self.receivers.values():
                coordinates_gps = receiver.get_gps_position()
                if coordinates_gps != None:
                    receiver.coordinates_gps = self.basemap(coordinates_gps[0],coordinates_gps[1])
            time.sleep(1)

    def get_gui_gps_position(self, serial):
        for gui in self.guis.values():
            threading.Thread(target = gui.rpc_manager.request, args = ("set_gps_position", [serial, self.receivers[serial].coordinates_gps])).start()

    def register_gui(self, ip_addr, hostname, id_gui, first):
        was_not_registered = False
        gui_serial = hostname + str(id_gui)
        if not self.guis.has_key(gui_serial):
            gui = gui_interface("tcp://" + ip_addr + ":" + str(7775 + id_gui), hostname)
            self.guis[gui_serial] = gui
            was_not_registered = True
        else:
            gui = self.guis[gui_serial]
        if first or was_not_registered:
            for serial in self.receivers:
                # request registration of each receiver in gui
                gui.rpc_manager.request("register_receiver",[serial, self.receivers[serial].gain, self.receivers[serial].antenna])
                gui.rpc_manager.request("sync_position",[serial, self.receivers[serial].coordinates])
                gui.rpc_manager.request("set_gps_position",[serial, self.receivers[serial].coordinates_gps])
            gui.rpc_manager.request("set_gui_frequency",[self.frequency])
            gui.rpc_manager.request("set_gui_lo_offset",[self.lo_offset])
            gui.rpc_manager.request("set_gui_samples_to_receive",[self.samples_to_receive])
            gui.rpc_manager.request("set_gui_bw",[self.bw])
            gui.rpc_manager.request("set_gui_samp_rate",[self.samp_rate])
            for gui in self.guis.values():
                for serial in self.guis:
                    # request registration in each gui
                    gui.rpc_manager.request("register_another_gui",[serial])
            print(gui_serial, "registered")
        return self.bbox

    def register_receiver(self, hostname, serial, id_rx, gps, first):
        was_not_registered = False
        rpc_adr = "tcp://" + hostname + ":" + str(6665 + id_rx)
        probe_adr = "tcp://" + hostname + ":" + str(5555 + id_rx)
        if not self.receivers.has_key(serial):
            # create new receiver in fusion center
            self.receivers[serial] = receiver_interface.receiver_interface(rpc_adr, probe_adr)
            was_not_registered = True
        elif first:
            # receiver might have restarted, so remove and add again
            self.receivers.pop(serial)
            self.receivers[serial] = receiver_interface.receiver_interface(rpc_adr, probe_adr)
        if first or was_not_registered:
            # set parameters of receiver in fusion center
            receiver = self.receivers[serial]
            receiver.set_gain(self.gain)
            receiver.set_bw(self.bw)
            receiver.set_samp_rate(self.samp_rate)
            receiver.set_antenna(self.antenna)
            receiver.frequency = self.frequency
            receiver.lo_offset = self.lo_offset
            receiver.samples_to_receive = self.samples_to_receive
            receiver.gps = gps
            self.probe_manager.add_socket(receiver.probe_address, 'complex64', receiver.receive_samples)
            for gui in self.guis.values():
                # request registration in each gui
                gui.rpc_manager.request("register_receiver",[serial, receiver.gain, receiver.antenna])
            self.update_receivers()
            print(serial, "registered")
            #threading.Thread(target = self.finish_register(serial)).start()
            #self.reset_receivers()

    def finish_register(self, serial):
        time.sleep(5)
        self.reset_receivers()
        print(serial, "registered")

    def start_receivers(self, freq, lo_offset, samples_to_receive):
        for receiver in self.receivers.values():
            receiver.frequency = freq
            receiver.lo_offset = lo_offset
            receiver.samples_to_receive = samples_to_receive
            receiver.request_samples()

    def reset_receivers(self):
        #self.update_timer.stop()
        for i in range(0,1000000):
            self.probe_manager.watcher()
        for receiver in self.receivers.values():
            receiver.samples = []
            receiver.first_packet = True
            receiver.reception_complete = False
        #self.connect(self.update_timer, QtCore.SIGNAL("timeout()"), self.probe_manager.watcher)
        #self.update_timer.start(33)
        self.probe_manager.watcher()

    def update_receivers(self):
        for receiver in self.receivers.values():
            receiver.set_gain(receiver.gain)
            receiver.set_bw(self.bw)
            receiver.set_samp_rate(self.samp_rate)
            receiver.set_antenna(receiver.antenna)
            receiver.frequency = self.frequency
            receiver.lo_offset = self.lo_offset
            receiver.samples_to_receive = self.samples_to_receive

    def start_correlation(self, receiver1, receiver2, freq, lo_offset, samples_to_receive):
        threading.Thread(target = self.run_correlation, args = (receiver1, receiver2, freq, lo_offset, samples_to_receive)).start()

    def start_correlation_loop(self, receiver1, receiver2, freq, lo_offset, samples_to_receive):
        self.store_results = True
        self.results_file = "results_" + time.strftime("%d_%m_%y-%H:%M:%S") + ".txt"
        print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
        print("time;delays(1-2,1-3,1-X...);sampling_rate;frequency;samples_to_receive;lo_offset;receivers_positions;selected_positions;receivers_gps;receivers_antenna;receivers_gain;estimated_positions", file=open(self.results_file,"a"))
        print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
        self.run_loop = True
        threading.Thread(target = self.run_correlation, args = (receiver1, receiver2, freq, lo_offset, samples_to_receive)).start()

    def run_correlation(self, receiver1, receiver2, freq, lo_offset, samples_to_receive):
        while True:
            self.start_receivers(freq, lo_offset, samples_to_receive)
            self.correlation_receivers[0] = receiver1
            self.correlation_receivers[1] = receiver2
            if not self.run_loop:
                break
            time.sleep(1.5)

    def localize(self, freq, lo_offset, samples_to_receive):
        if len(self.receivers) > 2:
            self.localizing = True
            threading.Thread(target = self.run_localization, args = (freq, lo_offset, samples_to_receive)).start()

    def localize_loop(self, freq, lo_offset, samples_to_receive):
        self.store_results = True
        self.results_file = "results_" + time.strftime("%d_%m_%y-%H:%M:%S") + ".txt"
        print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
        print("time;delays(1-2,1-3,1-X...);sampling_rate;frequency;samples_to_receive;lo_offset;receivers_positions;selected_positions;receivers_gps;receivers_antenna;receivers_gain;estimated_positions", file=open(self.results_file,"a"))
        print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
        self.run_loop = True
        threading.Thread(target = self.localize, args = (freq, lo_offset, samples_to_receive)).start()

    def run_localization(self, freq, lo_offset, samples_to_receive):
        while True:
            self.start_receivers(freq, lo_offset, samples_to_receive)
            if not self.run_loop:
                break
            time.sleep(1.5)

    def stop_loop(self):
        self.run_loop = False
        self.store_results = False
        self.delay_history = []
        self.localizing = False

    def set_frequency(self, frequency):
        self.frequency = frequency
        for receiver in self.receivers.values():
            receiver.frequency = frequency
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_frequency",[frequency])

    def set_lo_offset(self, lo_offset):
        self.lo_offset = lo_offset
        for receiver in self.receivers.values():
            receiver.lo_offset = lo_offset
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_lo_offset",[lo_offset])

    def set_samples_to_receive(self, samples_to_receive):
        self.samples_to_receive = samples_to_receive
        for receiver in self.receivers.values():
            receiver.samples_to_receive = samples_to_receive
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_samples_to_receive",[samples_to_receive])

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        for receiver in self.receivers.values():
            receiver.samp_rate = samp_rate
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_samp_rate",[samp_rate])

    def set_bw(self, bw):
        self.bw = bw
        for receiver in self.receivers.values():
            receiver.bw = bw
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_bw",[bw])

    def set_gain(self, gain, serial):
        self.receivers[serial].gain = gain
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_gain",[gain, serial])

    def set_antenna(self, antenna, serial):
        self.receivers[serial].antenna = antenna
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_antenna",[antenna, serial])

    def set_selected_position(self, selected_position, serial):
        self.receivers[serial].selected_position = selected_position
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_selected_position",[selected_position, serial])

    def process_results(self):
        while True:
            time.sleep(0.01)
            if all(self.receivers[key].reception_complete for key in self.receivers) and len(self.receivers.items()) > 0:
                receivers = copy.deepcopy(self.receivers)
                if self.localizing:
                    self.estimated_positions["chan"] = chan94_algorithm.localize(receivers.values())
                    self.estimated_positions["grid_based"] = grid_based_algorithm.localize(receivers.values(),np.round(self.basemap(self.bbox[2],self.bbox[3])))
                    for gui in self.guis.values():
                        threading.Thread(target = gui.rpc_manager.request, args = ("set_tx_position", [self.estimated_positions])).start()

                    self.correlation_receivers[0] = receivers.items()[0][0]
                    self.correlation_receivers[1] = receivers.items()[1][0]


                receiver1 = receivers[self.correlation_receivers[0]]
                receiver2 = receivers[self.correlation_receivers[1]]
                correlation, delay = self.correlate(receiver1,receiver2)
                if abs(delay) < 100:
                    self.delay_history.append(delay)
                results = {"receiver1":receiver1.samples,"receiver2":receiver2.samples,"correlation":correlation,"delay":delay,"delay_history":self.delay_history}
                if self.store_results:
                    # build receivers strings for log file
                    receivers_position = "["
                    selected_positions = "["
                    receivers_gps = "["
                    receivers_antenna = "["
                    receivers_gain = "["
                    i = 1
                    for receiver in receivers.values():
                        if i == 1:
                            if receiver.selected_position == "manual":
                                receivers_position = receivers_position + str(receiver.coordinates)
                            else:
                                receivers_position = receivers_position + str(receiver.coordinates_gps)
                            selected_positions = selected_positions + receiver.selected_position
                            receivers_gps = receivers_gps + receiver.gps
                            receivers_antenna = receivers_antenna + receiver.antenna
                            receivers_gain = receivers_gain + str(receiver.gain)
                        else:
                            if receiver.selected_position == "manual":
                                receivers_position = receivers_position + ";" + str(receiver.coordinates)
                            else:
                                receivers_position = receivers_position + ";" + str(receiver.coordinates_gps)
                            selected_positions = selected_positions + ";" + receiver.selected_position
                            receivers_gps = receivers_gps + ";" + receiver.gps
                            receivers_antenna = receivers_antenna + ";" + receiver.antenna
                            receivers_gain = receivers_gain + ";" + str(receiver.gain)
                        i = i + 1
                    receivers_position = receivers_position + "]"
                    selected_positions = selected_positions + "]"
                    receivers_gps = receivers_gps + "]"
                    receivers_antenna = receivers_antenna + "]"
                    receivers_gain = receivers_gain + "]"

                    if self.localizing:
                        print(str(time.time()) + ";" + str(results["delay"]) + ";" + str(self.samp_rate) + ";" + str(self.frequency) + ";" + str(self.samples_to_receive) + ";" + str(self.lo_offset) + ";" + receivers_position + ";" + selected_positions + ";" + receivers_gps + ";" + receivers_antenna + ";" + receivers_gain + ";" + str(self.estimated_positions.items()), file=open(self.results_file,"a"))
                    else:
                        print(str(time.time()) + ";" + str(results["delay"]) + ";" + str(self.samp_rate) + ";" + str(self.frequency) + ";" + str(self.samples_to_receive) + ";" + str(self.lo_offset) + ";" + receivers_position + ";" + selected_positions + ";" + receivers_gps + ";" + receivers_antenna + ";" + receivers_gain + ";" + "{}", file=open(self.results_file,"a"))
                for gui in self.guis.values():
                    gui.rpc_manager.request("get_results",[results])
                for receiver in self.receivers.values():
                    receiver.first_packet = True
                    receiver.reception_complete = False
            else:
                self.probe_manager.watcher()

    def correlate(self, receiver1, receiver2):
        correlation = np.absolute(np.correlate(receiver1.samples, receiver2.samples, "full", False)).tolist()
        delay = int(correlation.index(np.max(correlation)) - self.samples_to_receive + 1)
        print("Delay:", delay, "samples")
        print("Correlation length:", len(correlation))
        return correlation, delay

#    def send_results:
        # for gui in self.guis
        # data serialize 

class gui_interface():
    def __init__(self, rpc_address, hostname):
        self.rpc_address = rpc_address
        self.hostname = hostname
        self.rpc_manager = rpc_manager_local.rpc_manager()
        self.rpc_manager.set_request_socket(rpc_address)


###############################################################################
# Options Parser
###############################################################################
def parse_options():
    """ Options parser. """
    parser = OptionParser(option_class=eng_option, usage="%prog: [options]")
    parser.add_option("-s", "--servername", type="string", default="localhost",
                      help="Server hostname")
    parser.add_option("-c", "--clientname", type="string", default="localhost",
                      help="Server hostname")
    parser.add_option("", "--num-samples", type="string", default="5000",
                      help="Number of samples in burst")
    parser.add_option("", "--antenna", type="string", default="RX2",
                      help="Antenna to use")
    parser.add_option("", "--gain", type="float", default="30",
                      help="Gain in dB")
    parser.add_option("", "--frequency", type="string", default="2.48e9",
                      help="Frequency")
    parser.add_option("", "--samp-rate", type="string", default="50e6",
                      help="Sampling rate")
    parser.add_option("", "--lo-offset", type="string", default="0",
                      help="LO offset")
    parser.add_option("", "--bandwidth", type="string", default="1e6",
                      help="Bandwidth")
    (options, args) = parser.parse_args()
    return options

###############################################################################
# Main
###############################################################################
if __name__ == "__main__":
    options = parse_options()
    fc = fusion_center(options)
    fc.process_results()

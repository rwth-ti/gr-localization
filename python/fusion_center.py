#!/usr/bin/env python

###############################################################################
# Imports
###############################################################################
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

        self.correlation_receivers = ["",""]

        # ZeroMQ
        self.probe_manager = zeromq.probe_manager()
        self.rpc_manager = rpc_manager_local.rpc_manager()
        self.rpc_manager.set_reply_socket(rpc_adr)
        self.rpc_manager.add_interface("register_gui",self.register_gui)
        self.rpc_manager.add_interface("register_receiver",self.register_receiver)
        self.rpc_manager.add_interface("forward_chat",self.forward_chat)
        self.rpc_manager.add_interface("start_receivers",self.start_receivers)
        self.rpc_manager.add_interface("reset_receivers",self.reset_receivers)
        self.rpc_manager.add_interface("update_receivers",self.update_receivers)
        self.rpc_manager.add_interface("start_correlation",self.start_correlation)
        self.rpc_manager.add_interface("set_frequency",self.set_frequency)
        self.rpc_manager.add_interface("set_lo_offset",self.set_lo_offset)
        self.rpc_manager.add_interface("set_samples_to_receive",self.set_samples_to_receive)
        self.rpc_manager.add_interface("set_gain",self.set_gain)
        self.rpc_manager.add_interface("set_samp_rate",self.set_samp_rate)
        self.rpc_manager.add_interface("set_bw",self.set_bw)
        self.rpc_manager.add_interface("set_antenna",self.set_antenna)
        self.rpc_manager.start_watcher()

    def forward_chat(self, chat):
        for gui in self.guis.values():
            self.rpc_manager.set_request_socket(gui)
            self.rpc_manager.request("new_chat",[chat])

    def register_gui(self, hostname, id_gui, first):
        was_not_registered = False
        gui = "tcp://" + hostname + ":" + str(7775 + id_gui)
        gui_serial = hostname + str(id_gui)
        if not self.guis.has_key(gui_serial):
            self.guis[gui_serial] = gui
            was_not_registered = True
        if first or was_not_registered:
            self.rpc_manager.set_request_socket(gui)
            for serial in self.receivers:
                # request registration of each receiver in gui
                self.rpc_manager.request("register_receiver",[serial, self.receivers[serial].gain, self.receivers[serial].antenna])
            self.rpc_manager.request("set_gui_frequency",[self.frequency])
            self.rpc_manager.request("set_gui_lo_offset",[self.lo_offset])
            self.rpc_manager.request("set_gui_samples_to_receive",[self.samples_to_receive])
            self.rpc_manager.request("set_gui_bw",[self.bw])
            self.rpc_manager.request("set_gui_samp_rate",[self.samp_rate])
            for gui in self.guis.values():
                self.rpc_manager.set_request_socket(gui)
                for serial in self.guis:
                    # request registration in each gui
                    self.rpc_manager.request("register_another_gui",[serial])
            print gui_serial, "registered"

    def register_receiver(self, hostname, serial, id_rx, first):
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
            self.probe_manager.add_socket(receiver.probe_address, 'complex64', receiver.receive_samples)
            for gui in self.guis.values():
                # request registration in each gui
                self.rpc_manager.set_request_socket(gui)
                self.rpc_manager.request("register_receiver",[serial, receiver.gain, receiver.antenna])
            self.update_receivers()
            print serial, "registered"
            #threading.Thread(target = self.finish_register(serial)).start()
            #self.reset_receivers()

    def finish_register(self, serial):
        time.sleep(5)
        self.reset_receivers()
        print serial, "registered"

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
        self.start_receivers(freq, lo_offset, samples_to_receive)
        self.correlation_receivers[0] = receiver1
        self.correlation_receivers[1] = receiver2

    def set_frequency(self, frequency):
        self.frequency = frequency
        for receiver in self.receivers.values():
            receiver.frequency = frequency
        for gui in self.guis.values():
            self.rpc_manager.set_request_socket(gui)
            self.rpc_manager.request("set_gui_frequency",[frequency])

    def set_lo_offset(self, lo_offset):
        self.lo_offset = lo_offset
        for receiver in self.receivers.values():
            receiver.lo_offset = lo_offset
        for gui in self.guis.values():
            self.rpc_manager.set_request_socket(gui)
            self.rpc_manager.request("set_gui_lo_offset",[lo_offset])

    def set_samples_to_receive(self, samples_to_receive):
        self.samples_to_receive = samples_to_receive
        for receiver in self.receivers.values():
            receiver.samples_to_receive = samples_to_receive
        for gui in self.guis.values():
            self.rpc_manager.set_request_socket(gui)
            self.rpc_manager.request("set_gui_samples_to_receive",[samples_to_receive])

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        for receiver in self.receivers.values():
            receiver.samp_rate = samp_rate
        for gui in self.guis.values():
            self.rpc_manager.set_request_socket(gui)
            self.rpc_manager.request("set_gui_samp_rate",[samp_rate])

    def set_bw(self, bw):
        self.bw = bw
        for receiver in self.receivers.values():
            receiver.bw = bw
        for gui in self.guis.values():
            self.rpc_manager.set_request_socket(gui)
            self.rpc_manager.request("set_gui_bw",[bw])

    def set_gain(self, gain, serial):
        self.receivers[serial].gain = gain
        for gui in self.guis.values():
            self.rpc_manager.set_request_socket(gui)
            self.rpc_manager.request("set_gui_gain",[gain, serial])

    def set_antenna(self, antenna, serial):
        self.receivers[serial].antenna = antenna
        for gui in self.guis.values():
            self.rpc_manager.set_request_socket(gui)
            self.rpc_manager.request("set_gui_antenna",[antenna, serial])

    def process_results(self):
        while True:
            time.sleep(0.01)
            if all(self.receivers[key].reception_complete for key in self.receivers) and len(self.receivers.items()) > 0:
                receiver1 = self.receivers[self.correlation_receivers[0]]
                receiver2 = self.receivers[self.correlation_receivers[1]]
                correlation, delay = self.correlate(receiver1,receiver2)
                results = {"receiver1":receiver1.samples,"receiver2":receiver2.samples,"correlation":correlation,"delay":delay}
                for gui in self.guis.values():
                    self.rpc_manager.set_request_socket(gui)
                    self.rpc_manager.request("get_results",[results])
                for receiver in self.receivers.values():
                    receiver.first_packet = True
                    receiver.reception_complete = False
            else:
                self.probe_manager.watcher()

    def correlate(self, receiver1, receiver2):
        correlation = np.absolute(np.correlate(receiver1.samples, receiver2.samples, "full", False)).tolist()
        delay = int(correlation.index(np.max(correlation)) - self.samples_to_receive + 1)
        print "Delay:", delay, "samples"
        print "Correlation length:", len(correlation)
        return correlation, delay

#    def send_results:
        # for gui in self.guis
        # data serialize 

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
    parser.add_option("", "--samp-rate", type="string", default="10e6",
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

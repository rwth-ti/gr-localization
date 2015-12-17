# Imports
from gnuradio import zeromq
import numpy as np
import time
import rpc_manager as rpc_manager_local
from copy import deepcopy

class receiver_interface():

    def __init__(self, rpc_address, probe_address):
        if not rpc_address == None:
            self.rpc_address = rpc_address
            self.probe_address = probe_address
            self.rpc_mgr = rpc_manager_local.rpc_manager()
            self.rpc_mgr.set_request_socket(rpc_address)

        self.samples_to_receive = 0
        self.frequency = 0
        self.lo_offset = 0
        self.first_packet = True
        self.reception_complete = False
        self.samples = []
        self.bw = 0

        self.selected_position = "manual"
        self.gps = ""
        self.coordinates = [0.0, 0.0]
        self.coordinates_gps = [0.0, 0.0]

    def __deepcopy__(self, memo):
        cls = self.__class__
        result = self.__class__(None,None)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            if not (k == "rpc_mgr"):
                setattr(result, k, deepcopy(v, memo))
        return result

    def set_gain(self, gain):
        self.gain = gain
        self.rpc_mgr.request("set_gain",[self.gain])
    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.rpc_mgr.request("set_samp_rate",[self.samp_rate])
    def set_bw(self, bw):
        self.bw = bw
        self.rpc_mgr.request("set_bw",[self.bw])
    def set_antenna(self, antenna):
        self.antenna = antenna
        self.rpc_mgr.request("set_antenna",[self.antenna])

    def request_samples(self, ntp_sync, time_to_receive):
        self.samples = []
        self.first_packet = True
        self.reception_complete = False
        self.rpc_mgr.request("start_fg",[self.samples_to_receive, self.frequency, self.lo_offset, time_to_receive])

    def receive_samples(self, samples):
        if self.first_packet:
            self.samples = samples[100:]
            self.first_packet = False
        elif not self.reception_complete:
            self.samples = np.concatenate((self.samples, samples), axis=1)
            #print "reconstruction"
        if self.samples_to_receive == len(self.samples):
            self.reception_complete = True
    def get_gps_position(self):
        return self.rpc_mgr.request("get_gps_position")

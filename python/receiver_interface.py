# Imports
from gnuradio import zeromq
import numpy as np
import time

class receiver_interface():

    def __init__(self, rpc_address, probe_address):
        self.rpc_address = rpc_address
        self.probe_address = probe_address
        self.rpc_mgr = zeromq.rpc_manager()
        self.rpc_mgr.set_request_socket(rpc_address)

        self.samples_to_receive = 0
        self.first_packet = True
        self.reception_complete = False
        self.samples = []

    def request_samples(self):
        self.samples = []
        self.first_packet = True
        self.reception_complete = False
        self.rpc_mgr.request("start_fg",[self.samples_to_receive])

    def receive_samples(self, samples):
        if self.first_packet:
            self.samples = samples[100:]
            self.first_packet = False
        else:
            self.samples = np.concatenate((self.samples, samples), axis=1)
            #print "reconstruction"
        if self.samples_to_receive == len(self.samples):
            self.reception_complete = True

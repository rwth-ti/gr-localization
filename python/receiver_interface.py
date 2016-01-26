# Imports
from gnuradio import zeromq
import numpy as np
import time
import rpc_manager as rpc_manager_local
from copy import deepcopy
#from scipy.signal import resample
from scipy import interpolate

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
        self.auto_calibrate = False
        self.samples_calibration = []
        self.samples = []
        self.bw = 0

        self.oversample_factor = 1

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

    def request_samples(self, time_to_receive, freq_calibration):
        self.samples = []
        self.first_packet = True
        self.reception_complete = False
        self.rpc_mgr.request("start_fg",[self.samples_to_receive, self.frequency, self.lo_offset, time_to_receive, freq_calibration])

    def receive_samples(self, samples):
        if self.first_packet:
            self.samples = samples[100:]
            self.first_packet = False
        elif len(self.samples) < self.samples_to_receive:
            self.samples = np.concatenate((self.samples, samples), axis=1)
            #print "reconstruction"
        elif len(self.samples) == self.samples_to_receive > len(self.samples_calibration) and self.auto_calibrate:
            if len(self.samples_calibration) == 0:
                self.samples_calibration = samples[100:]
            else:
                self.samples_calibration = np.concatenate((self.samples_calibration, samples), axis=1)
        if self.samples_to_receive == len(self.samples):
            if self.samples_to_receive == len(self.samples_calibration) and self.auto_calibrate:
                #x = np.arange(0,len(self.samples))
                x = np.linspace(0,len(self.samples),len(self.samples))
                f = interpolate.interp1d(x, self.samples)
                x_interpolated = np.linspace(0,len(self.samples),len(self.samples) * self.oversample_factor)
                #x_interpolated = np.arange(0,len(self.samples) - 1, 1. / self.oversample_factor)
                self.samples = f(x_interpolated)
                f = interpolate.interp1d(x, self.samples_calibration)
                self.samples_calibration = f(x_interpolated)
                #self.samples = resample(self.samples, self.oversample_factor * len(self.samples))
                #self.samples_calibration = resample(self.samples_calibration, self.oversample_factor * len(self.samples_calibration))
                self.reception_complete = True
            elif not self.auto_calibrate:
                #self.samples = resample(self.samples, self.oversample_factor * len(self.samples))
                x = np.linspace(0,len(self.samples),len(self.samples))
                #x = np.arange(0,len(self.samples))
                f = interpolate.interp1d(x, self.samples)
                x_interpolated = np.linspace(0,len(self.samples),len(self.samples) * self.oversample_factor)
                #x_interpolated = np.arange(0,len(self.samples) - 1, 1. / self.oversample_factor)
                self.samples = f(x_interpolated)
                self.reception_complete = True

    def get_gps_position(self):
        return self.rpc_mgr.request("get_gps_position")

# Imports
from gnuradio import zeromq
import numpy as np
import time
import rpc_manager as rpc_manager_local
from copy import deepcopy
#from scipy.signal import resample
from scipy import interpolate

class receiver_interface():

    def __init__(self, rpc_address, probe_address, serial):
        if not rpc_address == None:
            self.rpc_address = rpc_address
            self.probe_address = probe_address
            self.rpc_mgr = rpc_manager_local.rpc_manager()
            self.rpc_mgr.set_request_socket(rpc_address)
            self.serial = serial

        self.error_detected = False
        self.first_packet = True
        self.reception_complete = False
        self.auto_calibrate = False
        self.samples_calibration = []
        self.samples = []
        self.tags = None
        self.tags_calibration = None
        self.samples_to_receive = 0
        self.frequency = 0
        self.lo_offset = 0
        self.bw = 0
        self.interpolation = 0
        self.samp_rate = 0
        self.samples_to_receive_calibration = 0
        self.frequency_calibration = 0
        self.lo_offset_calibration = 0
        self.bw_calibration = 0
        self.acquisition_state = "target"

        self.selected_position = "manual"
        self.gps = ""
        self.coordinates = [0.0, 0.0]
        self.coordinates_gps = [0.0, 0.0]

    def __deepcopy__(self, memo):
        cls = self.__class__
        result = self.__class__(None,None,None)
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
    def set_interpolation(self, interpolation):
        self.interpolation = interpolation
        self.rpc_mgr.request("set_interpolation",[self.interpolation])
    def set_antenna(self, antenna):
        self.antenna = antenna
        self.rpc_mgr.request("set_antenna",[self.antenna])
    def set_run_loop(self, run_loop):
        self.rpc_mgr.request("set_run_loop",[run_loop])
    def sync_time(self):
        self.rpc_mgr.request("sync_time")

    def request_samples(self, time_to_receive, acquisitions, acquisition_time):
        self.samples = []
        self.samples_calibration = []
        self.first_packet = True
        self.reception_complete = False
        self.rpc_mgr.request("start_fg",[self.samples_to_receive, self.frequency, self.lo_offset, self.bw, self.gain, self.samples_to_receive_calibration, self.frequency_calibration, self.lo_offset_calibration, self.bw_calibration, self.gain_calibration, time_to_receive, self.auto_calibrate, acquisitions, acquisition_time])

    def reset_receiver(self):
        print "Reset receiver: ", self.serial
        self.samples = []
        self.samples_calibration = []
        self.reception_complete = False
        self.first_packet = True
        self.error_detected = False
        self.acquisition_state = "target"

    def receive_samples(self, samples, tags):
        if tags is not None:
            print self.serial, tags, "num_samples", len(samples)
        else:
            print self.serial, "tags == None", len(samples)
        if self.frequency == self.frequency_calibration:
            print "Warning: calibration frequency should not be equal to target frequency"

        # state machine to toggle between target and calibration samples
        if self.acquisition_state == "target":
            print "target"
            if tags is not None:
                if tags["rx_freq"] == self.frequency:
                    self.samples = samples[300:]
                    self.tags = tags
                    self.first_packet = False
                    if len(self.samples) == self.samples_to_receive and self.auto_calibrate:
                        self.acquisition_state = "calibration"
                else:
                    #print "in receiver:" 
                    #print "tag frequency:"+str(tags["rx_freq"])+"\nreceiver frequency:"+str(self.frequency)
                    self.error_detected = True
                    print "Error: USRP-source tag shows unexpected target carrier frequency"
            elif tags is None:
                if len(self.samples) < self.samples_to_receive:
                    self.samples = np.concatenate((self.samples, samples), axis=1)
                    if len(self.samples) == self.samples_to_receive and self.auto_calibrate:
                        self.acquisition_state = "calibration"
                else:
                    self.error_detected = True
                    print "Error: more target samples received than expected"

        elif self.acquisition_state == "calibration":
            print "calibration"
            if tags is not None:
                if tags["rx_freq"] == self.frequency_calibration:
                    self.samples_calibration = samples[300:]
                    self.tags_calibration = tags
                    if len(self.samples_calibration) == self.samples_to_receive_calibration:
                        self.acquisition_state = "target"
                else:
                    #print "in receiver:"
                    #print "tag frequency:"+str(tags["rx_freq"])+"\nreceiver frequency:"+str(self.frequency)
                    self.acquisition_state = "target"
                    self.error_detected = True
                    print "Error: USRP-source tag shows unexpected calibration carrier frequency"
            elif tags is None:
                if len(self.samples_calibration) < self.samples_to_receive_calibration:
                    self.samples_calibration = np.concatenate((self.samples_calibration, samples), axis=1)
                    if len(self.samples_calibration) == self.samples_to_receive_calibration:
                        self.acquisition_state = "target"
                else:
                    self.error_detected = True
                    print "Error: more calibration samples received than expected"

        if self.samples_to_receive == len(self.samples):
            if self.samples_to_receive_calibration == len(self.samples_calibration) and self.auto_calibrate:
                #x = np.arange(0,len(self.samples))
                x = np.linspace(0,len(self.samples),len(self.samples))
                f = interpolate.interp1d(x, self.samples)
                x_interpolated = np.linspace(0,len(self.samples),len(self.samples) * self.interpolation)
                #x_interpolated = np.arange(0,len(self.samples) - 1, 1. / self.interpolation)
                self.samples = f(x_interpolated)
                x = np.linspace(0,len(self.samples_calibration),len(self.samples_calibration))
                f = interpolate.interp1d(x, self.samples_calibration)
                x_interpolated = np.linspace(0,len(self.samples_calibration),len(self.samples_calibration) * self.interpolation)
                self.samples_calibration = f(x_interpolated)
                #self.samples = resample(self.samples, self.interpolation * len(self.samples))
                #self.samples_calibration = resample(self.samples_calibration, self.interpolation * len(self.samples_calibration))
                self.reception_complete = True
            elif not self.auto_calibrate:
                #self.samples = resample(self.samples, self.interpolation * len(self.samples))
                x = np.linspace(0,len(self.samples),len(self.samples))
                #x = np.arange(0,len(self.samples))
                f = interpolate.interp1d(x, self.samples)
                x_interpolated = np.linspace(0,len(self.samples),len(self.samples) * self.interpolation)
                #x_interpolated = np.arange(0,len(self.samples) - 1, 1. / self.interpolation)
                self.samples = f(x_interpolated)
                self.reception_complete = True

    def get_gps_position(self):
        return self.rpc_mgr.request("get_gps_position")

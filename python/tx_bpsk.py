# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Tx Bpsk
# Generated: Fri Feb  3 15:39:39 2017
##################################################

from gnuradio import blocks
from gnuradio import gr
from gnuradio import uhd
from gnuradio.filter import firdes
import numpy as np 
import numpy.matlib
import time


class tx_bpsk(gr.hier_block2):

    def __init__(self, serial = "", gps = "internal", mcr = 0, bandwidth=50000000, center_freq=2510000000, gain=89.5, num_pulses=20000, pulse_length=1, samp_rate=10000000):
        gr.hier_block2.__init__(
            self, "Tx Bpsk",
            gr.io_signature(0, 0, 0),
            gr.io_signature(0, 0, 0),
        )

        ##################################################
        # Parameters
        ##################################################
        self.serial = serial
        self.mcr = mcr
        self.gps = gps
        self.bandwidth = bandwidth
        self.center_freq = center_freq
        self.num_pulses = num_pulses
        self.pulse_length = pulse_length
        self.samp_rate = samp_rate

        ##################################################
        # Blocks
        ##################################################

        if  serial != "":
            if  mcr != 0:
                 self.uhd_usrp_sink_0 = uhd.usrp_sink(
                    "serial == " +  serial
                    + ",master_clock_rate == " + str( mcr),
                    uhd.stream_args(
                        cpu_format="fc32",
                        channels=range(1),
                     ),
                )
            else:
                 self.uhd_usrp_sink_0 = uhd.usrp_sink(
                    "serial == " +  serial,
                    uhd.stream_args(
                        cpu_format="fc32",
                        channels=range(1),
                     ),
                )

        else:
            if  mcr != 0:
                 self.uhd_usrp_sink_0 = uhd.usrp_sink(
                    "master_clock_rate == " + str( mcr),
                    uhd.stream_args(
                        cpu_format="fc32",
                        channels=range(1),
                    ),
                )
            else:
                 self.uhd_usrp_sink_0 = uhd.usrp_sink(
                    "",
                    uhd.stream_args(
                        cpu_format="fc32",
                        channels=range(1),
                    ),
                )

        if self.gps != "internal":
             print "Using " + self.gps
             self.uhd_usrp_sink_0.set_clock_source("external", 0)
             self.uhd_usrp_sink_0.set_time_source("external", 0)
        if self.gps == "ltelite":
            self.ser = serial.Serial("/dev/ttyUSB0", 38400, timeout = 1)
            self.nmea_external = ""
            self.nmea_external_lock = threading.Lock()
            threading.Thread(target = self.poll_lte_lite).start()
        elif self.gps == "leam8f":
            self.ser = serial.Serial("/dev/ttyACM0", 38400, timeout = 1)
            self.nmea_external = ""
            self.nmea_external_lock = threading.Lock()
            threading.Thread(target = self.poll_lea_m8f).start()

        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_center_freq(center_freq, 0)
        self.uhd_usrp_sink_0.set_gain(gain, 0)
        self.uhd_usrp_sink_0.set_antenna('TX/RX', 0)
        self.blocks_vector_source_x_0 = blocks.vector_source_c(np.reshape(np.matlib.repmat(np.random.randint(0,2,num_pulses)*2-1,pulse_length,1).T,[1,num_pulses*pulse_length])[0].tolist(), True, 1, [])

        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_vector_source_x_0, 0), (self.uhd_usrp_sink_0, 0))    

    def get_bandwidth(self):
        return self.bandwidth

    def set_bandwidth(self, bandwidth):
        self.bandwidth = bandwidth
        self.uhd_usrp_sink_0.set_bandwidth(self.bandwidth, 0)

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self.uhd_usrp_sink_0.set_center_freq(self.center_freq, 0)


    def set_gain(self, gain):
        print "set tx gain to: ",gain
        #self.uhd_usrp_sink_0.set_gain(gain, 0)
        print "gain set to: ", self.uhd_usrp_sink_0.get_gain()
        
        	

    def get_num_pulses(self):
        return self.num_pulses

    def set_num_pulses(self, num_pulses):
        self.num_pulses = num_pulses
        self.blocks_vector_source_x_0.set_data(np.reshape(np.matlib.repmat(np.random.randint(0,2,self.num_pulses)*2-1,self.pulse_length,1).T,[1,self.num_pulses*self.pulse_length])[0].tolist(), [])

    def get_pulse_length(self):
        return self.pulse_length

    def set_pulse_length(self, pulse_length):
        self.pulse_length = pulse_length
        self.blocks_vector_source_x_0.set_data(np.reshape(np.matlib.repmat(np.random.randint(0,2,self.num_pulses)*2-1,self.pulse_length,1).T,[1,self.num_pulses*self.pulse_length])[0].tolist(), [])

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)

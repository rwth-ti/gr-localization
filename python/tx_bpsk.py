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

    def __init__(self, bandwidth=50000000, center_freq=2510000000, gain=50, num_pulses=20000, pulse_length=1, samp_rate=10000000):
        gr.hier_block2.__init__(
            self, "Tx Bpsk",
            gr.io_signature(0, 0, 0),
            gr.io_signature(0, 0, 0),
        )

        ##################################################
        # Parameters
        ##################################################
        self.bandwidth = bandwidth
        self.center_freq = center_freq
        self.gain = gain
        self.num_pulses = num_pulses
        self.pulse_length = pulse_length
        self.samp_rate = samp_rate

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
        	",".join(("", "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_sink_0.set_clock_rate(30.72e6, uhd.ALL_MBOARDS)
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_center_freq(center_freq, 0)
        self.uhd_usrp_sink_0.set_gain(gain, 0)
        self.uhd_usrp_sink_0.set_antenna('TX/RX', 0)
        self.uhd_usrp_sink_0.set_bandwidth(bandwidth, 0)
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

    def get_gain(self):
        return self.gain

    def set_gain(self, gain):
        self.gain = gain
        self.uhd_usrp_sink_0.set_gain(self.gain, 0)
        	

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

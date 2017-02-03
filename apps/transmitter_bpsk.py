#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Top Block
# Generated: Fri Feb  3 16:28:01 2017
##################################################

import os
import sys
sys.path.append(os.environ.get('GRC_HIER_PATH', os.path.expanduser('~/.grc_gnuradio')))

from gnuradio import eng_notation
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser

import numpy as np 
import numpy.matlib
sys.path.append("../python")
from tx_bpsk import tx_bpsk  # grc-generated hier_block


class top_block(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "Top Block")

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 20000000
        self.pulse_length = pulse_length = 1
        self.num_pulses = num_pulses = 20000

        ##################################################
        # Blocks
        ##################################################
        self.tx_bpsk_0 = tx_bpsk(
            bandwidth=10000000,
            center_freq=2510000000,
            gain=50,
            num_pulses=20000,
            pulse_length=1,
            samp_rate=10000000,
        )
        self.connect(tx_bpsk)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate

    def get_pulse_length(self):
        return self.pulse_length

    def set_pulse_length(self, pulse_length):
        self.pulse_length = pulse_length

    def get_num_pulses(self):
        return self.num_pulses

    def set_num_pulses(self, num_pulses):
        self.num_pulses = num_pulses


def main(top_block_cls=top_block, options=None):

    tb = top_block_cls()
    tb.start()
    try:
        raw_input('Press Enter to quit: ')
    except EOFError:
        pass
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()

#!/usr/bin/env python

###############################################################################
# Imports
###############################################################################
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from grc_gnuradio import wxgui as grc_wxgui
from optparse import OptionParser
import sys
import time
import wx
import numpy as np
sys.path.append("../python")
import mls
import numpy.matlib as nm


###############################################################################
# GNU Radio top_block
###############################################################################
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from grc_gnuradio import wxgui as grc_wxgui
from optparse import OptionParser
import time
import wx
import numpy as np

class top_block(gr.top_block):
    def __init__(self, options):
        gr.top_block.__init__(self)
        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 10000000
        self.f = f = 2510000000

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
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_center_freq(2510000000, 0)
        self.uhd_usrp_sink_0.set_gain(90, 0)
        self.blocks_float_to_complex_0 = blocks.float_to_complex(1)

        mcode = True
        nbits = 11
        if mcode == True:
            v = []
            for i in mls.mls(nbits): v.append(int(i))
            v = np.array(v)
        else:
            v = np.random.randint(0,2,np.power(nbits,2))
        z = np.zeros(np.power(nbits,2))
        v = v*2-1

        #v = nm.repmat(v,10,1).flatten("F")
        #z = nm.repmat(z,1,,1).flatten("F")

        self.blocks_vector_source_x_0 = blocks.vector_source_f((v), True, 1, [])
        self.blocks_vector_source_x_1 = blocks.vector_source_f((z), True, 1, [])

        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_vector_source_x_0, 0), (self.blocks_float_to_complex_0, 0))    
        self.connect((self.blocks_vector_source_x_1, 0), (self.blocks_float_to_complex_0, 1))    
        self.connect((self.blocks_float_to_complex_0, 0), (self.uhd_usrp_sink_0, 0))    


    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)

    def get_f(self):
        return self.f

    def set_f(self, f):
        self.f = f


###############################################################################
# Options Parser
###############################################################################
def parse_options():
    """ Options parser. """
    parser = OptionParser(option_class=eng_option, usage="%prog: [options]")
    parser.add_option("-s", "--serial", type="string", default="",
                      help="USRP serial number")
    parser.add_option("", "--fusion-center", type="string", default="localhost",
                      help="Fusion center address")
    parser.add_option("-g", "--gps", type="string", default="lc_xo",
                      help="GPS type")
    parser.add_option("-i", "--id-rx", type="int", default="1",
                      help="Receiver ID")
    parser.add_option("-d", "--delay", type="int", default="0",
                      help="Delay")
    parser.add_option("", "--snr", type="float", default="20",
                      help="SNR")
    parser.add_option("-m", "--modulation", type="string", default="bpsk",
                      help="Modulation type (BPSK/OFDM)")
    parser.add_option("", "--dot-graph", action="store_true", default=False,
                      help="Generate dot-graph file from flowgraph")
    (options, args) = parser.parse_args()
    return options

###############################################################################
# Main
###############################################################################
if __name__ == "__main__":
    options = parse_options()
    tb = top_block(options)

    if options.dot_graph:
        # write a dot graph of the flowgraph to file
        dot_str = tb.dot_graph()
        file_str = os.path.expanduser('flowgraph.dot')
        dot_file = open(file_str,'w')
        dot_file.write(dot_str)
        dot_file.close()

    try:
        tb.start()

        # keep the program running when flowgraph is stopped
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    print "Shutting down flowgraph."
    tb.stop()
    tb.wait()
    tb = None



#!/usr/bin/env python

###############################################################################
# Imports
###############################################################################
from gnuradio import zeromq
from gnuradio import gr
from gnuradio import blocks
from gnuradio import analog
from gnuradio import eng_notation
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from optparse import OptionParser
import numpy
import sys
import os
import threading
import time
import rpc_manager as rpc_manager_local
import socket
from grc_gnuradio import blks2 as grc_blks2
from gnuradio import digital


###############################################################################
# GNU Radio top_block
###############################################################################
class top_block(gr.top_block):
    def __init__(self, options):
        gr.top_block.__init__(self)

        self.options = options

        self.samp_rate = 50000000
        self.hostname = os.uname()[1]
        self.gps = "dummy"


        # socket addresses
        rpc_port = 6665 + options.id_rx
        rpc_adr = "tcp://*:" + str(rpc_port)
        fusion_center_adr = "tcp://" + options.fusion_center + ":6665"
        probe_port = 5555 + options.id_rx
        probe_adr = "tcp://*:" + str(probe_port)

        # blocks
        self.zmq_probe = zeromq.pub_sink(gr.sizeof_gr_complex, 1, probe_adr)
        self.vector_source = blocks.vector_source_f(([0,0,0,1,1,0,1,0,1,1,1]), True, 1, [])
        self.throttle = blocks.throttle(gr.sizeof_gr_complex*1, self.samp_rate,True)
        self.ofdm_mod = grc_blks2.packet_mod_f(digital.ofdm_mod(
        		options=grc_blks2.options(
        			modulation="qpsk",
        			fft_length=4096,
        			occupied_tones=200,
        			cp_length=0,
        			pad_for_usrp=False,
        			log=None,
        			verbose=None,
        		),
        	),
        	payload_length=0,
        )
        self.head = blocks.head(gr.sizeof_gr_complex*1, 5100)
        self.skiphead= blocks.skiphead(gr.sizeof_gr_complex*1,options.delay)


        # connects
        #self.connect(self.usrp_source, self.s_to_v, self.zmq_probe)
        self.connect((self.vector_source, 0), (self.ofdm_mod, 0))
        self.connect((self.ofdm_mod, 0), (self.throttle, 0))
        self.connect(self.throttle, self.skiphead)
        self.connect(self.skiphead, self.head)
        self.connect(self.head, self.zmq_probe)

        # ZeroMQ
        self.rpc_manager = rpc_manager_local.rpc_manager()
        self.rpc_manager.set_reply_socket(rpc_adr)
        self.rpc_manager.set_request_socket(fusion_center_adr)
        self.rpc_manager.add_interface("start_fg",self.start_fg)
        self.rpc_manager.add_interface("set_gain",self.set_gain)
        self.rpc_manager.add_interface("set_samp_rate",self.set_samp_rate)
        self.rpc_manager.add_interface("set_bw",self.set_bw)
        self.rpc_manager.add_interface("set_antenna",self.set_antenna)
        self.rpc_manager.add_interface("get_gps_position",self.get_gps_position)
        self.rpc_manager.start_watcher()


        # Find out ip address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if not options.fusion_center == "localhost":
            s.connect((options.fusion_center,22))
        else:
            s.connect(("www.rwth-aachen.de",80))
        self.ip_addr = s.getsockname()[0]

    def set_samp_rate(self,samp_rate):
        self.samp_rate = samp_rate
    def set_bw(self,bw):
        return
    def set_gain(self,gain):
        return
    def set_antenna(self,antenna):
        return

    def register_receiver(self):
        first = True
        while(True):
            # register receiver [hostname, usrp_serial, rx_id]
            self.rpc_manager.request("register_receiver",[self.ip_addr, "dummy" + self.hostname + str(options.id_rx), self.options.id_rx, self.gps, first])
            first = False
            time.sleep(10)

    def start_fg(self, samples_to_receive, freq, lo_offset):
        self.stop()
        self.wait()

        self.disconnect((self.vector_source, 0), (self.ofdm_mod, 0))
        self.disconnect((self.ofdm_mod, 0), (self.throttle, 0))
        self.disconnect(self.throttle, self.skiphead)
        self.disconnect(self.skiphead, self.head)
        self.disconnect(self.head, self.zmq_probe)

        self.vector_source = blocks.vector_source_f(([0,0,0,1,1,0,1,0,1,1,1]), True, 1, [])
        self.throttle = blocks.throttle(gr.sizeof_gr_complex*1, self.samp_rate,True)
        self.ofdm_mod = grc_blks2.packet_mod_f(digital.ofdm_mod(
        		options=grc_blks2.options(
        			modulation="qpsk",
        			fft_length=8192,
        			occupied_tones=200,
        			cp_length=0,
        			pad_for_usrp=False,
        			log=None,
        			verbose=None,
        		),
        	),
        	payload_length=0,
        )
        self.head = blocks.head(gr.sizeof_gr_complex*1, 5100)
        self.skiphead= blocks.skiphead(gr.sizeof_gr_complex*1,self.options.delay)

        self.connect((self.vector_source, 0), (self.ofdm_mod, 0))
        self.connect((self.ofdm_mod, 0), (self.throttle, 0))
        self.connect(self.throttle, self.skiphead)
        self.connect(self.skiphead, self.head)
        self.connect(self.head, self.zmq_probe)

        self.start()

    def get_gps_position(self):
        longitude = 6.062
        latitude = 50.7795
        return [longitude, latitude]

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

        tb.timer_register = threading.Thread(target = tb.register_receiver)
        tb.timer_register.daemon = True
        tb.timer_register.start()
        # keep the program running when flowgraph is stopped
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    print "Shutting down flowgraph."
    tb.rpc_manager.stop_watcher()
    tb.stop()
    tb.wait()
    tb = None

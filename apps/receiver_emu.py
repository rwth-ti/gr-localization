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
from numpy import genfromtxt
import numpy as np
import numpy.matlib
import sys
import os
import threading
import time
import socket
from grc_gnuradio import blks2 as grc_blks2
#from gnuradio import digital
sys.path.append("../python")
import rpc_manager as rpc_manager_local
import pmt
from gpsconfig import *



###############################################################################
# GNU Radio top_block
###############################################################################
class top_block(gr.top_block):
    def __init__(self, options):
        gr.top_block.__init__(self)
        self.c = 299700000
        self.options = options
        self.cnt=0
        self.run_loop = False
        self.samp_rate = 50000000
        self.hostname = os.uname()[1]
        self.gps = "emu_"
        self.id_rx = options.id_rx
        self.noise_amp = 1/np.sqrt(np.power(10,options.snr/10))
        self.modulation = options.modulation
        self.seed = 10
        self.delay = options.delay
        self.samples_to_receive = 1000
        self.samples_to_receive_calibration = 1000
        self.freq = 550000000
        coordinates_string = options.coordinates_m.split(",")
        self.coordinates = (float(coordinates_string[0]),float(coordinates_string[1]))
        tx_coordinates_string = options.tx_coordinates.split(",")
        self.tx_coordinates = np.array([float(tx_coordinates_string[0]),float(tx_coordinates_string[1])])
        # socket addresses
        rpc_port = 6665 + options.id_rx
        rpc_adr = "tcp://*:" + str(rpc_port)
        fusion_center_adr = "tcp://" + options.fusion_center + ":6665"
        probe_port = 5555 + options.id_rx
        probe_adr = "tcp://*:" + str(probe_port)
        self.stp_cnt = 0
        if options.movement_file != "EMPTY":
            self.track_coordinates = genfromtxt(options.movement_file, delimiter=',')
            print self.track_coordinates
            self.tx_coordinates = self.track_coordinates[0,:]
            self.track_coordinates = np.delete(self.track_coordinates,0,0)
        else:
            self.track_coordinates = np.array([])
        # blocks
        self.zmq_probe = zeromq.pub_sink(gr.sizeof_gr_complex, 1, probe_adr, 300, True)
        self.mod_block = ModulatorBlock(self.seed, self.samp_rate, self.noise_amp, self.modulation, self.delay, self.samples_to_receive, self.freq)
        self.seed += 1

        # connects
        self.connect(self.mod_block, self.zmq_probe)

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
        self.rpc_manager.add_interface("set_run_loop",self.set_run_loop)
        self.rpc_manager.add_interface("sync_time",self.sync_time)
        self.rpc_manager.add_interface("program_gps_position",self.program_gps_position)
        self.rpc_manager.start_watcher()


        # Find out ip address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if not options.ssh_proxy:
            s.connect((options.fusion_center,6665))
        else:
            s.connect(("www.rwth-aachen.de",80))
        self.ip_addr = s.getsockname()[0]

    def set_run_loop(self, run_loop):
        self.run_loop = run_loop

    def set_samp_rate(self,samp_rate):
        print "samp_rate set to", samp_rate
        self.samp_rate = samp_rate
    def set_bw(self,bw):
        return
    def set_gain(self,gain):
        return
    def set_antenna(self,antenna):
        return
    def sync_time(self):
        print "Reset seed"
        self.seed = 10

    def register_receiver(self):
        first = True
        while(True):
            # register receiver [hostname, usrp_serial, rx_id]
            self.rpc_manager.request("register_receiver",[self.ip_addr, self.hostname + "_emu" +  str(options.id_rx), self.options.id_rx, self.gps, first, self.coordinates])
            first = False
            time.sleep(10)

    def start_fg(self, samples_to_receive, freq, lo_offset, bw, gain, samples_to_receive_calibration, freq_calibration, lo_offset_calibration, bw_calibration, gain_calibration, time_to_recv, auto_calibrate, acquisitions, acquisition_time):
        threading.Thread(target = self.start_reception, args = (samples_to_receive, freq, lo_offset, bw, gain, samples_to_receive_calibration, freq_calibration, lo_offset_calibration, bw_calibration, gain_calibration, time_to_recv, auto_calibrate, acquisitions, acquisition_time)).start()


    def start_reception(self, samples_to_receive, freq, lo_offset, bw, gain, samples_to_receive_calibration, freq_calibration, lo_offset_calibration, bw_calibration, gain_calibration, time_to_recv, auto_calibrate, acquisitions, acquisition_time):

        self.freq = freq
        self.freq_calibration = freq_calibration
        
        auto_delay = 5 # delay simulation of auto calibration

        if acquisitions == 0:
            infinity = True
        else:
            infinity = False

        times = 1
        if auto_calibrate: times = 2
        # change loop structure? (confusing)
        while True:
            for i in range(0,times):
                self.samples_to_receive = samples_to_receive
                self.samples_to_receive_calibration = samples_to_receive_calibration
                self.stop()
                self.wait()
                
                self.disconnect(self.mod_block, self.zmq_probe)

                # blocks
                if i == 1:
                    print "Sending " + str(samples_to_receive_calibration) + " samples for autocalibtration"
                    delay = auto_delay
                    self.mod_block = ModulatorBlock(self.seed, self.samp_rate, self.noise_amp, self.modulation, delay, self.samples_to_receive_calibration, self.freq_calibration)
                else:
                    print "Sending " + str(samples_to_receive) + " samples"
                    if self.delay == 0:
                        # calculate delay from transmitter position
                        delay = self.get_delay_from_location(self.tx_coordinates)
                        if len(self.track_coordinates):
                            # update target location for next acquisition
                            self.tx_coordinates = self.track_coordinates[0,:]
                            self.track_coordinates = np.delete(self.track_coordinates,0,0)
                            #print self.tx_coordinates
                            #print self.track_coordinates
                    else:
                        delay = self.delay
                    print delay
                    self.mod_block = ModulatorBlock(self.seed, self.samp_rate, self.noise_amp, self.modulation, delay, self.samples_to_receive, self.freq)
                    
                self.seed += 1

                # connects
                self.connect(self.mod_block, self.zmq_probe)

                self.start()
                time.sleep(0.2)

            acquisitions -= 1
            
            
            if not self.run_loop or (acquisitions <= 0 and not infinity):
                break
            time.sleep(acquisition_time)

    def get_gps_position(self):
        if self.options.coordinates_wgs84 != "":
            coordinates_wgs84_string = self.options.coordinates_wgs84.split(",")
            latitude = float(coordinates_wgs84_string[0])
            longitude = float(coordinates_wgs84_string[1]) 
        
        else:
            longitude = 6.062
            latitude = 50.7795
        # basemap requires [long,lat]; we want to put in [lat,long] => swap
        return [longitude, latitude]
    def get_delay_from_location(self,transmitter_coordinates):

        delay=int((np.linalg.norm(np.array(list(self.coordinates))-transmitter_coordinates)/self.c)*self.samp_rate)
        return delay

    def program_gps_position(self, latitude, longitude, altitude):
        # needed in ublox settings; by now we assume at least dm accuracy
        ground_truth_accuracy = 0.1
        set_ublox_coordinates_fixed(latitude, longitude, altitude, ground_truth_accuracy)
        #add checker if it worked
    
class ModulatorBlock(gr.hier_block2):
    def __init__(self, seed, samp_rate, noise_amp, modulation, delay, samples_to_receive, freq):
        gr.hier_block2.__init__(self, "ModulatorBlock",
                       gr.io_signature(0, 0, 0),
                       gr.io_signature(1, 1, gr.sizeof_gr_complex))

        # Timing tag: This is preserved and updated:
        timing_tag = gr.tag_t()
        timing_tag.offset = 0
        timing_tag.key = pmt.string_to_symbol('rx_time')
        timing_tag.value = pmt.to_pmt((float(seed), 0.6))
        timing_tag.srcid = pmt.string_to_symbol(str('gr uhd usrp source1'))
        # Rx freq tags:
        #print "In source emulation (before tag)" 
        #print freq
        rx_freq_tag = gr.tag_t()
        rx_freq_tag.offset = 0
        rx_freq_tag.key = pmt.string_to_symbol('rx_freq')
        rx_freq_tag.value = pmt.from_double(freq)
        rx_freq_tag.srcid = pmt.string_to_symbol(str('gr uhd usrp source1'))
        # Samp_rate tags:
        rx_rate_tag = gr.tag_t()
        rx_rate_tag.offset = 0
        rx_rate_tag.key = pmt.string_to_symbol('rx_rate')
        rx_rate_tag.value = pmt.from_double(samp_rate)
        rx_rate_tag.srcid = pmt.string_to_symbol(str('gr uhd usrp source1'))

        add = blocks.add_vcc(1, )

        tag_debug = blocks.tag_debug(gr.sizeof_gr_complex*1, "", "")
        tag_debug.set_display(True)
    
        #if modulation == "bpsk":
        #    mod = digital.psk.psk_mod(
        #      constellation_points=2,
        #      mod_code="none",
        #      differential=True,
        #      samples_per_symbol=2,
        #      excess_bw=0.1,
        #      verbose=False,
        #      log=False,
        #      )
        #else:
        #    mod = grc_blks2.packet_mod_b(digital.ofdm_mod(
        #                    options=grc_blks2.options(
        #                            modulation="qpsk",
        #                            fft_length=4096,
        #                            occupied_tones=200,
        #                            cp_length=0,
        #                            pad_for_usrp=False,
        #                            log=None,
        #                            verbose=None,
        #                    ),
        #            ),
        #            payload_length=0,
        #    )
        #print "in source emulation(after_tag)"
        #print  pmt.to_double(rx_freq_tag.value)
        pulse_width = 4
        np.random.seed(seed=seed)
        
        tx_vector = np.reshape(np.matlib.repmat(np.random.randint(0,2,(5*samples_to_receive)/pulse_width)*2-1,pulse_width,1).T,[1,5*samples_to_receive])[0].tolist()
        # delay signal vector -> insert zeros at beginnig; nothing happens if signal has not reached the receiver:
        tx_vector_delayed = np.hstack((np.zeros(delay),tx_vector))
        tx_vector_delayed = tx_vector_delayed[:600]
        vector_source = blocks.vector_source_c(tx_vector_delayed, False, tags=(timing_tag, rx_freq_tag, rx_rate_tag)) 
        #clip first 600 samples
        head = blocks.head(gr.sizeof_gr_complex*1, samples_to_receive + 300)
        print "Delay is " + str(delay)
        # skiphead= blocks.skiphead(gr.sizeof_gr_complex*1,delay)
        throttle = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate,True)
        noise = analog.noise_source_c(analog.GR_GAUSSIAN, noise_amp, -seed)

        # connects
        #self.connect(vector_source, mod, (add,0))
        self.connect(vector_source, (add,0))
        self.connect(noise, (add,1))
        self.connect(add, throttle, head, self)
        self.connect(add, tag_debug)


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
    parser.add_option("-m", "--modulation", type="string", default="ofdm",
                      help="Modulation type (BPSK/OFDM)")
    parser.add_option("", "--coordinates-m", type="string", default="0.0,0.0",
                      help="Receiver coordinates in meters")
    parser.add_option("", "--coordinates-wgs84", type="string", default="",
                      help="Receiver coordinates in meters")
    parser.add_option("", "--dot-graph", action="store_true", default=False,
                      help="Generate dot-graph file from flowgraph")
    parser.add_option("", "--ssh-proxy", action="store_true", default=False,
                      help="Activate when using a ssh proxy")
    parser.add_option("-t", "--tx_coordinates", type="string", default="0.0,0.0",
                      help="Transmitter starting position for tracking simulations")
    parser.add_option("", "--movement-file", type="string", default="EMPTY",
                      help="csv file with target coordinates. Generate e.g. with MATLAB")                  
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
        print "flowgraph can be found in: "+ file_str

    try:
        #tb.start()

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



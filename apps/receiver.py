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
import numpy as np
import sys
import os
import threading
import time, sched
import socket
import serial
import calendar
sys.path.append("../python")
import rpc_manager as rpc_manager_local
from gpsconfig import *
from tx_bpsk import tx_bpsk
sys.path.append("../python/octoclock_wrapper")
#import octoclock


###############################################################################
# GNU Radio top_block
###############################################################################
class top_block(gr.top_block):
    def __init__(self, options):
        gr.top_block.__init__(self)

        self.options = options

        # create scheduler for retuning
        self.scheduler = sched.scheduler(time.time, time.sleep)
        self.transmit_flag = True # will be set to false after end of constructor

        self.run_loop = False

        coordinates_string = options.coordinates_m.split(",")
        self.coordinates = (float(coordinates_string[0]),float(coordinates_string[1]))
        
        
        # socket addresses
        rpc_port = 6665 + options.id_rx
        rpc_adr = "tcp://*:" + str(rpc_port)
        fusion_center_adr = "tcp://" + options.fusion_center + ":6665"
        probe_port = 5555 + options.id_rx
        probe_adr = "tcp://*:" + str(probe_port)

        # blocks
        self.zmq_probe = zeromq.pub_sink(gr.sizeof_gr_complex, 1, probe_adr, 100, True)
        self.tag_debug = blocks.tag_debug(gr.sizeof_gr_complex*1, "", ""); self.tag_debug.set_display(True)

        if self.options.serial != "":
            if self.options.mcr != 0:
                self.usrp_source = uhd.usrp_source(
                    "serial == " + self.options.serial
                    + ",master_clock_rate == " + str(self.options.mcr),
                    uhd.stream_args(
                        cpu_format="fc32",
                        channels=range(1),
                     ), False
                )
            else:
                self.usrp_source = uhd.usrp_source(
                    "serial == " + self.options.serial,
                    uhd.stream_args(
                        cpu_format="fc32",
                        channels=range(1),
                     ), False
                )

        else:
            if self.options.mcr != 0:
                self.usrp_source = uhd.usrp_source(
                    "master_clock_rate == " + str(self.options.mcr),
                    uhd.stream_args(
                        cpu_format="fc32",
                        channels=range(1),
                    ), False
                )
            else:
                self.usrp_source = uhd.usrp_source(
                    "",
                    uhd.stream_args(
                        cpu_format="fc32",
                        channels=range(1),
                    ), False
                )
        print "passed constructor"
        self.gps = options.gps

        if self.gps != "internal":
            print "Using " + self.gps
            self.usrp_source.set_clock_source("external", 0)
            self.usrp_source.set_time_source("external", 0)
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
        if self.options.log:
            file_name = "../log/receiver_" + time.strftime("%d_%m_%y-%H:%M:%S") + ".txt"
            self.file_sink = blocks.file_sink(gr.sizeof_gr_complex*1, file_name, True)
            self.file_sink.set_unbuffered(False)

        # call constructor at fist to secure synchronization
        self.tx_bpsk_0 = tx_bpsk(
            serial = self.options.serial,
            mcr= self.options.mcr,
            gps = self.gps,
            bandwidth=5000000,
            center_freq=2510000000,
            gain=89.5,
            num_pulses=20000,
            pulse_length=1,
            samp_rate=20000000,
        )
        


        # connects
        self.connect(self.tx_bpsk_0)
        #self.connect(self.usrp_source, self.s_to_v, self.zmq_probe)
        self.connect(self.usrp_source, self.zmq_probe)
        self.connect(self.usrp_source, self.tag_debug)
        if self.options.log:
            self.connect(self.usrp_source, self.file_sink)

        # ZeroMQ
        self.rpc_manager = rpc_manager_local.rpc_manager()
        self.rpc_manager.set_reply_socket(rpc_adr)
        self.rpc_manager.set_request_socket(fusion_center_adr)
        self.rpc_manager.add_interface("set_run_loop",self.set_run_loop)
        self.rpc_manager.add_interface("start_fg",self.start_fg)
        self.rpc_manager.add_interface("set_gain",self.set_gain)
        self.rpc_manager.add_interface("set_samp_rate",self.set_samp_rate)
        self.rpc_manager.add_interface("set_bw",self.set_bw)
        self.rpc_manager.add_interface("set_antenna",self.set_antenna)
        self.rpc_manager.add_interface("get_gps_position",self.get_gps_position)
        self.rpc_manager.add_interface("sync_time",self.sync_time)
        self.rpc_manager.add_interface("program_gps_position",self.program_gps_position)
        self.rpc_manager.add_interface("stop_transmitter", self.stop_transmitter)
        self.rpc_manager.add_interface("start_transmitter", self.start_transmitter)
        self.rpc_manager.add_interface("set_tx_gain", self.set_tx_gain)
        self.rpc_manager.start_watcher()


        # Find out ip address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if not options.ssh_proxy:
            s.connect((options.fusion_center,6665))
        else:
            s.connect(("www.rwth-aachen.de",80))
        self.ip_addr = s.getsockname()[0]
        print "Master clock rate: ", self.usrp_source.get_clock_rate()
        self.stop_transmitter() # transmit flag toggled inside


    '''
    def start_transmitter(self, center_freq, bandwidth, gain, sample_rate):
        print "Transmitter started"
        self.lock()
        self.transmit_flag = True
        self.tx_bpsk_0 = tx_bpsk(
            bandwidth = bandwidth,
            center_freq = center_freq,
            gain = gain,
            num_pulses = 20000,
            pulse_length = 1,
            samp_rate = sample_rate,
            )
        self.connect(self.tx_bpsk_0)
        time.sleep(0.0001)
        self.unlock()
    '''

    def stop_transmitter(self):
        self.lock()
        if self.transmit_flag:
            print "Transmitter stoped"
            self.disconnect(self.tx_bpsk_0)
            self.transmit_flag = False
        time.sleep(0.0001)
        self.unlock()
        self.run_loop = True
    
    def start_transmitter(self):
        self.lock()
        if not self.transmit_flag:
            print "--------------------Transmitter start------------------"
            self.connect(self.tx_bpsk_0)
            self.transmit_flag = True
        time.sleep(0.0001)
        self.unlock()
        self.run_loop = False

    def set_tx_gain(self,tx_gain):
        self.tx_bpsk_0.set_gain(tx_gain)

    def set_run_loop(self, run_loop):
        self.run_loop = run_loop

    def set_samp_rate(self,samp_rate):
        self.usrp_source.set_samp_rate(samp_rate)
        self.tx_bpsk_0.set_samp_rate(samp_rate)

    def set_bw(self,bw):
        self.usrp_source.set_bandwidth(bw,0)
        self.tx_bpsk_0.set_bandwidth(bw)

    def set_gain(self,gain):
        self.usrp_source.set_gain(gain, 0)

    def set_antenna(self,antenna):
        self.usrp_source.set_antenna(antenna, 0)

    def sync_time(self):
        threading.Thread(target = self.sync_time_nmea).start()

    def register_receiver(self):
        first = True
        while(True):
            # register receiver [hostname, usrp_serial, rx_id]
            self.rpc_manager.request("register_receiver",[self.ip_addr, self.usrp_source.get_usrp_info().vals()[2], self.options.id_rx, self.gps, first, self.coordinates])
            first = False
            time.sleep(10)

    def start_fg(self, samples_to_receive, freq, lo_offset, bw, gain, samples_to_receive_calibration, freq_calibration, lo_offset_calibration, bw_calibration, gain_calibration, time_to_recv, autocalibrate, acquisitions, acquisition_time):
        print "reception loop started"
        threading.Thread(target = self.start_reception, args = (samples_to_receive, freq, lo_offset, bw, gain, samples_to_receive_calibration, freq_calibration, lo_offset_calibration, bw_calibration, gain_calibration, time_to_recv, autocalibrate, acquisitions, acquisition_time)).start()


    def start_reception(self, samples_to_receive, freq, lo_offset, bw, gain, samples_to_receive_calibration, freq_calibration, lo_offset_calibration, bw_calibration, gain_calibration, time_to_recv, autocalibrate, acquisitions, acquisition_time):

        if acquisitions == 0:
            infinity = True
        else:
            infinity = False
        
        
        if time_to_recv is None:
            time_to_recv = np.ceil(self.usrp_source.get_time_last_pps().get_real_secs()) + 2.5

        time_now = self.usrp_source.get_time_now().get_real_secs()
        if time_to_recv < time_now:
            print time_to_recv,time_now
            print "Can't start in the past!"
            return
        # retune once to reconfigure receiver
        self.retune(freq, lo_offset, gain, bw)
        usrp = self.usrp_source
        print "Parameters:", usrp.get_center_freq(0),usrp.get_gain(0),usrp.get_samp_rate(),usrp.get_bandwidth(0),samples_to_receive,usrp.get_antenna(0)
        
        time_to_sample_last = 0
        # receiver acquisition loop
        while True:
            # get current time from usrp
            time_now = self.usrp_source.get_time_now().get_real_secs()
            print "time now:",time_now
            # without autocalibration we don't have to retune           
            if not autocalibrate:
                if ((time_to_recv - time_now) < acquisition_time):
                    time_to_sample = uhd.time_spec(time_to_recv)
                    if not time_to_sample == time_to_sample_last:
                        # ask for samples at a specific time
                        stream_cmd = uhd.stream_cmd(uhd.stream_cmd_t.STREAM_MODE_NUM_SAMPS_AND_DONE )
                        # add 300 samples to the burst to get rid of transient
                        stream_cmd.num_samps = samples_to_receive + 300
                        stream_cmd.stream_now = False
                        stream_cmd.time_spec = time_to_sample
                        self.usrp_source.issue_stream_cmd(stream_cmd)
                    # remember last sampling time
                    time_to_sample_last = time_to_sample
                    # update the reception time for next acquisition
                    time_to_recv = time_to_recv + acquisition_time
                    print "Time to sample:", time_to_sample.get_real_secs()
                    print "time to receive:", time_to_recv
                    acquisitions -= 1
                    if not self.run_loop or (acquisitions <= 0 and not infinity):
                        time.sleep(acquisition_time/4)
                        break
                time.sleep(acquisition_time/4)
                
            else:
                if ((time_to_recv - time_now) < 3 *acquisition_time/10):
                    # get times from USRP
                    time_to_sample = uhd.time_spec(time_to_recv + 3 * acquisition_time/10.0)
                    # ask for samples at a specific time
                    stream_cmd = uhd.stream_cmd(uhd.stream_cmd_t.STREAM_MODE_NUM_SAMPS_AND_DONE)
                    # add 300 samples to the burst to get rid of transient
                    stream_cmd.num_samps = samples_to_receive + 300
                    stream_cmd.stream_now = False
                    stream_cmd.time_spec = time_to_sample
                    self.usrp_source.issue_stream_cmd(stream_cmd)
                    time_to_calibrate = uhd.time_spec(time_to_recv + 7 * acquisition_time/10.0)
                    # ask for samples at a specific time
                    stream_cmd = uhd.stream_cmd(uhd.stream_cmd_t.STREAM_MODE_NUM_SAMPS_AND_DONE)
                    # add 300 samples to the burst to get rid of transient
                    stream_cmd.num_samps = samples_to_receive_calibration + 300
                    stream_cmd.stream_now = False
                    stream_cmd.time_spec = time_to_calibrate
                    self.usrp_source.issue_stream_cmd(stream_cmd)
                    # synchronize LOs
                    time_retune_1 = self.usrp_source.get_time_now().get_real_secs()
                    self.retune(freq, lo_offset, gain, bw)
                    time_now = self.usrp_source.get_time_now().get_real_secs()
                    if (time_to_sample > time_now):
                        if (time_now > time_to_recv):
                            print "time_now > time_to_recv"
                            self.scheduler.enter((4 * acquisition_time/10.0), 1, self.retune, ([freq_calibration, lo_offset_calibration, gain_calibration, bw_calibration]))
                        else:
                            print "time_now < time_to_recv"
                            self.scheduler.enter(((time_to_recv-time_now) + 4 * acquisition_time/10.0), 1, self.retune, ([freq_calibration, lo_offset_calibration, gain_calibration, bw_calibration]))
                        self.scheduler.run()
                    print "Time retune 1:", time_retune_1
                    print "Time to sample:", time_to_sample.get_real_secs()
                    if autocalibrate:
                        #print "Time retune 2:", time_retune_2
                        print "Time to calibrate:", time_to_calibrate.get_real_secs()
                    usrp = self.usrp_source
                    print "Parameters:", usrp.get_center_freq(0),usrp.get_gain(0),usrp.get_samp_rate(),usrp.get_bandwidth(0),samples_to_receive,usrp.get_antenna(0)
                    acquisitions -= 1
                    time_to_recv = time_to_recv + acquisition_time
                    if not self.run_loop or (acquisitions <= 0 and not infinity):
                        break
                
        
    def retune(self, freq, lo_offset, gain, bw):
        # synchronize LOs
        time_retune_2 = self.usrp_source.get_time_now().get_real_secs()
        print "Time retune 2:", time_retune_2
        self.usrp_source.set_center_freq(uhd.tune_request(freq, lo_offset), 0)
        self.tx_bpsk_0.set_center_freq(uhd.tune_request(freq, lo_offset))
        self.usrp_source.set_gain(gain,0)
        self.tx_bpsk_0.set_gain(gain)
        self.usrp_source.set_bandwidth(bw,0)
        self.tx_bpsk_0.set_bandwidth(bw)

    def sync_time_nmea(self):
        print "Begin time sync"
        if self.gps == "octoclock":
            clock = octoclock.multi_usrp_clock()
        # get time of last pps from USRP
        last_pps_time = self.usrp_source.get_time_last_pps().get_real_secs()
        print "Last pps time before sync:", last_pps_time
        synced = False
        while not synced:
            if (self.gps == "ltelite") or (self.gps == "leam8f"):
                while self.nmea_external is "":
                    time.sleep(0.1)
                self.nmea_external_lock.acquire()
                s = self.nmea_external
                self.nmea_external_lock.release()
                t = s.split(",")[1].split(".")[0]
                d = s.split(",")[9]
                my_time = time.strptime(t+d,"%H%M%S%d%m%y")
                time_nmea = calendar.timegm(my_time)
                self.usrp_source.set_time_next_pps(uhd.time_spec(time_nmea + 1))
                if self.options.ntp_server and time_nmea > 1400000000:
                    os.system("sudo date +%s -s @"+str(time_nmea))
                    print "System time set to:", str(time_nmea)
                print "Set USRP to NMEA time + 1s:", time_nmea + 1
                synced = True
            else:
                # check for occurence of next pps
                last_pps_time_check = self.usrp_source.get_time_last_pps().get_real_secs()
                print "Check last pps time:", last_pps_time_check
                if last_pps_time_check > last_pps_time:
                    # get pps time from NMEA and set time of next pps
                    if self.gps == "octoclock":

                        time_nmea = clock.get_time_real_secs()

                    else:
                        time_nmea = [int(s) for s in self.usrp_source.get_mboard_sensor("gps_time",0).to_pp_string().split() if s.isdigit()][0]
                    # set internal time registers in USRP
                    self.usrp_source.set_time_next_pps(uhd.time_spec(time_nmea + 1))
                    # set system time if ntp server option activated
                    if self.options.ntp_server and time_nmea > 1400000000:
                        os.system("sudo date +%s -s @"+str(time_nmea))
                        print "System time set to:", str(time_nmea)
                    print "Set USRP to NMEA time + 1s:", time_nmea + 1
                    synced = True
                else:
                    # sleep for 100 msec
                    time.sleep(0.1)
        print "Just after NMEA sync: ", self.usrp_source.get_time_last_pps().get_real_secs()
        time.sleep(1)
        print "After 1s: ", self.usrp_source.get_time_last_pps().get_real_secs()
        print "NMEA time sync complete!"
        self.rpc_manager.request("sync_success",[self.ip_addr, self.options.ntp_server])

    def poll_lte_lite(self):
        while True:
            nmea = self.ser.readline().replace("\x00","").replace("\n","")
            if "GPRMC" in nmea and len(nmea.split(",")) ==13 :
                self.nmea_external_lock.acquire()
                self.nmea_external = nmea
                self.nmea_external_lock.release()

    def poll_lea_m8f(self):
        while True:
            nmea = self.ser.readline().replace("\x00","").replace("\n","")
            # nmea identifier different for lea m8f
            if "GNRMC" in nmea and len(nmea.split(",")) ==13 :
                self.nmea_external_lock.acquire()
                self.nmea_external = nmea
                self.nmea_external_lock.release()

    def get_gps_gprmc(self):
        if self.gps == "octoclock":
            clock = octoclock.multi_usrp_clock()
            nmea = clock.get_sensor("gps_gprmc").split(":")[1].strip()
        elif (self.gps == "ltelite") or (self.gps == "leam8f"):
            while self.nmea_external is "":
                time.sleep(0.1)
            self.nmea_external_lock.acquire()
            nmea = self.nmea_external
            self.nmea_external_lock.release()
        else:
            nmea = self.usrp_source.get_mboard_sensor("gps_gprmc",0).value

        return nmea

    def get_gps_position(self):
        if self.options.coordinates_wgs84 != "":
            coordinates_wgs84_string = self.options.coordinates_wgs84.split(",")
            latitude = float(coordinates_wgs84_string[0])
            longitude = float(coordinates_wgs84_string[1]) 
        else:
            if self.gps == "octoclock":
                # set to ict rooftop if using octoclock
                latitude = 50.7793333333
                longitude = 6.06295555555
            else:
                nmea = self.get_gps_gprmc()
                latitude = nmea.split(",")[3:5]
                # the NMEA sentence may vary for each receiver. This code works for 
                # the next structure in latitude DDMM.XXXXXX
                if len(latitude) > 1:
                    if latitude[0] != "":
                        if latitude[1] == "N":
                            latitude = int(latitude[0][0:2])+(float(latitude[0][2:])/60)
                        else:
                            latitude = -int(latitude[0][0:2])-(float(latitude[0][2:])/60)

                        longitude = nmea.split(",")[5:7]
                        # the NMEA sentence may vary for each receiver. This code works for 
                        # the next structure in longitude DDDMM.XXXXXX
                        if longitude[1] == "E":
                            longitude = int(longitude[0][0:3])+(float(longitude[0][3:])/60)
                        else:
                            longitude = -int(longitude[0][0:3])-(float(longitude[0][3:])/60)
                    else:
                        # set to ict rooftop if coordinates are missing
                        print "Invalid NMEA-message! Set to default coordinates"
                        latitude = 50.7793333333
                        longitude = 6.06295555555
                else:
                    # set to ict rooftop if coordinates are missing
                    print "Invalid NMEA-message! Set to default coordinates"
                    latitude = 50.7793333333
                    longitude = 6.06295555555
        # basemap requires [long,lat]; we want to put in [lat,long] => swap
        return [longitude, latitude]

    def program_gps_position(self, latitude, longitude, altitude):
        # needed in ublox settings; by now we assume at least dm accuracy
        ground_truth_accuracy = 0.3
        print 'Configure u-blox TMODE2 through USB'        
        set_ublox_coordinates_fixed(latitude, longitude, altitude, ground_truth_accuracy)
        print "Check position through UHD NMEA"
        usrp_coordinates = self.get_gps_position()
        # compare values with the accuracy possible in u-blox devices
        if int(usrp_coordinates[0]*1e7) == int(longitude*1e7) and int(usrp_coordinates[1]*1e7) == int(latitude*1e7):
            print "Correct"
        else:
            print "Error: difference between USB and UHD"
            print "USRP coordinates: ",usrp_coordinates[0], usrp_coordinates[1]
            print "Difference: ",usrp_coordinates[0] - longitude, usrp_coordinates[1] - latitude


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
    parser.add_option("-g", "--gps", type="string", default="internal",
                      help="GPS type")
    parser.add_option("-i", "--id-rx", type="int", default="1",
                      help="Receiver ID")
    parser.add_option("--mcr", type="float", default="40e6",
                      help="Master clock rate")
    parser.add_option("", "--coordinates-m", type="string", default="0.0,0.0",
                      help="Receiver coordinates in meters")
    parser.add_option("", "--coordinates-wgs84", type="string", default="",
                      help="Receiver coordinates in meters")
    parser.add_option("", "--dot-graph", action="store_true", default=False,
                      help="Generate dot-graph file from flowgraph")
    parser.add_option("", "--ntp-server", action="store_true", default=False,
                      help="Activate ntp server")
    parser.add_option("-l", "--log", action="store_true", default=False,
                      help="Activate receiver logging")
    parser.add_option("", "--ssh-proxy", action="store_true", default=False,
                      help="Activate when using a ssh proxy")
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
        tb.usrp_source.stop()

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
    tb.ser.close()
    tb = None

#!/usr/bin/env python

###############################################################################
# Imports
###############################################################################
from __future__ import print_function
from optparse import OptionParser
from gnuradio.eng_option import eng_option
import sys
import os
import pprint
from gnuradio import zeromq
import signal
import numpy as np
import time
import threading
import json
from mpl_toolkits.basemap import Basemap
import copy
sys.path.append("../python")
import rpc_manager as rpc_manager_local
import receiver_interface
import chan94_algorithm
import grid_based_algorithm

class fusion_center():
    def __init__(self, options):
        # give Ctrl+C back to system
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        # socket addresses
        rpc_adr = "tcp://*:6665"

        self.samples_to_receive = int(options.num_samples)
        self.frequency = float(options.frequency)
        self.samp_rate = float(options.samp_rate)
        self.bw = float(options.bandwidth)
        self.interpolation = int(options.interpolation)
        self.lo_offset = float(options.lo_offset)
        self.samples_to_receive_calibration = int(options.num_samples_calibration)
        self.frequency_calibration = float(options.frequency_calibration)
        self.bw_calibration = float(options.bandwidth_calibration)
        self.lo_offset_calibration = float(options.lo_offset_calibration)
        self.gain = float(options.gain)
        self.gain_calibration = float(options.gain_calibration)
        self.antenna = options.antenna
        self.auto_calibrate = not options.no_auto_calibrate

        self.receivers = {}
        self.ref_receiver = ""
        self.guis = {}

        self.grid_based = {"resolution":10,"num_samples":self.samples_to_receive * self.interpolation}

        self.delay_history = []
        self.delay_calibration = []
        self.delay_auto_calibration = []
        self.store_results = False
        self.store_samples = False
        self.results_file = ""
        if not os.path.exists("../log"):
                os.makedirs("../log")
        self.results = None
        self.run_loop = False
        self.localizing = False
        self.ntp_sync = False

        # ICT + surroundings
        self.bbox = 6.0580,50.7775,6.0690,50.7810
        # Football court
        #self.bbox = 6.06429,50.77697,6.07271,50.78033
        # ICT indoor
        #self.bbox = 6.061698496341705,50.77914404797512,6.063739657402039,50.77976138469289
        #self.bbox = 6.061738267169996,50.779093354299285,6.063693919000911,50.77980828706738
        # UPB Campus
        self.bbox = -75.59124,6.24113,-75.58851,6.24261
        self.init_map()


        # ZeroMQ
        self.probe_manager = zeromq.probe_manager()
        self.rpc_manager = rpc_manager_local.rpc_manager()
        self.rpc_manager.set_reply_socket(rpc_adr)
        self.rpc_manager.add_interface("register_gui",self.register_gui)
        self.rpc_manager.add_interface("sync_position",self.sync_position)
        self.rpc_manager.add_interface("register_receiver",self.register_receiver)
        self.rpc_manager.add_interface("forward_chat",self.forward_chat)
        self.rpc_manager.add_interface("start_receivers",self.start_receivers)
        self.rpc_manager.add_interface("reset_receivers",self.reset_receivers)
        self.rpc_manager.add_interface("update_receivers",self.update_receivers)
        self.rpc_manager.add_interface("get_gui_gps_position",self.get_gui_gps_position)
        self.rpc_manager.add_interface("localize",self.localize)
        self.rpc_manager.add_interface("localize_loop",self.localize_loop)
        self.rpc_manager.add_interface("calibrate",self.calibrate)
        self.rpc_manager.add_interface("remove_calibration",self.remove_calibration)
        self.rpc_manager.add_interface("start_correlation",self.start_correlation)
        self.rpc_manager.add_interface("start_correlation_loop",self.start_correlation_loop)
        self.rpc_manager.add_interface("stop_loop",self.stop_loop)
        self.rpc_manager.add_interface("set_frequency",self.set_frequency)
        self.rpc_manager.add_interface("set_lo_offset",self.set_lo_offset)
        self.rpc_manager.add_interface("set_samples_to_receive",self.set_samples_to_receive)
        self.rpc_manager.add_interface("set_samp_rate",self.set_samp_rate)
        self.rpc_manager.add_interface("set_bw",self.set_bw)
        self.rpc_manager.add_interface("set_interpolation",self.set_interpolation)
        self.rpc_manager.add_interface("set_frequency_calibration",self.set_frequency_calibration)
        self.rpc_manager.add_interface("set_lo_offset_calibration",self.set_lo_offset_calibration)
        self.rpc_manager.add_interface("set_samples_to_receive_calibration",self.set_samples_to_receive_calibration)
        self.rpc_manager.add_interface("set_bw_calibration",self.set_bw_calibration)
        self.rpc_manager.add_interface("set_gain",self.set_gain)
        self.rpc_manager.add_interface("set_gain_calibration",self.set_gain_calibration)
        self.rpc_manager.add_interface("set_antenna",self.set_antenna)
        self.rpc_manager.add_interface("set_selected_position",self.set_selected_position)
        self.rpc_manager.add_interface("set_ref_receiver",self.set_ref_receiver)
        self.rpc_manager.add_interface("set_auto_calibrate",self.set_auto_calibrate)
        self.rpc_manager.add_interface("set_TDOA_grid_based_resolution",self.set_TDOA_grid_based_resolution)
        self.rpc_manager.add_interface("set_TDOA_grid_based_num_samples",self.set_TDOA_grid_based_num_samples)
        self.rpc_manager.add_interface("sync_success",self.sync_ntp)
        self.rpc_manager.add_interface("set_bbox",self.set_bbox)
        self.rpc_manager.start_watcher()

        threading.Thread(target = self.poll_gps_position).start()

    def init_map(self):
        # get reference UTM grid
        lon = self.bbox[0]
        lat = self.bbox[1]
        if lat>=72:
            lat_0 = 72
            if 0<=lon and lon<= 9:
                lon_0 = 0
            elif 9<=lon and lon<= 21:
                lon_0 = 9
            elif 21<=lon and lon<= 33:
                lon_0 = 21
            elif 33<=lon and lon<= 42:
                lon_0 = 33
            else:
                lon_0 = int(lon/6)*6

        elif 56<=lat and lat<= 64:
            lat_0 = 56
            if 3<=lon and lon<=12:
                lon_0 = 3
            else:
                lon_0 = int(lon/6)*6

        else:
            lat_0 = int(lat/8)*8
            lon_0 = int(lon/6)*6

        self.basemap = Basemap(llcrnrlon=self.bbox[0], llcrnrlat=self.bbox[1],
                      urcrnrlon=self.bbox[2], urcrnrlat=self.bbox[3],
                      projection='tmerc', lon_0=lon_0, lat_0=lat_0)

        #self.frequency_calibration = 602000000
        self.coordinates_calibration = self.basemap(50.745597, 6.043278)

    def calibrate(self, coordinates, delays=None):
        # no autocalibration
        if self.results is not None and delays is None:
            if self.results["delay"] is not None:
                if len(self.results["delay"]) > 0:
                    ref_receiver = self.receivers[self.ref_receiver]
                    if ref_receiver.selected_position == "manual":
                        pos_ref = ref_receiver.coordinates
                    else:
                        pos_ref = ref_receiver.coordinates_gps
                    index_delay = 0
                    for i in range(0,len(self.receivers)):
                        if not self.ref_receiver == self.receivers.keys()[i]:
                            receiver = self.receivers.values()[i]
                            if receiver.selected_position == "manual":
                                pos_receiver = receiver.coordinates
                            else:
                                pos_receiver = receiver.coordinates_gps
                            d_ref = np.linalg.norm(np.array(coordinates)-pos_ref)
                            d_receiver = np.linalg.norm(np.array(coordinates)-pos_receiver)
                            delay_true = (d_receiver-d_ref) * self.samp_rate * self.interpolation / 299700000
                            print("True delay: ",delay_true)
                            if len(self.delay_calibration) < len(self.results["delay"]):
                                self.delay_calibration.append(int(np.floor(delay_true)-self.results["delay"][index_delay]))
                            else:
                                self.delay_calibration[index_delay] = int(np.floor(delay_true)-self.results["delay"][index_delay])+self.delay_calibration[index_delay]
                            index_delay += 1
            print ("Delay calibration: ", self.delay_calibration)
        # autocalibration
        else:
            ref_receiver = self.receivers[self.ref_receiver]
            if ref_receiver.selected_position == "manual":
                pos_ref = ref_receiver.coordinates
            else:
                pos_ref = ref_receiver.coordinates_gps
            index_delay_auto = 0
            for i in range(0,len(self.receivers)):
                if not self.ref_receiver == self.receivers.keys()[i]:
                    receiver = self.receivers.values()[i]
                    if receiver.selected_position == "manual":
                        pos_receiver = receiver.coordinates
                    else:
                        pos_receiver = receiver.coordinates_gps
                    d_ref = np.linalg.norm(np.array(coordinates)-pos_ref)
                    d_receiver = np.linalg.norm(np.array(coordinates)-pos_receiver)
                    delay_true = (d_receiver-d_ref) * self.samp_rate * self.interpolation / 299700000
                    print("True delay: ",delay_true)
                    if len(self.delay_auto_calibration) < len(delays):
                        self.delay_auto_calibration.append(int(np.floor(delay_true)-delays[index_delay_auto]))
                    else:
                        self.delay_auto_calibration[index_delay_auto] = int(np.floor(delay_true)-delays[index_delay_auto])
                    index_delay_auto += 1
            print ("Delay auto calibration: ", self.delay_auto_calibration)

    def remove_calibration(self):
        self.delay_calibration = []
        self.delay_auto_calibration = []

    def forward_chat(self, chat):
        for gui in self.guis.values():
            gui.rpc_manager.request("new_chat",[chat])

    def sync_position(self, serial, coordinates):
        self.receivers[serial].coordinates = coordinates
        for gui in self.guis.values():
            gui.rpc_manager.request("sync_position",[serial, coordinates])

    def poll_gps_position(self):
        while True:
            for receiver in self.receivers.values():
                coordinates_gps = receiver.get_gps_position()
                if coordinates_gps != None:
                    receiver.coordinates_gps = self.basemap(coordinates_gps[0],coordinates_gps[1])
            time.sleep(1)

    def get_gui_gps_position(self, serial):
        for gui in self.guis.values():
            threading.Thread(target = gui.rpc_manager.request, args = ("set_gps_position", [serial, self.receivers[serial].coordinates_gps])).start()

    def register_gui(self, ip_addr, hostname, id_gui, first):
        was_not_registered = False
        gui_serial = hostname + str(id_gui)
        if not self.guis.has_key(gui_serial):
            gui = gui_interface("tcp://" + ip_addr + ":" + str(7775 + id_gui), hostname)
            self.guis[gui_serial] = gui
            was_not_registered = True
        else:
            gui = self.guis[gui_serial]
        if first or was_not_registered:
            for serial in self.receivers:
                # request registration of each receiver in gui
                gui.rpc_manager.request("register_receiver",[serial, self.receivers[serial].gain, self.receivers[serial].antenna, self.receivers[serial].gain_calibration])
                gui.rpc_manager.request("sync_position",[serial, self.receivers[serial].coordinates])
                gui.rpc_manager.request("set_gps_position",[serial, self.receivers[serial].coordinates_gps])
            gui.rpc_manager.request("set_gui_frequency",[self.frequency])
            gui.rpc_manager.request("set_gui_lo_offset",[self.lo_offset])
            gui.rpc_manager.request("set_gui_samples_to_receive",[self.samples_to_receive])
            gui.rpc_manager.request("set_gui_bw",[self.bw])
            gui.rpc_manager.request("set_gui_interpolation",[self.interpolation])
            gui.rpc_manager.request("set_gui_samp_rate",[self.samp_rate])
            gui.rpc_manager.request("set_gui_frequency_calibration",[self.frequency_calibration])
            gui.rpc_manager.request("set_gui_lo_offset_calibration",[self.lo_offset_calibration])
            gui.rpc_manager.request("set_gui_samples_to_receive_calibration",[self.samples_to_receive_calibration])
            gui.rpc_manager.request("set_gui_bw_calibration",[self.bw_calibration])
            gui.rpc_manager.request("set_gui_auto_calibrate",[self.auto_calibrate])
            gui.rpc_manager.request("set_gui_TDOA_grid_based_resolution",[self.grid_based["resolution"]])
            gui.rpc_manager.request("set_gui_TDOA_grid_based_num_samples",[self.grid_based["num_samples"]])

            for gui in self.guis.values():
                for serial in self.guis:
                    # request registration in each gui
                    gui.rpc_manager.request("register_another_gui",[serial])
            print(gui_serial, "registered")
        return self.bbox

    def register_receiver(self, hostname, serial, id_rx, gps, first):
        was_not_registered = False
        rpc_adr = "tcp://" + hostname + ":" + str(6665 + id_rx)
        probe_adr = "tcp://" + hostname + ":" + str(5555 + id_rx)
        if not self.receivers.has_key(serial):
            # create new receiver in fusion center
            self.receivers[serial] = receiver_interface.receiver_interface(rpc_adr, probe_adr)
            was_not_registered = True
        elif first:
            # receiver might have restarted, so remove and add again
            self.receivers.pop(serial)
            self.receivers[serial] = receiver_interface.receiver_interface(rpc_adr, probe_adr)
        if first or was_not_registered:
            # set parameters of receiver in fusion center
            receiver = self.receivers[serial]
            receiver.set_gain(self.gain)
            receiver.set_antenna(self.antenna)
            receiver.set_bw(self.bw)
            receiver.interpolation = self.interpolation
            receiver.set_samp_rate(self.samp_rate)
            receiver.frequency = self.frequency
            receiver.lo_offset = self.lo_offset
            receiver.samples_to_receive = self.samples_to_receive
            receiver.gain_calibration = self.gain_calibration
            receiver.bw_calibration = self.bw_calibration
            receiver.frequency_calibration = self.frequency_calibration
            receiver.lo_offset_calibration = self.lo_offset_calibration
            receiver.samples_to_receive_calibration = self.samples_to_receive_calibration
            receiver.gps = gps
            receiver.auto_calibrate = self.auto_calibrate
            self.probe_manager.add_socket(receiver.probe_address, 'complex64', receiver.receive_samples)
            for gui in self.guis.values():
                # request registration in each gui
                gui.rpc_manager.request("register_receiver",[serial, receiver.gain, receiver.antenna, receiver.gain_calibration])
            self.update_receivers()
            print(serial, "registered")
            #threading.Thread(target = self.finish_register(serial)).start()
            #self.reset_receivers()

    def finish_register(self, serial):
        time.sleep(5)
        self.reset_receivers()
        print(serial, "registered")

    def start_receivers(self, freq, lo_offset, samples_to_receive):
        time_to_receive = time.time() + 1
        for receiver in self.receivers.values():
            threading.Thread(target = self.start_receiver, args = (receiver, time_to_receive, freq, lo_offset, samples_to_receive)).start()


    def start_receiver(self, receiver, time_to_receive, freq, lo_offset, samples_to_receive):
            receiver.samples = []
            receiver.samples_calibration = []
            receiver.first_packet = True
            receiver.reception_complete = False
            receiver.frequency = freq
            receiver.lo_offset = lo_offset
            receiver.samples_to_receive = samples_to_receive
            receiver.set_run_loop(self.run_loop)
            if self.ntp_sync:
                receiver.request_samples(str(time_to_receive))
            else:
                receiver.request_samples(None)

    def reset_receivers(self):
        #self.update_timer.stop()
        for i in range(0,1000000):
            self.probe_manager.watcher()
        for receiver in self.receivers.values():
            receiver.samples = []
            receiver.samples_calibration = []
            receiver.first_packet = True
            receiver.reception_complete = False
        #self.connect(self.update_timer, QtCore.SIGNAL("timeout()"), self.probe_manager.watcher)
        #self.update_timer.start(33)
        self.probe_manager.watcher()

    def update_receivers(self):
        for receiver in self.receivers.values():
            receiver.set_gain(receiver.gain)
            receiver.set_bw(self.bw)
            receiver.set_interpolation(self.interpolation)
            receiver.set_samp_rate(self.samp_rate)
            receiver.set_antenna(receiver.antenna)
            receiver.frequency = self.frequency
            receiver.lo_offset = self.lo_offset
            receiver.samples_to_receive = self.samples_to_receive
            receiver.auto_calibrate = self.auto_calibrate

    def start_correlation(self, freq, lo_offset, samples_to_receive):
        self.start_receivers(freq, lo_offset, samples_to_receive)

    def start_correlation_loop(self, freq, lo_offset, samples_to_receive):
        self.store_results = True
        self.results_file = "../log/results_" + time.strftime("%d_%m_%y-%H:%M:%S") + ".txt"
        self.store_samples = True
        self.samples_file = "../log/samples_" + time.strftime("%d_%m_%y-%H:%M:%S") + ".txt"
        print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
        print("time,delays(1-2,1-3,1-X...),delays_calibration(1-2,1-3,1-X...),delays_auto_calibration(1-2,1-3,1-X...),sampling_rate,frequency,frequency_calibration,calibration_position,interpolation,bandwidth,samples_to_receive,lo_offset,bbox,receivers_positions,selected_positions,receivers_gps,receivers_antenna,receivers_gain,estimated_positions,index_ref_receiver,auto_calibrate", file=open(self.results_file,"a"))
        print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
        self.run_loop = True
        self.start_receivers(freq, lo_offset, samples_to_receive)

    def localize(self, freq, lo_offset, samples_to_receive):
        if len(self.receivers) > 2:
            self.localizing = True
            self.start_receivers(freq, lo_offset, samples_to_receive)

    def localize_loop(self, freq, lo_offset, samples_to_receive):
        if len(self.receivers) > 2:
            self.localizing = True
            self.store_results = True
            self.results_file = "../log/results_" + time.strftime("%d_%m_%y-%H:%M:%S") + ".txt"
            self.store_samples = False
            #self.samples_file = "../log/samples_" + time.strftime("%d_%m_%y-%H:%M:%S") + ".txt"
            print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
            print("time,delays(1-2,1-3,1-X...),delays_calibration(1-2,1-3,1-X...),delays_auto_calibration(1-2,1-3,1-X...),sampling_rate,frequency,frequency_calibration,calibration_position,interpolation,bandwidth,samples_to_receive,lo_offset,bbox,receivers_positions,selected_positions,receivers_gps,receivers_antenna,receivers_gain,estimated_positions,index_ref_receiver,auto_calibrate", file=open(self.results_file,"a"))
            print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
            self.run_loop = True
            self.start_receivers(freq, lo_offset, samples_to_receive)

    def stop_loop(self):
        self.run_loop = False
        self.store_results = False
        self.delay_history = []
        self.localizing = False
        for receiver in self.receivers.values():
            threading.Thread(target = receiver.set_run_loop, args = [self.run_loop]).start()

    def set_frequency(self, frequency):
        self.frequency = frequency
        for receiver in self.receivers.values():
            receiver.frequency = frequency
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_frequency",[frequency])

    def set_lo_offset(self, lo_offset):
        self.lo_offset = lo_offset
        for receiver in self.receivers.values():
            receiver.lo_offset = lo_offset
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_lo_offset",[lo_offset])

    def set_samples_to_receive(self, samples_to_receive):
        self.samples_to_receive = samples_to_receive
        for receiver in self.receivers.values():
            receiver.samples_to_receive = samples_to_receive
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_samples_to_receive",[samples_to_receive])

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        for receiver in self.receivers.values():
            receiver.samp_rate = samp_rate
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_samp_rate",[samp_rate])

    def set_bw(self, bw):
        self.bw = bw
        for receiver in self.receivers.values():
            receiver.bw = bw
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_bw",[bw])

    def set_interpolation(self, interpolation):
        self.interpolation = interpolation
        for receiver in self.receivers.values():
            receiver.interpolation = interpolation
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_interpolation",[interpolation])

    def set_frequency_calibration(self, frequency):
        self.frequency_calibration = frequency
        for receiver in self.receivers.values():
            receiver.frequency_calibration = frequency
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_frequency_calibration",[frequency])

    def set_lo_offset_calibration(self, lo_offset):
        self.lo_offset_calibration = lo_offset
        for receiver in self.receivers.values():
            receiver.lo_offset_calibration = lo_offset
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_lo_offset_calibration",[lo_offset])

    def set_samples_to_receive_calibration(self, samples_to_receive):
        self.samples_to_receive_calibration = samples_to_receive
        for receiver in self.receivers.values():
            receiver.samples_to_receive_calibration = samples_to_receive
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_samples_to_receive_calibration",[samples_to_receive])

    def set_bw_calibration(self, bw):
        self.bw_calibration = bw
        for receiver in self.receivers.values():
            receiver.bw_calibration = bw
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_bw_calibration",[bw])

    def set_gain(self, gain, serial):
        self.receivers[serial].gain = gain
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_gain",[gain, serial])

    def set_gain_calibration(self, gain, serial):
        self.receivers[serial].gain_calibration = gain
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_gain_calibration",[gain, serial])

    def set_antenna(self, antenna, serial):
        self.receivers[serial].antenna = antenna
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_antenna",[antenna, serial])

    def set_selected_position(self, selected_position, serial):
        self.receivers[serial].selected_position = selected_position
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_selected_position",[selected_position, serial])

    def set_ref_receiver(self, ref_receiver):
        self.ref_receiver = ref_receiver
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_ref_receiver",[ref_receiver])

    def set_auto_calibrate(self, auto_calibrate):
        self.auto_calibrate = auto_calibrate
        for receiver in self.receivers.values():
            receiver.auto_calibrate = auto_calibrate
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_auto_calibrate",[auto_calibrate])

    def set_TDOA_grid_based_resolution(self, resolution):
        self.grid_based["resolution"] = resolution
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_TDOA_grid_based_resolution",[resolution])

    def set_TDOA_grid_based_num_samples(self, num_samples):
        num_samples = min(num_samples,self.samples_to_receive * self.interpolation)
        self.grid_based["num_samples"] = num_samples
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_TDOA_grid_based_num_samples",[num_samples])

    def sync_ntp(self, ip_address, is_ntp_server):
        if is_ntp_server:
            print("Synchronize time to receiver NTP server ",ip_address)
            os.system("sudo ntpdate -u " + ip_address)
            self.ntp_sync = True
            print("NTP synchronization done")

    def set_bbox(self, bbox):
        self.bbox = bbox
        self.init_map()
        for gui in self.guis.values():
            gui.rpc_manager.request("init_map",[bbox])



    def process_results(self, receivers, delay_auto_calibration):
        estimated_positions = {}

        if self.auto_calibrate and len(receivers) > 1:
            correlation, delay = self.correlate(receivers, True)
            self.calibrate(self.coordinates_calibration, delay)

        if len(delay_auto_calibration) > 0 and self.auto_calibrate:
            #for i in range(0,len(delay_auto_calibration)):
            #    receivers.values()[i+1].samples = np.roll(receivers.values()[i+1].samples,delay_auto_calibration[i])
            index_delay_auto = 0
            for i in range(0,len(receivers)):
                if not self.ref_receiver == receivers.keys()[i]:
                    receivers.values()[i].samples = np.roll(receivers.values()[i].samples,delay_auto_calibration[index_delay_auto])
                    index_delay_auto += 1

        if len(self.delay_calibration) > 0:
            #for i in range(0,len(self.delay_calibration)):
            #    receivers.values()[i+1].samples = np.roll(receivers.values()[i+1].samples,self.delay_calibration[i])
            index_delay = 0
            for i in range(0,len(receivers)):
                if not self.ref_receiver == receivers.keys()[i]:
                    receivers.values()[i].samples = np.roll(receivers.values()[i].samples,self.delay_calibration[index_delay])
                    index_delay += 1

        if self.localizing:
            estimated_positions["chan"] = chan94_algorithm.localize(receivers, self.ref_receiver)
            estimated_positions["grid_based"] = grid_based_algorithm.localize(receivers,np.round(self.basemap(self.bbox[2],self.bbox[3])), self.grid_based["resolution"], self.grid_based["num_samples"], self.ref_receiver)
            if not self.run_loop:
                self.localizing = False

        correlation, delay = None,None
        if len(receivers) > 1:
            correlation, delay = self.correlate(receivers)
            if len(self.delay_history) < len(delay):
                self.delay_history = []
                for i in range(0,len(delay)):
                    self.delay_history.append([])
            for i in range(0,len(delay)):
                self.delay_history[i].append(delay[i])
        receivers_samples = []
        for receiver in receivers.values():
            receivers_samples.append(receiver.samples)
        self.results = {"receivers":receivers_samples,"correlation":correlation,"delay":delay,"delay_history":self.delay_history,"estimated_positions":estimated_positions}

        for gui in self.guis.values():
            gui.rpc_manager.request("get_results",[self.results])

        if self.store_results:
            # build receivers strings for log file
            receivers_position = "["
            selected_positions = "["
            receivers_gps = "["
            receivers_antenna = "["
            receivers_gain = "["
            i = 1
            for receiver in receivers.values():
                if i == 1:
                    if receiver.selected_position == "manual":
                        receivers_position = receivers_position + str(receiver.coordinates)
                    else:
                        receivers_position = receivers_position + str(receiver.coordinates_gps)
                    selected_positions = selected_positions + "'" + receiver.selected_position + "'"
                    receivers_gps = receivers_gps + "'" + receiver.gps + "'"
                    receivers_antenna = receivers_antenna + "'" + receiver.antenna + "'"
                    receivers_gain = receivers_gain + str(receiver.gain)
                else:
                    if receiver.selected_position == "manual":
                        receivers_position = receivers_position + "," + str(receiver.coordinates)
                    else:
                        receivers_position = receivers_position + "," + str(receiver.coordinates_gps)
                    selected_positions = selected_positions + "," + "'" + receiver.selected_position + "'"
                    receivers_gps = receivers_gps + "," + "'" + receiver.gps + "'"
                    receivers_antenna = receivers_antenna + "," + "'" + receiver.antenna + "'"
                    receivers_gain = receivers_gain + "," + str(receiver.gain)
                i = i + 1
            receivers_position = receivers_position + "]"
            selected_positions = selected_positions + "]"
            receivers_gps = receivers_gps + "]"
            receivers_antenna = receivers_antenna + "]"
            receivers_gain = receivers_gain + "]"

            # remove grid from results to log
            for key in estimated_positions.keys():
                if key == "grid_based":
                    estimated_positions[key].pop("grid")

            for i in range(0,len(receivers)):
                if receivers.keys()[i] == self.ref_receiver:
                    index_ref_receiver = i

            line = "[" + str(time.time()) + "," + str(self.results["delay"]) + "," + str(self.delay_calibration) + "," + str(delay_auto_calibration) + "," + str(self.samp_rate) + "," + str(self.frequency) + "," + str(self.frequency_calibration) + "," + str(self.coordinates_calibration) + "," + str(self.interpolation) + "," + str(self.bw)+ "," + str(self.samples_to_receive) + "," + str(self.lo_offset) + "," + str(self.bbox) + "," + receivers_position + "," + selected_positions + "," + receivers_gps + "," + receivers_antenna + "," + receivers_gain + "," + str(estimated_positions) + "," + str(index_ref_receiver) + "," + str(self.auto_calibrate) + "]"
            f = open(self.results_file,"a")
            pprint.pprint(line,f,width=9000)
            f.close()

            if self.store_samples:
                f_s = open(self.samples_file,"a")
                pprint.pprint("[" + str(self.results["correlation"]) + "," + str(self.results["receivers"][0].tolist()) + "]",f_s,width=9000)
                f_s.close()

    def main_loop(self):
        while True:
            time.sleep(0.01)
            if all(self.receivers[key].reception_complete for key in self.receivers) and len(self.receivers) > 0:
                receivers = copy.deepcopy(self.receivers)
                threading.Thread(target = self.process_results, args = (receivers,self.delay_auto_calibration,)).start()
                for receiver in self.receivers.values():
                    receiver.samples = []
                    receiver.samples_calibration = []
                    receiver.first_packet = True
                    receiver.reception_complete = False
            else:
                self.probe_manager.watcher()

    def correlate(self, receivers, calibration=False):
        correlation = []
        for receiver in receivers:
            if not self.ref_receiver == receiver:
                if not calibration:
                    correlation.append(np.absolute(np.correlate(receivers[receiver].samples, receivers[self.ref_receiver].samples, "full", False)).tolist())
                    delay = (np.argmax(correlation, axis=1) - self.samples_to_receive * self.interpolation + 1).tolist()
                else:
                    correlation.append(np.absolute(np.correlate(receivers[receiver].samples_calibration, receivers[self.ref_receiver].samples_calibration, "full", False)).tolist())
                    delay = (np.argmax(correlation, axis=1) - self.samples_to_receive_calibration * self.interpolation + 1).tolist()
        print("Delay:", delay, "samples")
        return correlation, delay

class gui_interface():
    def __init__(self, rpc_address, hostname):
        self.rpc_address = rpc_address
        self.hostname = hostname
        self.rpc_manager = rpc_manager_local.rpc_manager()
        self.rpc_manager.set_request_socket(rpc_address)


###############################################################################
# Options Parser
###############################################################################
def parse_options():
    """ Options parser. """
    parser = OptionParser(option_class=eng_option, usage="%prog: [options]")
    parser.add_option("-s", "--servername", type="string", default="localhost",
                      help="Server hostname")
    parser.add_option("-c", "--clientname", type="string", default="localhost",
                      help="Server hostname")
    parser.add_option("", "--num-samples", type="string", default="300",
                      help="Number of samples in burst")
    parser.add_option("", "--interpolation", type="string", default="10",
                      help="Interpolation factor")
    parser.add_option("", "--frequency", type="string", default="2.51e9",
                      help="Frequency")
    parser.add_option("", "--samp-rate", type="string", default="50e6",
                      help="Sampling rate")
    parser.add_option("", "--lo-offset", type="string", default="0",
                      help="LO offset")
    parser.add_option("", "--bandwidth", type="string", default="5e6",
                      help="Bandwidth")
    parser.add_option("", "--num-samples-calibration", type="string", default="300",
                      help="Number of samples in burst for calibration")
    parser.add_option("", "--interpolation-calibration", type="string", default="10",
                      help="Interpolation factor for calibration")
    parser.add_option("", "--frequency-calibration", type="string", default="2.37e9",
                      help="Frequency for calibration")
    parser.add_option("", "--samp-rate-calibration", type="string", default="50e6",
                      help="Sampling rate for calibration")
    parser.add_option("", "--lo-offset-calibration", type="string", default="0",
                      help="LO offset for calibration")
    parser.add_option("", "--bandwidth-calibration", type="string", default="5e6",
                      help="Bandwidth for calibration")
    parser.add_option("", "--antenna", type="string", default="RX2",
                      help="Antenna to use")
    parser.add_option("", "--gain", type="float", default="30",
                      help="Gain in dB")
    parser.add_option("", "--gain-calibration", type="float", default="30",
                      help="Gain in dB for calibration")
    parser.add_option("", "--no-auto-calibrate", action="store_true", default=False,
                      help="Deactivate reference calibration station")
    (options, args) = parser.parse_args()
    return options

###############################################################################
# Main
###############################################################################
if __name__ == "__main__":
    options = parse_options()
    fc = fusion_center(options)
    fc.main_loop()

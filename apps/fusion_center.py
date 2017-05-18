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
from scipy import interpolate
from scipy import signal as sig
import time
import threading
import json
from mpl_toolkits.basemap import Basemap
import copy
sys.path.append("../python")
import rpc_manager as rpc_manager_local
import probe_manager as probe_manager_local
import receiver_interface
import chan94_algorithm, kalman
import grid_based_algorithm
import dop
from interpolation import corr_spline_interpolation
import mds_self_tdoa
from procrustes import procrustes
import helpers

class fusion_center():
    def __init__(self, options):
        # give Ctrl+C back to system
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        # socket addresses
        rpc_adr = "tcp://*:6665"

        #debug
        self.cnt_smpl_log = 0

        self.samples_to_receive = int(options.num_samples)
        self.frequency = float(options.frequency)
        self.samp_rate = float(options.samp_rate)
        self.bw = float(options.bandwidth)
        self.sample_interpolation = int(options.interpolation)
        self.correlation_interpolation = options.correlation_interpolation
        self.lo_offset = float(options.lo_offset)
        self.samples_to_receive_calibration = int(options.num_samples_calibration)
        self.frequency_calibration = float(options.frequency_calibration)
        self.bw_calibration = float(options.bandwidth_calibration)
        self.lo_offset_calibration = float(options.lo_offset_calibration)
        self.gain = float(options.gain)
        self.gain_calibration = float(options.gain_calibration)
        self.antenna = options.antenna
        self.auto_calibrate = options.auto_calibrate 
        self.acquisition_time = float(options.acquisition_time)

        self.receivers = {}
        self.ref_receiver = ""
        self.guis = {}
        
        self.grid_based = {"resolution":10,"num_samples":self.samples_to_receive * self.sample_interpolation}
        self.grid_based_active = False

        self.estimated_positions_history = []
        self.delay_history = []
        self.delay_calibration = []
        self.delay_auto_calibration = []
        self.calibration_loop_delays = []
        self.calibration_average = 1
        
        # postprocessing
        self.location_average_length = 3
        self.target_dynamic = 0.22
        self.max_acc = 5
        self.measurement_noise = 10
        self.reference_selections = ["Manual","Max-signal-power","Min-signal-power","Min-DOP"]
        self.filtering_types = ["No filtering","Moving average","Kalman filter"]
        self.motion_models = ["maneuvering","simple"]
        self.reference_selection = "Manual"
        self.filtering_type = "No filtering"
        self.motion_model = "maneuvering"
        self.init_settings_kalman = dict()
        
        self.processing = False
        
        self.record_results = False
        self.record_samples = False
        self.recording_results = False
        self.recording_samples = False
        self.results_file = ""
        if not os.path.exists("../log"):
            os.makedirs("../log")
        self.results = {}
        self.results_selfloc = {}
        self.run_loop = False
        self.localizing = False
        self.ntp_sync = False
        self.calibrating = False
        
        # selfloc
        self.sample_history = []
        self.pos_selfloc = []
        self.alpha = 1.0
        self.init_stress = 10.0
        self.max_it = 99 

        # Anchoring:
        self.num_anchor = 0
        self.num_anchors = 3
        self.anchoring = False
        self.anchor_loop = False
        self.anchor_average = 3
        self.anchor_loop_delays = []
        self.anchor_positions = []
        self.anchor_gt_positions = []
        self.anchor_loop_delay_history = []
        
        self.map_type = "Online"
        self.map_file = "../maps/map.png"
        self.coordinates_type = "Geographical"
        #kalman filtering
        self.xk_1_chan = np.array([])
        self.xk_1_grid = np.array([])
        self.Pk_1_chan = np.array([])
        self.Pk_1_grid = np.array([])
        self.init_settings_kalman["model"] = self.motion_model
        self.init_settings_kalman["delta_t"] = self.acquisition_time
        self.init_settings_kalman["noise_factor"] = self.target_dynamic 
        self.init_settings_kalman["filter_receivers"] = False
        self.init_settings_kalman["noise_var_x"] = self.measurement_noise
        self.init_settings_kalman["noise_var_y"] = self.measurement_noise
        self.init_settings_kalman["max_acceleration"] = self.max_acc


        # Parameters for self-localization:
        self.self_localization = False
        self.transmitter = -1 # No sensor is transmitting
        self.transmitter_debug = 0
        self.sample_average = int(options.selfloc_average_length)
        self.transmitter_history = []
        self.timestamp_history = []
        #[[[[]<-average]<-cnt_l]<-cnt_k]<-cnt_j
        self.delay_tensor = [[[[]]]]
        self.cnt_j = 0
        self.cnt_average = 0
        self.tx_gain = 50 #in db

        # ICT + surroundings
        #self.bbox = 6.0580,50.7775,6.0690,50.7810
        self.bbox = 6.0606,50.77819,6.06481,50.77967

        # campus hoern:
        #self.bbox = 6.05938, 50.77728, 6.06646, 50.78075
        # closeup of meadow in front of ict cubes
        #self.bbox = (6.06210,50.77865,6.06306,50.77923)
        # Football court
        #self.bbox = 6.06429,50.77697,6.07271,50.78033
        # ICT indoor
        #self.bbox = 6.061698496341705,50.77914404797512,6.063739657402039,50.77976138469289
        #self.bbox = 6.061738267169996,50.779093354299285,6.063693919000911,50.77980828706738
        # UPB Campus
        #self.bbox = -75.59124,6.24113,-75.58851,6.24261
        # Hack for Hofburg Vienna
        #self.bbox = 16.366270782839383, 48.20741002023011, 16.367042, 48.2075765
        self.init_map()


        # ZeroMQ
        self.probe_manager = probe_manager_local.probe_manager()
        self.rpc_manager = rpc_manager_local.rpc_manager()
        self.rpc_manager.set_reply_socket(rpc_adr)
        self.rpc_manager.add_interface("register_gui",self.register_gui)
        self.rpc_manager.add_interface("sync_position",self.sync_position)
        self.rpc_manager.add_interface("register_receiver",self.register_receiver)
        self.rpc_manager.add_interface("forward_chat",self.forward_chat)
        self.rpc_manager.add_interface("switch_transmitter",self.switch_transmitter_debug)
        self.rpc_manager.add_interface("stop_transmitter",self.stop_transmitter)
        self.rpc_manager.add_interface("update_receivers",self.update_receivers)
        self.rpc_manager.add_interface("get_gui_gps_position",self.get_gui_gps_position)
        self.rpc_manager.add_interface("localize",self.localize)
        self.rpc_manager.add_interface("localize_loop",self.localize_loop)
        self.rpc_manager.add_interface("calibrate",self.calibrate)
        self.rpc_manager.add_interface("remove_calibration",self.remove_calibration)
        self.rpc_manager.add_interface("calibration_loop",self.calibration_loop)
        self.rpc_manager.add_interface("set_calibration_average",self.set_calibration_average)
        self.rpc_manager.add_interface("set_num_anchors",self.set_num_anchors)
        self.rpc_manager.add_interface("set_anchor_average",self.set_anchor_average)
        self.rpc_manager.add_interface("set_tx_gain",self.set_tx_gain)
        self.rpc_manager.add_interface("set_sample_average",self.set_sample_average)
        self.rpc_manager.add_interface("set_location_average_length",self.set_location_average_length)
        self.rpc_manager.add_interface("set_target_dynamic",self.set_target_dynamic)
        self.rpc_manager.add_interface("set_init_stress",self.set_init_stress)
        self.rpc_manager.add_interface("set_max_it",self.set_max_it)
        self.rpc_manager.add_interface("set_alpha",self.set_alpha)
        self.rpc_manager.add_interface("set_max_acc",self.set_max_acc)
        self.rpc_manager.add_interface("set_measurement_noise",self.set_measurement_noise)
        self.rpc_manager.add_interface("start_correlation",self.start_correlation)
        self.rpc_manager.add_interface("start_selfloc_loop",self.start_selfloc_loop)
        self.rpc_manager.add_interface("start_correlation_loop",self.start_correlation_loop)
        self.rpc_manager.add_interface("stop_loop",self.stop_loop)
        self.rpc_manager.add_interface("set_frequency",self.set_frequency)
        self.rpc_manager.add_interface("set_lo_offset",self.set_lo_offset)
        self.rpc_manager.add_interface("set_samples_to_receive",self.set_samples_to_receive)
        self.rpc_manager.add_interface("set_samp_rate",self.set_samp_rate)
        self.rpc_manager.add_interface("set_bw",self.set_bw)
        self.rpc_manager.add_interface("set_interpolation",self.set_interpolation)
        self.rpc_manager.add_interface("set_correlation_interpolation",self.set_correlation_interpolation)
        self.rpc_manager.add_interface("set_frequency_calibration",self.set_frequency_calibration)
        self.rpc_manager.add_interface("set_lo_offset_calibration",self.set_lo_offset_calibration)
        self.rpc_manager.add_interface("set_samples_to_receive_calibration",self.set_samples_to_receive_calibration)
        self.rpc_manager.add_interface("set_bw_calibration",self.set_bw_calibration)
        self.rpc_manager.add_interface("set_gain",self.set_gain)
        self.rpc_manager.add_interface("set_gain_calibration",self.set_gain_calibration)
        self.rpc_manager.add_interface("set_antenna",self.set_antenna)
        self.rpc_manager.add_interface("set_selected_position",self.set_selected_position)
        self.rpc_manager.add_interface("set_ref_receiver",self.set_ref_receiver)
        self.rpc_manager.add_interface("set_reference_selection",self.set_reference_selection)
        self.rpc_manager.add_interface("set_filtering_type",self.set_filtering_type)
        self.rpc_manager.add_interface("set_motion_model",self.set_motion_model)
        self.rpc_manager.add_interface("set_map_type",self.set_map_type)
        self.rpc_manager.add_interface("set_map_file",self.set_map_file)
        self.rpc_manager.add_interface("set_coordinates_type",self.set_coordinates_type)
        self.rpc_manager.add_interface("set_auto_calibrate",self.set_auto_calibrate)
        self.rpc_manager.add_interface("set_TDOA_grid_based_resolution",self.set_TDOA_grid_based_resolution)
        self.rpc_manager.add_interface("set_TDOA_grid_based_num_samples",self.set_TDOA_grid_based_num_samples)
        self.rpc_manager.add_interface("set_record_results",self.set_record_results)
        self.rpc_manager.add_interface("set_record_samples",self.set_record_samples)
        self.rpc_manager.add_interface("sync_success",self.sync_ntp)
        self.rpc_manager.add_interface("set_bbox",self.set_bbox)
        self.rpc_manager.add_interface("set_acquisition_time",self.set_acquisition_time)
        self.rpc_manager.add_interface("set_grid_based_active",self.set_grid_based_active)
        self.rpc_manager.add_interface("program_gps_receiver",self.program_gps_receiver)
        self.rpc_manager.add_interface("set_anchor_gt_position",self.set_anchor_gt_position)
        self.rpc_manager.add_interface("start_anchoring_loop",self.start_anchoring_loop)
        self.rpc_manager.add_interface("stop_selfloc", self.stop_selfloc)
        self.rpc_manager.start_watcher()

        self.probe_manager_lock = threading.Lock()
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

        if self.coordinates_type == "Geographical":
            self.basemap = Basemap(llcrnrlon=self.bbox[0], llcrnrlat=self.bbox[1],
                          urcrnrlon=self.bbox[2], urcrnrlat=self.bbox[3],
                          projection='tmerc', lon_0=lon_0, lat_0=lat_0)
        else:
            self.basemap = Basemap(width=self.bbox[2], height=self.bbox[3],
                          lon_0=0,lat_0=0,
                          projection='tmerc')

        self.coordinates_calibration = (40.7, 2.5)

    def calibrate(self, coordinates, delays=None):
        # calculate offset calibration
        if self.results is not None and delays is None:
            if len(self.calibration_loop_delays) > 0:
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
                        elif ref_receiver.selected_position == "selfloc" :
                            pos_ref = ref_receiver.coordinates_gps
                        else:
                            pos_receiver = receiver.coordinates_gps
                        d_ref = np.linalg.norm(np.array(coordinates)-pos_ref)
                        d_receiver = np.linalg.norm(np.array(coordinates)-pos_receiver)
                        delay_true = (d_receiver-d_ref) * self.samp_rate * self.sample_interpolation / 299700000
                        print("True delay: ",delay_true)
                        print(index_delay,len(self.delay_calibration),len(self.calibration_loop_delays))
                        print(self.calibration_loop_delays)
                        # int correct?!
                        if len(self.delay_calibration) < len(self.calibration_loop_delays[-1]):
                            self.delay_calibration.append(int(np.floor(delay_true)-np.array(self.calibration_loop_delays).mean(0)[index_delay]))
                        else:
                            self.delay_calibration[index_delay] = int(np.floor(delay_true)-np.array(self.calibration_loop_delays).mean(0)[index_delay])
                        index_delay += 1
            print ("Delay calibration: ", self.delay_calibration)
            self.calibrating = False
            for gui in self.guis.values():
                gui.rpc_manager.request("calibration_status",[True])
        # calculate autocalibration
        else:
            ref_receiver = self.receivers[self.ref_receiver]
            if ref_receiver.selected_position == "manual":
                pos_ref = ref_receiver.coordinates
            elif ref_receiver.selected_position == "GPS" :
                pos_ref = ref_receiver.coordinates_gps
            else: 
                pos_ref = ref_receiver.coordinates_selfloc
            index_delay_auto = 0
            for i in range(0,len(self.receivers)):
                if not self.ref_receiver == self.receivers.keys()[i]:
                    receiver = self.receivers.values()[i]
                    if receiver.selected_position == "manual":
                        pos_receiver = receiver.coordinates
                    elif receiver.selected_position == "GPS":
                        pos_receiver = receiver.coordinates_gps
                    else:
                        pos_receiver = receiver.coordinates_selfloc
                    d_ref = np.linalg.norm(np.array(coordinates)-pos_ref)
                    d_receiver = np.linalg.norm(np.array(coordinates)-pos_receiver)
                    delay_true = (d_receiver-d_ref) * self.samp_rate * self.sample_interpolation / 299700000
                    print("True delay: ",delay_true)
                    if len(self.delay_auto_calibration) < len(delays):
                        self.delay_auto_calibration.append(int(np.floor(delay_true)-delays[index_delay_auto]))
                    else:
                        self.delay_auto_calibration[index_delay_auto] = int(np.floor(delay_true)-delays[index_delay_auto])
                    index_delay_auto += 1
            print ("Delay auto calibration: ", self.delay_auto_calibration)

    def set_location_average_length(self, location_average_length):
        self.location_average_length = location_average_length
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_location_average_length",[location_average_length])
            
    def set_max_acc(self, max_acc):
        self.max_acc = max_acc
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_max_acc",[max_acc])
            
    def set_target_dynamic(self, target_dynamic):
        self.target_dynamic = target_dynamic
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_target_dynamic",[target_dynamic])

    def set_alpha(self, alpha):
        self.alpha = alpha
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_alpha",[alpha])

    def set_init_stress(self, init_stress):
        self.init_stress = init_stress
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_init_stress",[init_stress])

    def set_max_it(self, max_it):
        self.max_it = max_it
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_max_it",[max_it])
            
    def set_measurement_noise(self, measurement_noise):
        self.measurement_noise = measurement_noise
        for receiver in self.receivers.values():
            receiver.measurement_noise = measurement_noise
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_measurement_noise",[measurement_noise])

    def set_calibration_average(self, calibration_average):
        self.calibration_average = calibration_average
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_calibration_average",[calibration_average])

    def set_sample_average(self, sample_average):
        self.sample_average = sample_average
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_sample_average",[sample_average])

    def set_anchor_average(self, anchor_average):
        self.anchor_average = anchor_average
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_anchor_average",[anchor_average])

    def set_tx_gain(self, tx_gain):
        self.tx_gain = tx_gain
        for receiver in self.receivers.values():
            receiver.set_tx_gain(tx_gain)
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_tx_gain",[tx_gain])
            #TODO:new construction of usrp sink


    def set_num_anchors(self, num_anchors):
        self.num_anchors = num_anchors
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_num_anchors",[num_anchors])

    def calibration_loop(self, freq, lo_offset, samples_to_receive, acquisitions):
        if len(self.calibration_loop_delays) > 0:
            self.remove_calibration()
            return False
        else:
            if len(self.receivers) > 1:
                self.delay_calibration = []
                self.calibrating = True
                self.run_loop = True
                self.start_receivers(acquisitions)
                for gui in self.guis.values():
                    gui.rpc_manager.request("calibration_status",[False])
                    gui.rpc_manager.request("calibration_loop",[True])
                return True

    def remove_calibration(self):
        self.delay_calibration = []
        self.delay_auto_calibration = []
        self.calibrating = False
        self.calibration_loop_delays = []
        self.run_loop = False
        for gui in self.guis.values():
            gui.rpc_manager.request("calibration_status",[False])

    def forward_chat(self, chat):
        for gui in self.guis.values():
            gui.rpc_manager.request("new_chat",[chat])

    def sync_position(self, serial, coordinates):
        print("Position set for serial:",serial,coordinates)
        self.receivers[serial].coordinates = coordinates
        for gui in self.guis.values():
            gui.rpc_manager.request("sync_position",[serial, coordinates])
        self.get_gui_gps_position(serial)

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

    def register_gui(self, ip_addr, hostname, id_gui, first_time):
        was_not_registered = False
        gui_serial = hostname + str(id_gui)
        if not self.guis.has_key(gui_serial):
            gui = gui_interface("tcp://" + ip_addr + ":" + str(7775 + id_gui), hostname)
            self.guis[gui_serial] = gui
            was_not_registered = True
        else:
            gui = self.guis[gui_serial]
        if first_time or was_not_registered:
            for serial in self.receivers:
                # request registration of each receiver in gui
                gui.rpc_manager.request("register_receiver",[serial, self.receivers[serial].gain, self.receivers[serial].antenna, self.receivers[serial].gain_calibration])
                gui.rpc_manager.request("sync_position",[serial, self.receivers[serial].coordinates])
                gui.rpc_manager.request("set_gps_position",[serial, self.receivers[serial].coordinates_gps])
            gui.rpc_manager.request("set_gui_frequency",[self.frequency])
            gui.rpc_manager.request("set_gui_lo_offset",[self.lo_offset])
            gui.rpc_manager.request("set_gui_samples_to_receive",[self.samples_to_receive])
            gui.rpc_manager.request("set_gui_bw",[self.bw])
            gui.rpc_manager.request("set_gui_interpolation",[self.sample_interpolation])
            gui.rpc_manager.request("set_gui_correlation_interpolation",[self.correlation_interpolation])
            gui.rpc_manager.request("set_gui_samp_rate",[self.samp_rate])
            gui.rpc_manager.request("set_gui_frequency_calibration",[self.frequency_calibration])
            gui.rpc_manager.request("set_gui_lo_offset_calibration",[self.lo_offset_calibration])
            gui.rpc_manager.request("set_gui_samples_to_receive_calibration",[self.samples_to_receive_calibration])
            gui.rpc_manager.request("set_gui_bw_calibration",[self.bw_calibration])
            gui.rpc_manager.request("set_gui_auto_calibrate",[self.auto_calibrate])
            gui.rpc_manager.request("set_gui_calibration_average",[self.calibration_average])
            gui.rpc_manager.request("set_gui_sample_average",[self.sample_average])
            gui.rpc_manager.request("set_gui_anchor_average",[self.anchor_average])
            gui.rpc_manager.request("set_gui_num_anchors",[self.num_anchors])
            gui.rpc_manager.request("set_gui_tx_gain",[self.tx_gain])
            gui.rpc_manager.request("set_gui_TDOA_grid_based_resolution",[self.grid_based["resolution"]])
            gui.rpc_manager.request("set_gui_TDOA_grid_based_num_samples",[self.grid_based["num_samples"]])
            gui.rpc_manager.request("set_gui_record_results",[self.record_results])
            gui.rpc_manager.request("set_gui_record_samples",[self.record_samples])
            gui.rpc_manager.request("set_gui_filtering_types",[self.filtering_types])
            gui.rpc_manager.request("set_gui_reference_selections",[self.reference_selections])
            gui.rpc_manager.request("set_gui_filtering_type",[self.filtering_type])
            gui.rpc_manager.request("set_gui_reference_selection",[self.reference_selection])
            gui.rpc_manager.request("set_gui_motion_models",[self.motion_models])
            gui.rpc_manager.request("set_gui_motion_model",[self.motion_model])
            gui.rpc_manager.request("set_gui_map_type",[self.map_type])
            gui.rpc_manager.request("set_gui_map_file",[self.map_file])
            gui.rpc_manager.request("set_gui_coordinates_type",[self.coordinates_type])
            gui.rpc_manager.request("set_gui_location_average_length",[self.location_average_length])
            gui.rpc_manager.request("set_gui_acquisition_time",[self.acquisition_time])
            gui.rpc_manager.request("set_gui_grid_based_active",[self.grid_based_active])
            gui.rpc_manager.request("set_gui_target_dynamic",[self.target_dynamic])
            gui.rpc_manager.request("set_gui_max_it",[self.max_it])
            gui.rpc_manager.request("set_gui_alpha",[self.alpha])
            gui.rpc_manager.request("set_gui_init_stress",[self.init_stress])
            gui.rpc_manager.request("set_gui_max_acc",[self.max_acc])
            gui.rpc_manager.request("set_gui_measurement_noise",[self.measurement_noise])



            for gui in self.guis.values():
                for serial in self.guis:
                    # request registration in each gui
                    gui.rpc_manager.request("register_another_gui",[serial])
            print(gui_serial, "registered")
        return [self.bbox, self.map_type, self.map_file, self.coordinates_type]

    def register_receiver(self, hostname, serial, id_rx, gps, first_time, coordinates):
        was_not_registered = False
        rpc_adr = "tcp://" + hostname + ":" + str(6665 + id_rx)
        probe_adr = "tcp://" + hostname + ":" + str(5555 + id_rx)
        if not self.receivers.has_key(serial):
            # create new receiver in fusion center
            self.receivers[serial] = receiver_interface.receiver_interface(rpc_adr, probe_adr, serial)
            was_not_registered = True
        elif first_time:
            # receiver might have restarted, so remove and add again
            self.probe_manager_lock.acquire()
            self.probe_manager.remove_socket(self.receivers[serial].probe_address)
            self.probe_manager_lock.release()
            receiver = self.receivers.pop(serial)
            self.receivers[serial] = receiver_interface.receiver_interface(rpc_adr, probe_adr, serial)
        if first_time or was_not_registered:
            # set parameters of receiver in fusion center
            receiver = self.receivers[serial]
            receiver.set_gain(self.gain)
            receiver.set_antenna(self.antenna)
            receiver.set_bw(self.bw)
            receiver.interpolation = self.sample_interpolation
            receiver.correlation_interpolation = self.correlation_interpolation
            receiver.set_samp_rate(self.samp_rate)
            print(receiver.frequency,self.frequency)
            receiver.frequency = self.frequency
            receiver.lo_offset = self.lo_offset
            receiver.samples_to_receive = self.samples_to_receive
            receiver.gain_calibration = self.gain_calibration
            receiver.bw_calibration = self.bw_calibration
            print(receiver.frequency_calibration,self.frequency_calibration)
            receiver.frequency_calibration = self.frequency_calibration
            receiver.lo_offset_calibration = self.lo_offset_calibration
            receiver.samples_to_receive_calibration = self.samples_to_receive_calibration
            receiver.gps = gps
            receiver.auto_calibrate = self.auto_calibrate
            receiver.measurement_noise = self.measurement_noise
            self.probe_manager_lock.acquire()
            self.probe_manager.add_socket(serial, receiver.probe_address, 'complex64', receiver.receive_samples)
            self.probe_manager_lock.release()
            for gui in self.guis.values():
                # request registration in each gui
                gui.rpc_manager.request("register_receiver",[serial, receiver.gain, receiver.antenna, receiver.gain_calibration])
            self.set_ref_receiver(serial)
            self.sync_position(serial,coordinates)
            self.update_receivers()
            print(serial, "registered")

    def start_receivers(self, acquisitions=0):
        # reception 1 second in the future at full second
        if not self.processing:
            time_to_receive = np.ceil(time.time()) + 1
            self.init_kalman = True
            for receiver in self.receivers.values():
                threading.Thread(target = self.start_receiver, args = (receiver, time_to_receive, acquisitions)).start()


    def start_receiver(self, receiver, time_to_receive, acquisitions):
            receiver.samples = np.array([])
            receiver.samples_calibration = np.array([])
            receiver.first_packet = True
            receiver.reception_complete = False
            receiver.set_run_loop(self.run_loop)
            if self.ntp_sync:
                receiver.request_samples(time_to_receive, acquisitions, self.acquisition_time)
            else:
                receiver.request_samples(None, acquisitions, self.acquisition_time)

    def update_receivers(self):
        for receiver in self.receivers.values():
            receiver.sync_time()

    def start_correlation(self, freq, lo_offset, samples_to_receive):
        self.start_receivers()
        

    def start_correlation_loop(self, freq, lo_offset, samples_to_receive, acquisitions = 0):
        self.delay_history = []
        self.estimated_positions_history = []
        self.recording_results = self.record_results
        if self.filtering_type == "Kalman filter" :
            self.init_settings_kalman["model"] = self.motion_model
            self.init_settings_kalman["delta_t"] = self.acquisition_time
            self.init_settings_kalman["noise_factor"] = self.target_dynamic 
            self.init_settings_kalman["filter_receivers"] = False
            self.init_settings_kalman["noise_var_x"] = self.measurement_noise
            self.init_settings_kalman["noise_var_y"] = self.measurement_noise
            self.init_settings_kalman["max_acceleration"]= self.max_acc
        self.results_file = "../log/results_" + time.strftime("%d_%m_%y-%H_%M_%S") + ".txt"
        self.recording_samples = self.record_samples
        self.samples_file = "../log/samples_" + time.strftime("%d_%m_%y-%H_%M_%S") + ".txt"
        if self.recording_results:
            print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
            print("rx_time,delays(1-2,1-3,1-X...),delays_calibration(1-2,1-3,1-X...),delays_auto_calibration(1-2,1-3,1-X...),sampling_rate,frequency,frequency_calibration,calibration_position,interpolation,bandwidth,samples_to_receive,lo_offset,bbox,receivers_positions,selected_positions,receivers_gps,receivers_antenna,receivers_gain,estimated_positions,index_ref_receiver,auto_calibrate,acquisition_time,kalman_states,init_settings_kalman, reference_selection", file=open(self.results_file,"a"))
            print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
        self.run_loop = True
        self.start_receivers(acquisitions)

    def localize(self, freq, lo_offset, samples_to_receive):
        if len(self.receivers) > 2:
            # check if receiver coordinates have been set:
            for receiver in self.receivers.values():
                print(self.basemap(receiver.coordinates[0],receiver.coordinates[1], inverse = True))
            if self.check_receiver_positions(self.receivers):
                self.localizing = True
                self.start_receivers()
            else:
                print ("Set receiver positions at first!")

    def localize_loop(self, freq, lo_offset, samples_to_receive, acquisitions = 0):
        self.delay_history = []
        self.estimated_positions_history = []
        if len(self.receivers) > 2:
            if self.check_receiver_positions(self.receivers):
                self.localizing = True
                self.recording_results = self.record_results
                self.results_file = "../log/results_" + time.strftime("%d_%m_%y-%H_%M_%S") + ".txt"
                self.recording_samples = self.record_samples
                self.samples_file = "../log/samples_" + time.strftime("%d_%m_%y-%H_%M_%S") + ".txt"
                if self.recording_results:
                    print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
                    print("rx_time,delays(1-2,1-3,1-X...),delays_calibration(1-2,1-3,1-X...),delays_auto_calibration(1-2,1-3,1-X...),sampling_rate,frequency,frequency_calibration,calibration_position,interpolation,bandwidth,samples_to_receive,lo_offset,bbox,receivers_positions,selected_positions,receivers_gps,receivers_antenna,receivers_gain,estimated_positions,index_ref_receiver,auto_calibrate,acquisition_time,kalman_states,init_settings_kalman, reference_selection", file=open(self.results_file,"a"))
                    print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
                self.run_loop = True
                self.start_receivers(acquisitions)
            else:
                print ("Set receiver positions at first!")

    def check_receiver_positions(self,receivers):
        return all((receiver.selected_position == "manual" and all(coordinate > 0 for coordinate in receiver.coordinates ))
                    or(receiver.selected_position == "selfloc" and all(coordinate > 0 for coordinate in receiver.coordinates_selfloc ))
                    or(receiver.selected_position == "GPS" and all(coordinate > 0 for coordinate in receiver.coordinates_gps)) for receiver in receivers.values() )

    def start_selfloc_loop(self):
        # base structure for obtaining the delays required to perform the differential mds self localization algorithm
        print("selfloc_loop")
        self.delay_history = []
        self.estimated_positions_history = []
        self.stop_transmitter()
        self.recording_results = self.record_results
        self.recording_samples = self.record_samples
        self.samples_file = "../log/samples_selfloc_" + time.strftime("%d_%m_%y-%H_%M_%S") + ".txt"
        #if len(self.receivers) > 3:
        # number of receptions required for the whole process
        #acquisitions = len(self.receivers)*self.sample_average
        acquisitions = len(self.receivers)*self.sample_average
        self.transmitter_history = []
        self.timestamp_history = []
        self.cnt_j = 0
        self.cnt_average = 0
        self.delay_tensor = np.ndarray(shape=(len(self.receivers),len(self.receivers),len(self.receivers),self.sample_average))
        self.switch_transmitter(self.cnt_j)
        self.self_localization = True
        self.run_loop = True
        self.anchor_interrupt = False
        self.anchor_positions = []
        self.anchor_gt_positions = []
        self.start_receivers(acquisitions)

                
    def stop_loop(self):
        self.run_loop = False
        self.anchor_loop = False
        self.recording_results = False
        self.recording_samples = False
        self.self_localization = False
        self.estimated_positions_history = []
        self.localizing = False
        self.processing = False

        
        for receiver in self.receivers.values():
            threading.Thread(target = receiver.set_run_loop, args = [self.run_loop]).start()

    def set_frequency(self, frequency):
        #print("in fc f", frequency)
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
            receiver.set_samp_rate(samp_rate)
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_samp_rate",[samp_rate])

    def set_bw(self, bw):
        self.bw = bw
        for receiver in self.receivers.values():
            receiver.set_bw(bw)
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_bw",[bw])

    def set_interpolation(self, interpolation):
        self.sample_interpolation = interpolation
        for receiver in self.receivers.values():
            receiver.interpolation = interpolation
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_interpolation",[interpolation])

    def set_correlation_interpolation(self, correlation_interpolation):
        self.correlation_interpolation = correlation_interpolation
        for receiver in self.receivers.values():
            receiver.correlation_interpolation = correlation_interpolation
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_correlation_interpolation",[correlation_interpolation])

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
        self.receivers[serial].set_gain(gain)
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
            
    def set_reference_selection(self, reference_selection):
        self.reference_selection = reference_selection
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_reference_selection",[reference_selection])        
    
    def set_filtering_type(self, filtering_type):
        self.filtering_type = filtering_type
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_filtering_type",[filtering_type])
            
    def set_motion_model(self, motion_model):
        self.motion_model = motion_model
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_motion_model",[motion_model])

    def set_map_type(self, map_type):
        self.map_type = map_type
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_map_type",[map_type])

    def set_map_file(self, map_file):
        self.map_file = map_file
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_map_file",[map_file])

    def set_coordinates_type(self, coordinates_type):
        self.coordinates_type = coordinates_type
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_coordinates_type",[coordinates_type])

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
        num_samples = min(num_samples,self.samples_to_receive * self.sample_interpolation)
        self.grid_based["num_samples"] = num_samples
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_TDOA_grid_based_num_samples",[num_samples])

    def set_record_results(self, record_results):
        self.record_results = record_results
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_record_results",[record_results])

    def set_record_samples(self, record_samples):
        self.record_samples = record_samples
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_record_samples",[record_samples])

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
            gui.rpc_manager.request("init_map",[bbox, self.map_type, self.map_file, self.coordinates_type])

    def set_acquisition_time(self, acquisition_time):
        self.acquisition_time = acquisition_time
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_acquisition_time",[acquisition_time])

    def set_grid_based_active(self, grid_based_active):
        self.grid_based_active = grid_based_active
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_grid_based_active",[grid_based_active])

    def program_gps_receiver(self, serial, latitude, longitude, altitude):
        self.receivers[serial].program_receiver_position(latitude, longitude, altitude)

    def switch_transmitter_debug(self):
        if self.transmitter_debug == len(self.receivers):
            self.transmitter_debug = 0
        self.switch_transmitter(self.transmitter_debug)
        
    def switch_transmitter(self, idx_new): 
        #cmmnt: maybe some wait required here
        if self.transmitter != -1:
            self.receivers.values()[self.transmitter].stop_transmitter()
        self.receivers.values()[idx_new].start_transmitter()
        self.transmitter = idx_new

    def stop_transmitter(self):
        if self.transmitter != -1:
            self.receivers.values()[self.transmitter].stop_transmitter()
        self.transmitter = -1

    def start_anchoring(self):
        # immediatly interrupt if all positions required are optained
        if (len(self.anchor_gt_positions) == self.num_anchors) and (len(self.anchor_positions) == self.num_anchors):
            self.anchoring = False
            return True
        else:
            if self.anchor_interrupt:
                return True
            if not self.anchoring:
                if len(self.anchor_gt_positions) == 0:
                    for gui in self.guis.values():
                        gui.rpc_manager.request("start_anchoring")
                return False
            else:
                return False
            
    def start_anchoring_loop(self, num_anchor):
        self.num_anchor = num_anchor
        print("num_anchor: ", num_anchor)
        #tbd build up similar to calibration loop?
        #acquire delay/position similar to calibration_loop_delays/calibration_loop
        self.anchor_loop_delays = []
        self.anchoring = True
        self.anchor_loop = True
        self.recording_results = False
        self.run_loop = True
        self.start_receivers(self.anchor_average)

    # rename/even necessary?!
    def set_anchor_position(self,position):
        self.anchor_position = position
        for gui in self.guis.values():
            gui.rpc_manager.request("set_anchor_position", [self.anchor_position])

    def set_anchor_gt_position(self,pos):
        self.anchor_gt_positions.append(np.array(pos))

    def stop_selfloc_loop(self):
        self.cnt_j = 0
        self.self_localization = False
        self.stop_transmitter()
        self.stop_loop()
        # continue logging samples! 
        self.recording_samples = self.record_samples
        print("delay tensor recordings complete")

    def stop_selfloc(self):
        self.cnt_j = 0
        self.self_localization = False
        self.stop_transmitter()
        self.stop_loop()
        # continue logging samples!
        self.processing = False 
        self.recording_samples = False
        self.anchor_loop = False
        for receiver in self.receivers.values():
            receiver.reset_receiver()
        self.anchor_interrupt = True
        print("self localization process stopped")

    def evaluate_selfloc(self,receivers):
        print("cnt_smpl: ",self.cnt_smpl_log)
        D = np.ndarray(shape=(len(receivers),len(receivers),len(receivers)))
        sum_square_tdoa = 0
        for j in range(len(receivers)):
            for l in range(len(receivers)):
                for k in range(len(receivers)):
                    # average distance differences
                    tdoa = sum(self.delay_tensor[j,l,k]) / self.sample_average / self.samp_rate * 299700000.0
                    sum_square_tdoa += tdoa**2
                    D[j,l,k] = tdoa
        pos_selfloc = None
        stress_list = [self.init_stress]
        self.pos_selfloc, stress_list = mds_self_tdoa.selfloc(D,self.basemap(self.bbox[2],self.bbox[3]), sum_square_tdoa, pos_selfloc, self.max_it, self.alpha, stress_list)
        print(stress_list)
        if self.record_results:
            print(self.recording_results)
            receivers_positions, selected_positions, receivers_gps, receivers_antenna, receivers_gain = helpers.build_results_strings(receivers)
            header =  "["  + str(self.samp_rate) + "," + str(self.frequency) + "," + str(self.frequency_calibration) + "," \
            + str(self.coordinates_calibration) + "," + str(self.sample_interpolation) + "," \
            + str(self.bw) + "," + str(self.samples_to_receive) + "," + str(self.lo_offset) + "," \
            + str(self.bbox) + "," + receivers_positions + "," + selected_positions + "," \
            + receivers_gps + "," + receivers_antenna + "," + receivers_gain + "," + str(self.sample_average) + "," + str(self.num_anchors) + "," + str(self.anchor_average) + "," + str(receivers.keys().index(self.ref_receiver)) + "," + str(self.alpha) + "]\n" 
        for j, receiver in enumerate(receivers.values()):
                receiver.coordinates_selfloc = self.pos_selfloc[j]
                #FIXME logs
                receiver.selected_position = "selfloc"
        # Wait for gui here
        # Split!

        anchoring_complete = False
        while anchoring_complete == False:
            anchoring_complete = self.start_anchoring()
            time.sleep(0.5)
        if self.anchor_interrupt:
            self.anchor_interrupt = False
            return
        print("anchoring done")

        # Split!

        self.anchor_gt_positions = np.array(self.anchor_gt_positions)
        self.anchor_positions = np.array(self.anchor_positions)
        d, coordinates_procrustes, tform = procrustes(self.anchor_gt_positions, self.anchor_positions, scaling = False)
        print(coordinates_procrustes)
        reflection = np.linalg.det(tform["rotation"])
        self.pos_selfloc_procrustes =  np.dot(self.pos_selfloc,tform["rotation"]) + tform["translation"]
        print(self.pos_selfloc_procrustes)
        for j, receiver in enumerate(receivers.values()):
                receiver.coordinates_selfloc = self.pos_selfloc_procrustes[j]
                receiver.selected_position = "selfloc"
        for gui in self.guis.values():
            gui.rpc_manager.request("sync_position_selfloc",[self.pos_selfloc_procrustes[:,0],self.pos_selfloc_procrustes[:,1]])
        time.sleep(0.05)

        # Split!

        if self.recording_results:
            results_file_selfloc = "../log/results_selfloc_" + time.strftime("%d_%m_%y-%H_%M_%S") + ".txt"
            fi = open(results_file_selfloc,'w')
            fi.write("##########################################################################################################################################################################################\n")
            fi.write("rx_time,sampling_rate,frequency,frequency_calibration,calibration_position,interpolation,bandwidth,samples_to_receive,lo_offset,bbox,receivers_positions,selected_positions,receivers_gps,receivers_antenna,receivers_gain,sample_average,num_anchors,anchor_average,index_ref_receiver,alpha\n")
            fi.write("##########################################################################################################################################################################################\n")
            fi.write(header)
            fi.write(str(self.transmitter_history) + "\n")
            fi.write(str(self.timestamp_history) + "\n")
            fi.write(str(self.delay_tensor.tolist()) + "\n")
            fi.write(str(D.tolist()) + "\n")
            fi.write(str(self.anchor_loop_delay_history) + "\n")
            fi.write(str(self.anchor_positions.tolist()) + "\n")
            fi.write(str(coordinates_procrustes.tolist()) + "\n")
            fi.write(str(self.anchor_gt_positions.tolist()) + "\n")
            fi.write(str(self.pos_selfloc.tolist()) + "\n")
            fi.write(str(self.pos_selfloc_procrustes.tolist()) + "\n")
            #fi.write(str(tform.keys()) + "\n")
            tform["rotation"] = tform["rotation"].tolist()
            tform["translation"] = tform["translation"].tolist()
            #fi.write(str(tform.values()) + "\n")
            fi.write(str(stress_list) + "\n")
            fi.close()
        if self.recording_samples: 
            f_s = open(self.samples_file,"a")
            for entry in self.sample_history:
                pprint.pprint(str(entry) + "\n",f_s,width=9000)
            f_s.close() 


    def process_results(self, receivers, delay_auto_calibration):
        # check if timestamps are equal for all the receivers
        times_target = []
        times_calibration = []
        H = np.array([])
        x_cov=0
        y_cov=0
        last_time_target = receivers.values()[0].tags["rx_time"]
        if self.auto_calibrate:
            last_time_calibration = receivers.values()[0].tags_calibration["rx_time"]

        for i in range(1,len(receivers)):

            if not (last_time_target == receivers.values()[i].tags["rx_time"]):
                print("Error: target timestamps do not match")
                for i in range(0,len(receivers)):
                    print (i,receivers.values()[i].serial,receivers.values()[i].tags)
                for receiver in self.receivers.values():
                    receiver.reset_receiver()
                self.processing = False
                return
            if self.auto_calibrate:
                if not (last_time_calibration == receivers.values()[i].tags_calibration["rx_time"]):
                    print("Error: calibration timestamps do not match")
                    for receiver in self.receivers.values():
                        receiver.reset_receiver()
                    self.processing = False
                    return
        
        # all timestamps correct -> continue processing
        
        # log samples before interpolation to get logs of capable size and faster logging. Interpolation can be redone in parser. 
        if self.recording_samples:
            self.cnt_smpl_log +=1
            print("cnt_smpl: ",self.cnt_smpl_log)
            
            receiver_samples = []
            for receiver in receivers.values():
                receiver_samples.append(receiver.samples.tolist())
            if self.anchor_loop:
                self.sample_history.append(receiver_samples)
            else:
                f_s = open(self.samples_file,"a")
                pprint.pprint("[" + str(receiver_samples) +","+ str(receiver.interpolation)+","+ str(receivers.keys().index(self.ref_receiver))+ "]",f_s,width=9000)
                f_s.close()   
                    
        # interpolate samples
        signal_strength = []
        for receiver in receivers.values():    
            #do not interpolate if sector is set to 1 for system speedup
            if self.sample_interpolation > 1:      
                x = np.linspace(0,len(receiver.samples),len(receiver.samples))
                f = interpolate.interp1d(x, receiver.samples)
                x_interpolated = np.linspace(0,len(receiver.samples),len(receiver.samples) * receiver.interpolation)
                receiver.samples = np.array(f(x_interpolated))
                if receiver.auto_calibrate:
                    x = np.linspace(0,len(receiver.samples_calibration),len(receiver.samples_calibration))
                    f = interpolate.interp1d(x, receiver.samples_calibration)
                    x_interpolated = np.linspace(0,len(receiver.samples_calibration),len(receiver.samples_calibration) * receiver.interpolation)
                receiver.samples_calibration = f(x_interpolated)
            # find receiver with highest signal strength
            if self.reference_selection in ["Min-signal-power","Max-signal-power"]:
                # parseval theorem 
                signal_strength.append(np.sum(np.square(np.abs(receiver.samples))))
                
        if self.reference_selection == "Max-signal-power":
            self.ref_receiver = receivers.keys()[np.argmax(signal_strength)]
        elif self.reference_selection == "Min-signal-power":
            self.ref_receiver = receivers.keys()[np.argmin(signal_strength)]
        
        
        

        estimated_positions = {}
        kalman_states = {}

        if self.auto_calibrate and len(receivers) > 1:
            correlation, delay, correlation_labels = self.correlate(receivers, True)
            self.calibrate(self.coordinates_calibration, delay)

        if len(delay_auto_calibration) > 0 and self.auto_calibrate:
            index_delay_auto = 0
            for i in range(0,len(receivers)):
                if not self.ref_receiver == receivers.keys()[i]:
                    receivers.values()[i].samples = np.roll(receivers.values()[i].samples,delay_auto_calibration[index_delay_auto])
                    index_delay_auto += 1

        if len(self.delay_calibration) > 0:
            index_delay = 0
            for i in range(0,len(receivers)):
                if not self.ref_receiver == receivers.keys()[i]:
                    receivers.values()[i].samples = np.roll(receivers.values()[i].samples,self.delay_calibration[index_delay])
                    index_delay += 1
        
        if self.localizing:

            if not self.filtering_type=="Kalman filter":
                estimated_positions["chan"] = chan94_algorithm.localize(receivers, self.ref_receiver, np.round(self.basemap(self.bbox[2],self.bbox[3])))
                if self.reference_selection == "Min-DOP" :
                    try:
                        self.ref_receiver,dop_location,H = dop.reference_selection_dop(estimated_positions["chan"]["coordinates"],receivers)
                    except:
                        print ("reference selection not possible, localizing already stopped")
                if self.grid_based_active:
                    estimated_positions["grid_based"] = grid_based_algorithm.localize(receivers,np.round(self.basemap(self.bbox[2],self.bbox[3])), self.grid_based["resolution"], self.grid_based["num_samples"], self.ref_receiver)

                if self.filtering_type == "Moving average":
                    # average position estimation
                    if len(self.estimated_positions_history) == self.location_average_length:
                        self.estimated_positions_history.pop(0)
                    self.estimated_positions_history.append(estimated_positions)

                    average_chan = []
                    average_grid = []
                    for position in self.estimated_positions_history:
                        average_chan.append(position["chan"]["coordinates"])
                        if self.grid_based_active and position.has_key("grid_based"):
                            average_grid.append(position["grid_based"]["coordinates"])

                    estimated_positions["chan"]["average_coordinates"] = np.array(average_chan).mean(0).tolist()
                    if self.grid_based_active:
                        estimated_positions["grid_based"]["average_coordinates"] = np.array(average_grid).mean(0)
            else:
                if self.init_kalman:
                    print (self.init_settings_kalman)
                    self.init_settings_kalman["num_rx"] = len(self.receivers.values())
                    self.kalman_filter = kalman.kalman_filter(self.init_settings_kalman)
                    estimated_positions["chan"] = chan94_algorithm.localize(receivers, self.ref_receiver, np.round(self.basemap(self.bbox[2],self.bbox[3])))
                    if self.grid_based_active:
                        estimated_positions["grid_based"] = grid_based_algorithm.localize(receivers,np.round(self.basemap(self.bbox[2],self.bbox[3])), self.grid_based["resolution"], self.grid_based["num_samples"], self.ref_receiver)
                        self.xk_1_grid = np.hstack((np.array(list(estimated_positions["grid_based"]["coordinates"])),np.zeros(self.kalman_filter.get_state_size()-2)))
                        self.Pk_1_grid = self.kalman_filter.get_init_cov()
                        estimated_positions["grid_based"]["kalman_coordinates"] = np.array(list(estimated_positions["grid_based"]["coordinates"]))
                        kalman_states["grid_based"] = self.xk_1_grid
                    self.xk_1_chan = np.hstack((np.array(list(estimated_positions["chan"]["coordinates"])),np.zeros(self.kalman_filter.get_state_size()-2)))
                    self.Pk_1_chan = self.kalman_filter.get_init_cov()
                    estimated_positions["chan"]["kalman_coordinates"] = np.array(list(estimated_positions["chan"]["coordinates"]))
                    kalman_states["chan"] = self.xk_1_chan
                    x_cov = self.Pk_1_chan[0,0]
                    y_cov = self.Pk_1_chan[1,1]
                    self.init_kalman = False
                else:
                    '''
                    if dop_location < 1:
                        dop_location = 1
                    if dop_location > 10:
                        dop_location = 10
                    '''
                    
                    estimated_positions["chan"] = chan94_algorithm.localize(receivers, self.ref_receiver, np.round(self.basemap(self.bbox[2],self.bbox[3])),self.kalman_filter.get_a_priori_est(self.xk_1_chan)[:2])
                    #print (estimated_positions["chan"]["coordinates"])
                    measurement = self.kalman_filter.pre_filter(estimated_positions["chan"]["coordinates"],self.xk_1_chan)
                    if self.reference_selection == "Min-DOP" :
                        try:
                            self.ref_receiver,dop_location,H = dop.reference_selection_dop(estimated_positions["chan"]["coordinates"],receivers)
                        except:
                            print ("reference selection not possible, localizing already stopped")
                    else:
                        dop_location,H = dop.calc_dop(estimated_positions["chan"]["coordinates"],receivers, self.ref_receiver)
                    self.kalman_filter.adapt_R(H)
                    self.xk_1_chan,self.Pk_1_chan = self.kalman_filter.kalman_fltr(np.array(list(measurement)),self.Pk_1_chan,self.xk_1_chan,"chan")    
                    estimated_positions["chan"]["kalman_coordinates"] = self.xk_1_chan[:2]
                    #print (estimated_positions["chan"]["kalman_coordinates"])
                    x_cov = self.Pk_1_chan[0,0]
                    y_cov = self.Pk_1_chan[1,1]
                    if self.grid_based_active:
                        estimated_positions["grid_based"] = estimated_positions["grid_based"] = grid_based_algorithm.localize(receivers,np.round(self.basemap(self.bbox[2],self.bbox[3])), self.grid_based["resolution"], self.grid_based["num_samples"], self.ref_receiver)
                        estimated_positions["grid_based"]["coordinates"] = self.kalman_filter.pre_filter(np.array(list(estimated_positions["grid_based"]["coordinates"])),self.xk_1_grid)
                        self.xk_1_grid,self.Pk_1_grid = self.kalman_filter.kalman_fltr(estimated_positions["grid_based"]["coordinates"],self.Pk_1_grid,self.xk_1_grid,"grid_based")
                        estimated_positions["grid_based"]["kalman_coordinates"] = self.xk_1_grid[:2]
                        estimated_positions["grid_based"]["grid"] = 0
                        kalman_states["grid_based"] = self.xk_1_grid
                    kalman_states["chan"] = self.xk_1_chan
            if not self.run_loop:
                self.localizing = False
            else:
                self.init_kalman = False 

        correlation, delay, correlation_labels = None,None,None
        if len(receivers) > 1:
            correlation, delay, correlation_labels = self.correlate(receivers)
            if len(self.delay_history) < len(delay):
                self.delay_history = []
                for i in range(0,len(delay)):
                    self.delay_history.append([])
            for i in range(0,len(delay)):
                self.delay_history[i].append(delay[i])
        receivers_samples = []
        for receiver in receivers.values():
            receivers_samples.append(receiver.samples)
        self.results = {"rx_time":receivers.values()[0].tags["rx_time"],"receivers":receivers_samples,"correlation":correlation,"delay":delay,"delay_history":self.delay_history,"estimated_positions":estimated_positions,"correlation_labels":correlation_labels,"ref_receiver": self.ref_receiver, "Tx":None}
        for gui in self.guis.values():
            gui.rpc_manager.request("get_results",[self.results, {}])

        if self.recording_results:
            # build receivers strings for log file
            receivers_positions, selected_positions, receivers_gps, receivers_antenna, receivers_gain = helpers.build_results_strings(receivers)

            # remove grid from results to log
            for key in estimated_positions.keys():
                if key == "grid_based":
                    estimated_positions[key].pop("grid")

            for i in range(0,len(receivers)):
                if receivers.keys()[i] == self.ref_receiver:
                    index_ref_receiver = i

            line = "[" + str(self.results["rx_time"]) + "," + str(self.results["delay"]) + "," \
            + str(self.delay_calibration) + "," + str(delay_auto_calibration) + "," \
            + str(self.samp_rate) + "," + str(self.frequency) + "," + str(self.frequency_calibration) + "," \
            + str(self.coordinates_calibration) + "," + str(self.sample_interpolation) + "," \
            + str(self.bw)+ "," + str(self.samples_to_receive) + "," + str(self.lo_offset) + "," \
            + str(self.bbox) + "," + receivers_positions + "," + selected_positions + "," \
            + receivers_gps + "," + receivers_antenna + "," + receivers_gain + "," \
            + str(estimated_positions) + "," + str(index_ref_receiver) + "," \
            + str(self.auto_calibrate) + "," +str(self.acquisition_time) + "," \
            + str(kalman_states) + "," + str(self.init_settings_kalman) + "," \
            + "'" + str(self.reference_selection) + "'" + "," + str(x_cov) + ","+str(y_cov) +"]"

            f = open(self.results_file,"a")
            pprint.pprint(line,f,width=9000)
            f.close()

        # select new reference after acquisition if no prediction is available  
        if self.calibrating:
            self.calibration_loop_delays.append(delay)
            print(len(self.calibration_loop_delays))
            if len(self.calibration_loop_delays) == self.calibration_average:
                self.run_loop = False
                for gui in self.guis.values():
                    gui.rpc_manager.request("calibration_loop",[False])

        # may cause trouble with logging ?
        if self.anchor_loop:
            self.anchor_loop_delays.append(delay)
            # no sensor is transmitting
            self.transmitter_history.append(-1)
            self.timestamp_history.append(receivers.values()[0].tags["rx_time"]) 
            if len(self.anchor_loop_delays) == self.anchor_average:
                self.run_loop = False
                delay_mean = np.array(self.anchor_loop_delays).mean(0)
                self.anchor_positions.append(chan94_algorithm.localize(receivers, self.ref_receiver, np.round(self.basemap(self.bbox[2],self.bbox[3])), delay_mean)["coordinates"])
                self.set_anchor_position(self.anchor_positions[-1])
                self.anchor_loop = False
                self.anchor_loop_delay_history.append(self.anchor_loop_delays)
                self.anchor_loop_delays = []
                self.stop_loop()
                self.recording_samples = self.record_samples
                self.recording_results = self.record_results

        # set flags to enable new reception
        for receiver in receivers.values():
            receiver.samples = np.array([])
            receiver.samples_calibration = np.array([])
            receiver.first_packet = True
            receiver.reception_complete = False
        self.processing = False

    def process_selfloc(self, receivers, delay_auto_calibration):
        # transmitter=rx_j
        # add possibility to log!
        # "nested loop in class"
        # just take samples from sensors that are not transmitting. Should not be disturbing if other sensor receives if the samples are not processed.
        receiver_samples = []
        for receiver in receivers.values():
            receiver_samples.append(receiver.samples.tolist())
        if self.recording_samples:
            self.cnt_smpl_log += 1
            self.sample_history.append(receiver_samples)
        
        for cnt_l, rx_l in enumerate(receivers):
            for cnt_k, rx_k in enumerate(receivers):
                if self.cnt_j != cnt_l and self.cnt_j != cnt_k and cnt_l != cnt_k:
                    window_size = 13
                    #by now ugly hack, rethink later
                    self.delay_tensor[self.cnt_j,cnt_l,cnt_k,self.cnt_average] = corr_spline_interpolation(receivers.values()[cnt_l].samples, receivers.values()[cnt_k].samples, window_size)[1] 
                    #print(self.correlate({receivers.keys()[cnt_l]:receivers.values()[cnt_k],receivers.keys()[cnt_l]:receivers.values()[cnt_k]})[1])
                else:
                    self.delay_tensor[self.cnt_j,cnt_l,cnt_k,self.cnt_average] = 0.0
        self.transmitter_history.append(self.cnt_j)
        self.timestamp_history.append(receivers.values()[0].tags["rx_time"]) 
        delay = []
        delay_labels = []
        for idx in range(len(self.delay_tensor)):
            for idx_2 in range(len(self.delay_tensor)):
                if idx_2 < idx and idx_2 != self.cnt_j and idx != self.cnt_j:
                    delay.append(self.delay_tensor[self.cnt_j,idx_2,idx,self.cnt_average])
                    delay_labels.append("Delay%i"%(10*(idx_2+1)+idx+1))
        print("Delay: ",delay)
        if len(self.delay_history) < len(delay):
            self.delay_history = []
            for i in range(0,len(delay)):
                self.delay_history.append([])
        for i in range(0,len(delay)):
            self.delay_history[i].append(delay[i])
        self.results_selfloc = {"rx_time":receivers.values()[0].tags["rx_time"],"receivers":receiver_samples,"correlation":None,"delay":delay,"delay_history":self.delay_history, "estimated_positions":None,"correlation_labels": delay_labels,"ref_receiver": None, "Tx":self.receivers.keys()[self.cnt_j]}
        for gui in self.guis.values():
            gui.rpc_manager.request("get_results",[{}, self.results_selfloc])
        self.cnt_average += 1
        if self.cnt_average == self.sample_average:
            self.cnt_average = 0
            self.delay_history = []
            self.cnt_j += 1
            if self.cnt_j == len(receivers):
                for receiver in receivers.values():
                    receiver.samples = np.array([])
                    receiver.samples_calibration = np.array([])
                    receiver.first_packet = True
                    receiver.reception_complete = False
                self.processing = False

                # Recording of delays finished
                self.stop_selfloc_loop()
                self.evaluate_selfloc(receivers)
                self.processing = False
                return

            self.switch_transmitter(self.cnt_j)
        
        for receiver in receivers.values():
            receiver.samples = np.array([])
            receiver.samples_calibration = np.array([])
            receiver.first_packet = True
            receiver.reception_complete = False
        self.processing = False
        # by now, unclear where to set this again(point when tx needs to be turned off again) self.transmitter = -1

    def main_loop(self):
        reception_complete = {}
        while True:
            time.sleep(self.acquisition_time/4)
            if len(self.receivers) > 0:
                if all(self.receivers[key].reception_complete for key in self.receivers):
                    '''receivers = copy.deepcopy(self.receivers) # get rid off
                        threading.Thread(target = self.process_results, args = (self.receivers,self.delay_auto_calibration,)).start()'''
                    if not self.processing:
                        self.processing = True
                        if not self.self_localization:
                            threading.Thread(target = self.process_results, args = (self.receivers,self.delay_auto_calibration,)).start()
                        else:
                            threading.Thread(target = self.process_selfloc, args = (self.receivers,self.delay_auto_calibration,)).start()
 
                elif any(self.receivers[key].error_detected for key in self.receivers):
                    for receiver in self.receivers.values():
                        receiver.reset_receiver()
                else:
                    self.probe_manager_lock.acquire()
                    reception_complete = {}
                    for key in self.receivers:
                        reception_complete[key] = self.receivers[key].reception_complete
                    self.probe_manager.watcher(reception_complete)
                    self.probe_manager_lock.release()

    def correlate(self, receivers, calibration=False):
        correlation = []
        correlation_labels = []
        delay=[]
        i = 1
        for receiver in receivers:
            if not self.ref_receiver == receiver:
                if not calibration:
                    # For the spline interpolation of the cross correlation, a sufficient support is necessary
                    window_size = 13
                    if receivers[receiver].correlation_interpolation:
                        correlation_acquisition, delay_acquisition  = corr_spline_interpolation(receivers[receiver].samples, receivers[self.ref_receiver].samples,window_size)
                        delay.append(delay_acquisition)
                        correlation.append(correlation_acquisition)
                    else:
                        correlation.append(np.absolute(np.correlate(receivers[receiver].samples, receivers[self.ref_receiver].samples, "full")).tolist())
                        delay = (np.argmax(correlation, axis=1) - (self.samples_to_receive * self.sample_interpolation)+ 1).tolist()
                else:
                    if receivers[receiver].correlation_interpolation:
                        correlation_acquisition, delay_acquisition  = corr_spline_interpolation(receivers[receiver].samples_calibration, receivers[self.ref_receiver].samples_calibration,window_size)
                        delay.append(delay_acquisition)
                        correlation.append(correlation_acquisition)
                    else:
                        correlation.append(np.absolute(np.correlate(receivers[receiver].samples_calibration, receivers[self.ref_receiver].samples_calibration, "full")).tolist())
                        delay = (np.argmax(correlation, axis=1) - (self.samples_to_receive_calibration * self.sample_interpolation) + 1).tolist()
                correlation_labels.append("Rx" + str(i) + ",Rx" + str(receivers.keys().index(self.ref_receiver)+1))
            i += 1
        print("Delay:", delay, "samples")
        return correlation, delay, correlation_labels

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
    parser.add_option("", "--interpolation", type="string", default="1",
                      help="Interpolation factor")
    parser.add_option("", "--correlation-interpolation", action="store_true", default="False",
                      help="Interpolation factor")
    parser.add_option("", "--frequency", type="string", default="2.51e9",
                      help="Frequency")
    parser.add_option("", "--samp-rate", type="string", default="30.72e6",
                      help="Sampling rate")
    parser.add_option("", "--lo-offset", type="string", default="0",
                      help="LO offset")
    parser.add_option("", "--bandwidth", type="string", default="10e6",
                      help="Bandwidth")
    parser.add_option("", "--num-samples-calibration", type="string", default="300",
                      help="Number of samples in burst for calibration")
    parser.add_option("", "--frequency-calibration", type="string", default="2.37e9",
                      help="Frequency for calibration")
    parser.add_option("", "--samp-rate-calibration", type="string", default="50e6",
                      help="Sampling rate for calibration")
    parser.add_option("", "--lo-offset-calibration", type="string", default="0",
                      help="LO offset for calibration")
    parser.add_option("", "--bandwidth-calibration", type="string", default="50e6",
                      help="Bandwidth for calibration")
    parser.add_option("", "--antenna", type="string", default="RX2",
                      help="Antenna to use")
    parser.add_option("", "--gain", type="float", default="36",
                      help="Gain in dB")
    parser.add_option("", "--gain-calibration", type="float", default="36",
                      help="Gain in dB for calibration")
    parser.add_option("", "--auto-calibrate", action="store_true", default=False,
                      help="Activate reference calibration station")
    parser.add_option("", "--acquisition-time", type="float", default="0.5",
                      help="Seconds between acquisitions")
    parser.add_option("", "--selfloc-average-length", type="string", default="3",
                      help="Average length for self localization")
                      
    (options, args) = parser.parse_args()
    return options

###############################################################################
# Main
###############################################################################
if __name__ == "__main__":
    options = parse_options()
    fc = fusion_center(options)
    fc.main_loop()

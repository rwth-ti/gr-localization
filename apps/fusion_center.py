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
import chan94_algorithm, chan94_algorithm_filtered, kalman
import grid_based_algorithm
import dop
#from gcc_phat import gcc_phat

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
        self.auto_calibrate = options.no_auto_calibrate
        self.acquisition_time = float(options.acquisition_time)

        self.receivers = {}
        self.ref_receiver = ""
        self.guis = {}
        
        self.grid_based = {"resolution":10,"num_samples":self.samples_to_receive * self.interpolation}
        self.grid_based_active = False

        self.estimated_positions_history = []
        self.delay_history = []
        self.delay_calibration = []
        self.delay_auto_calibration = []
        self.calibration_loop_delays = []
        self.calibration_average = 60
        
        # postprocessing
        self.location_average_length = 3
        self.target_dynamic = 0.04
        self.max_acc = 1.2
        self.measurement_noise = 0.8
        self.reference_selections = ["Manual","Max-signal-power","Min-signal-power","Min-DOP"]
        self.filtering_types = ["No filtering","Moving average","Kalman filter"]
        self.motion_models = ["maneuvering","simple"]
        self.reference_selection = "Min-DOP"
        self.filtering_type = "Kalman filter"
        self.motion_model = "maneuvering"
        self.init_settings_kalman=dict()
        
        self.processing = False
        
        self.record_results = False
        self.record_samples = False
        self.recording_results = False
        self.recording_samples = False
        self.results_file = ""
        if not os.path.exists("../log"):
                os.makedirs("../log")
        self.results = None
        self.run_loop = False
        self.localizing = False
        self.ntp_sync = False
        self.calibrating = False
        
        
        
        
        
        self.map_type = "Online"
        self.map_file = ""
        self.coordinates_type = "Geographical"
        #kalman filtering
        self.xk_1_chan=np.array([])
        self.xk_1_grid=np.array([])
        self.Pk_1_chan=np.array([])
        self.Pk_1_grid=np.array([])
        self.init_settings_kalman["model"] = self.motion_model
        self.init_settings_kalman["delta_t"] = self.acquisition_time
        self.init_settings_kalman["noise_factor"] = self.target_dynamic 
        self.init_settings_kalman["filter_receivers"] = False
        self.init_settings_kalman["noise_var_x"] = self.measurement_noise
        self.init_settings_kalman["noise_var_y"] = self.measurement_noise
        self.init_settings_kalman["max_acceleration"]= self.max_acc
        # ICT + surroundings
        #self.bbox = 6.0580,50.7775,6.0690,50.7810
        self.bbox = 6.0606,50.77819,6.06481,50.77967
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
        self.rpc_manager.add_interface("update_receivers",self.update_receivers)
        self.rpc_manager.add_interface("get_gui_gps_position",self.get_gui_gps_position)
        self.rpc_manager.add_interface("localize",self.localize)
        self.rpc_manager.add_interface("localize_loop",self.localize_loop)
        self.rpc_manager.add_interface("calibrate",self.calibrate)
        self.rpc_manager.add_interface("remove_calibration",self.remove_calibration)
        self.rpc_manager.add_interface("calibration_loop",self.calibration_loop)
        self.rpc_manager.add_interface("set_calibration_average",self.set_calibration_average)
        self.rpc_manager.add_interface("set_location_average_length",self.set_location_average_length)
        self.rpc_manager.add_interface("set_target_dynamic",self.set_target_dynamic)
        self.rpc_manager.add_interface("set_max_acc",self.set_max_acc)
        self.rpc_manager.add_interface("set_measurement_noise",self.set_measurement_noise)
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

        #self.frequency_calibration = 602000000
        #self.coordinates_calibration = self.basemap(50.745597, 6.043278)
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
                        else:
                            pos_receiver = receiver.coordinates_gps
                        d_ref = np.linalg.norm(np.array(coordinates)-pos_ref)
                        d_receiver = np.linalg.norm(np.array(coordinates)-pos_receiver)
                        delay_true = (d_receiver-d_ref) * self.samp_rate * self.interpolation / 299700000
                        print("True delay: ",delay_true)
                        #TODO average calibration_loop_delays
                        print(index_delay,len(self.delay_calibration),len(self.calibration_loop_delays))
                        print(self.calibration_loop_delays)
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
            
    def set_measurement_noise(self, measurement_noise):
        self.measurement_noise = measurement_noise
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_measurement_noise",[measurement_noise])

    def set_calibration_average(self, calibration_average):
        self.calibration_average = calibration_average
        for gui in self.guis.values():
            gui.rpc_manager.request("set_gui_calibration_average",[calibration_average])

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
            gui.rpc_manager.request("set_gui_interpolation",[self.interpolation])
            gui.rpc_manager.request("set_gui_samp_rate",[self.samp_rate])
            gui.rpc_manager.request("set_gui_frequency_calibration",[self.frequency_calibration])
            gui.rpc_manager.request("set_gui_lo_offset_calibration",[self.lo_offset_calibration])
            gui.rpc_manager.request("set_gui_samples_to_receive_calibration",[self.samples_to_receive_calibration])
            gui.rpc_manager.request("set_gui_bw_calibration",[self.bw_calibration])
            gui.rpc_manager.request("set_gui_auto_calibrate",[self.auto_calibrate])
            gui.rpc_manager.request("set_gui_calibration_average",[self.calibration_average])
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
            receiver.interpolation = self.interpolation
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
        time_to_receive = np.ceil(time.time()) + 1
        for receiver in self.receivers.values():
            threading.Thread(target = self.start_receiver, args = (receiver, time_to_receive, acquisitions)).start()


    def start_receiver(self, receiver, time_to_receive, acquisitions):
            receiver.samples = []
            receiver.samples_calibration = []
            receiver.first_packet = True
            receiver.reception_complete = False
            self.init_kalman = True
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
        self.results_file = "../log/results_" + time.strftime("%d_%m_%y-%H:%M:%S") + ".txt"
        self.recording_samples = self.record_samples
        self.samples_file = "../log/samples_" + time.strftime("%d_%m_%y-%H:%M:%S") + ".txt"
        if self.recording_results:
            print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
            print("rx_time,delays(1-2,1-3,1-X...),delays_calibration(1-2,1-3,1-X...),delays_auto_calibration(1-2,1-3,1-X...),sampling_rate,frequency,frequency_calibration,calibration_position,interpolation,bandwidth,samples_to_receive,lo_offset,bbox,receivers_positions,selected_positions,receivers_gps,receivers_antenna,receivers_gain,estimated_positions,index_ref_receiver,auto_calibrate,acquisition_time,kalman_states,init_settings_kalman, reference_selection", file=open(self.results_file,"a"))
            print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
        self.run_loop = True
        self.start_receivers(acquisitions)

    def localize(self, freq, lo_offset, samples_to_receive):
        if len(self.receivers) > 2:
            self.localizing = True
            self.start_receivers()

    def localize_loop(self, freq, lo_offset, samples_to_receive, acquisitions = 0):
        self.delay_history = []
        self.estimated_positions_history = []
        if len(self.receivers) > 2:
            self.localizing = True
            self.recording_results = self.record_results
            self.results_file = "../log/results_" + time.strftime("%d_%m_%y-%H:%M:%S") + ".txt"
            self.recording_samples = self.record_samples
            self.samples_file = "../log/samples_" + time.strftime("%d_%m_%y-%H:%M:%S") + ".txt"
            if self.recording_results:
                print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
                print("rx_time,delays(1-2,1-3,1-X...),delays_calibration(1-2,1-3,1-X...),delays_auto_calibration(1-2,1-3,1-X...),sampling_rate,frequency,frequency_calibration,calibration_position,interpolation,bandwidth,samples_to_receive,lo_offset,bbox,receivers_positions,selected_positions,receivers_gps,receivers_antenna,receivers_gain,estimated_positions,index_ref_receiver,auto_calibrate,acquisition_time,kalman_states,init_settings_kalman, reference_selection", file=open(self.results_file,"a"))
                print("##########################################################################################################################################################################################", file=open(self.results_file,"a"))
            self.run_loop = True
            self.start_receivers(acquisitions)

    def stop_loop(self):
        self.run_loop = False
        self.recording_results = False
        self.recording_samples = False
        self.estimated_positions_history = []
        self.localizing = False
        
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
        #print("in fc f_cal", frequency)
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
        num_samples = min(num_samples,self.samples_to_receive * self.interpolation)
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

    def process_results(self, receivers, delay_auto_calibration):
        # check if timestamps are equal for all the receivers
        times_target = []
        times_calibration = []

        last_time_target = receivers.values()[0].tags["rx_time"]
        #print(receivers.values()[0].tags)
        if self.auto_calibrate:
            last_time_calibration = receivers.values()[0].tags_calibration["rx_time"]

        for i in range(1,len(receivers)):

            if not (last_time_target == receivers.values()[i].tags["rx_time"]):
                print("Error: target timestamps do not match")
                for i in range(0,len(receivers)):
                    print (i,receivers.values()[i].serial,receivers.values()[i].tags)
                for receiver in self.receivers.values():
                    receiver.reset_receiver()
                return
            if self.auto_calibrate:
                if not (last_time_calibration == receivers.values()[i].tags_calibration["rx_time"]):
                    print("Error: calibration timestamps do not match")
                    for receiver in self.receivers.values():
                        receiver.reset_receiver()
                    return
        
        # all timestamps correct -> continue processing
        
        # log samples before interpolation to get logs of capable size and faster logging. Interpolation can be redone in parser. 
        if self.recording_samples:
            f_s = open(self.samples_file,"a")
            receiver_samples = []
            for receiver in receivers.values():
                receiver_samples.append(receiver.samples.tolist())
            pprint.pprint("[" + str(receiver_samples) +","+ str(receiver.interpolation)+","+ str(receivers.keys().index(self.ref_receiver))+ "]",f_s,width=9000)
            f_s.close()   
                    
        # interpolate samples
        signal_strength = []
        for receiver in receivers.values():          
            x = np.linspace(0,len(receiver.samples),len(receiver.samples))
            f = interpolate.interp1d(x, receiver.samples)
            x_interpolated = np.linspace(0,len(receiver.samples),len(receiver.samples) * receiver.interpolation)
            receiver.samples = f(x_interpolated)
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
        print (self.ref_receiver)
        
        if self.reference_selection == "Min-DOP" and  self.xk_1_chan.any():
            self.ref_receiver,dop_location = dop.reference_selection_dop(self.kalman_filter.get_a_priori_est(self.xk_1_chan)[:2],receivers)
        elif self.xk_1_chan.any():
            dop_location=dop.calc_dop(self.kalman_filter.get_a_priori_est(self.xk_1_chan)[:2],receivers,self.ref_receiver)
        
        

        estimated_positions = {}
        kalman_states = {}

        if self.auto_calibrate and len(receivers) > 1:
            correlation, delay, correlation_labels = self.correlate(receivers, True)
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

            if not self.filtering_type=="Kalman filter":
                estimated_positions["chan"] = chan94_algorithm.localize(receivers, self.ref_receiver, np.round(self.basemap(self.bbox[2],self.bbox[3])))
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
                    self.kalman_filter = kalman.kalman_filter(self.init_settings_kalman)
                    estimated_positions["chan"] = chan94_algorithm.localize(receivers, self.ref_receiver, np.round(self.basemap(self.bbox[2],self.bbox[3])))
                    if self.grid_based_active:
                        estimated_positions["grid_based"] = grid_based_algorithm.localize(receivers, self.ref_receiver, np.round(self.basemap(self.bbox[2],self.bbox[3])))
                        self.xk_1_grid = np.hstack((np.array(list(estimated_positions["grid_based"]["coordinates"])),np.zeros(self.kalman_filter.get_state_size()-2)))
                        self.Pk_1_grid = self.kalman_filter.get_init_cov()
                        estimated_positions["grid-based"]["kalman_coordinates"] = np.array(list(estimated_positions["grid-based"]["coordinates"]))
                        kalman_states["grid-based"] = self.xk_1_grid
                    self.xk_1_chan = np.hstack((np.array(list(estimated_positions["chan"]["coordinates"])),np.zeros(self.kalman_filter.get_state_size()-2)))
                    self.Pk_1_chan = self.kalman_filter.get_init_cov()
                    estimated_positions["chan"]["kalman_coordinates"] = np.array(list(estimated_positions["chan"]["coordinates"]))
                    #print (estimated_positions["chan"]["kalman_coordinates"])
                    kalman_states["chan"] = self.xk_1_chan
                    self.init_kalman = False
                else:
                    if dop_location < 1:
                        dop_location = 1
                    if dop_location > 10:
                        dop_location = 10
                    self.kalman_filter.scale_measurement_noise(dop_location)
                    estimated_positions["chan"] = chan94_algorithm_filtered.localize(receivers, self.ref_receiver, np.round(self.basemap(self.bbox[2],self.bbox[3])),self.kalman_filter.get_a_priori_est(self.xk_1_chan)[:2])
                    #print (estimated_positions["chan"]["coordinates"])
                    estimated_positions["chan"]["coordinates"] = self.kalman_filter.pre_filter(estimated_positions["chan"]["coordinates"],self.xk_1_chan)
                    self.xk_1_chan,self.Pk_1_chan = self.kalman_filter.kalman_fltr(np.array(list(estimated_positions["chan"]["coordinates"])),self.Pk_1_chan,self.xk_1_chan,"chan")    
                    estimated_positions["chan"]["kalman_coordinates"] = self.xk_1_chan[:2]
                    #print (estimated_positions["chan"]["kalman_coordinates"])

                    if self.grid_based_active:
                        estimated_positions["grid_based"] = grid_based_algorithm.localize(receivers, self.ref_receiver, np.round(self.basemap(self.bbox[2],self.bbox[3])))
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
        self.results = {"rx_time":receivers.values()[0].tags["rx_time"],"receivers":receivers_samples,"correlation":correlation,"delay":delay,"delay_history":self.delay_history,"estimated_positions":estimated_positions,"correlation_labels":correlation_labels,"ref_receiver": self.ref_receiver}
        
        for gui in self.guis.values():
            gui.rpc_manager.request("get_results",[self.results])

        if self.recording_results:
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

            line = "[" + str(self.results["rx_time"]) + "," + str(self.results["delay"]) + "," + str(self.delay_calibration) + "," + str(delay_auto_calibration) + "," + str(self.samp_rate) + "," + str(self.frequency) + "," + str(self.frequency_calibration) + "," + str(self.coordinates_calibration) + "," + str(self.interpolation) + "," + str(self.bw)+ "," + str(self.samples_to_receive) + "," + str(self.lo_offset) + "," + str(self.bbox) + "," + receivers_position + "," + selected_positions + "," + receivers_gps + "," + receivers_antenna + "," + receivers_gain + "," + str(estimated_positions) + "," + str(index_ref_receiver) + "," + str(self.auto_calibrate) + "," +str(self.acquisition_time) + "," + str(kalman_states)+","+str(self.init_settings_kalman)+","+"'"+str(self.reference_selection)+"'"+"]"
            f = open(self.results_file,"a")
            pprint.pprint(line,f,width=9000)
            f.close()
        # select new reference after acquisition if no prediction is available
        
        if self.reference_selection == "Min-DOP" and not self.xk_1_chan.any():
            try:
                self.ref_receiver,dop_location = dop.reference_selection_dop(estimated_positions["chan"]["coordinates"],receivers)
            except:
                print ("reference selection not possible, localizing already stopped")
        
        
        if self.calibrating:
            self.calibration_loop_delays.append(delay)
            print(len(self.calibration_loop_delays))
            if len(self.calibration_loop_delays) == self.calibration_average:
                self.run_loop = False
                for gui in self.guis.values():
                    gui.rpc_manager.request("calibration_loop",[False])
        
        # set flags to enable new reception
        for receiver in receivers.values():
            receiver.samples = []
            receiver.samples_calibration = []
            receiver.first_packet = True
            receiver.reception_complete = False
        self.processing=False

    def main_loop(self):
        while True:
            time.sleep(self.acquisition_time/4)
            if len(self.receivers) > 0:
                if all(self.receivers[key].reception_complete for key in self.receivers):
                    '''receivers = copy.deepcopy(self.receivers) # get rid off
                        threading.Thread(target = self.process_results, args = (self.receivers,self.delay_auto_calibration,)).start()'''
                    if not self.processing:
                        self.processing = True
                        #self.process_results(self.receivers,self.delay_auto_calibration)
                        threading.Thread(target = self.process_results, args = (self.receivers,self.delay_auto_calibration,)).start()
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
        i = 1
        for receiver in receivers:
            if not self.ref_receiver == receiver:
                if not calibration:
                    correlation.append(np.absolute(np.correlate(receivers[receiver].samples, receivers[self.ref_receiver].samples, "full", False)).tolist())
                    #correlation.append(np.absolute(gcc_phat(receivers[receiver].samples, receivers[self.ref_receiver].samples)).tolist())
                    delay = (np.argmax(correlation, axis=1) - (self.samples_to_receive * self.interpolation)+ 1).tolist()
                else:
                    correlation.append(np.absolute(np.correlate(receivers[receiver].samples_calibration, receivers[self.ref_receiver].samples_calibration, "full", False)).tolist())
                    #correlation.append(gcc_phat(np.correlate(receivers[receiver].samples_calibration, receivers[self.ref_receiver].samples_calibration)).tolist())
                    delay = (np.argmax(correlation, axis=1) - (self.samples_to_receive_calibration * self.interpolation) + 1).tolist()
                correlation_labels.append("Rx" + str(i) + ",Rx" + str(receivers.keys().index(self.ref_receiver)+1))
            i +=1
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
    parser.add_option("", "--interpolation", type="string", default="10",
                      help="Interpolation factor")
    parser.add_option("", "--frequency", type="string", default="2.51e9",
                      help="Frequency")
    parser.add_option("", "--samp-rate", type="string", default="50e6",
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
    parser.add_option("", "--bandwidth-calibration", type="string", default="10e6",
                      help="Bandwidth for calibration")
    parser.add_option("", "--antenna", type="string", default="RX2",
                      help="Antenna to use")
    parser.add_option("", "--gain", type="float", default="36",
                      help="Gain in dB")
    parser.add_option("", "--gain-calibration", type="float", default="36",
                      help="Gain in dB for calibration")
    parser.add_option("", "--no-auto-calibrate", action="store_true", default=False,
                      help="Deactivate reference calibration station")
    parser.add_option("", "--acquisition-time", type="float", default="0.5",
                      help="Seconds between acquisitions")
                      
    (options, args) = parser.parse_args()
    return options

###############################################################################
# Main
###############################################################################
if __name__ == "__main__":
    options = parse_options()
    fc = fusion_center(options)
    fc.main_loop()

#!/usr/bin/env python

###############################################################################
# Imports
###############################################################################
from optparse import OptionParser
from gnuradio.eng_option import eng_option
import sys
import os
from PyQt4 import Qt, QtGui, QtCore, uic
import PyQt4.Qwt5 as Qwt
from gnuradio import zeromq
import signal
import numpy as np
import time
import threading
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap
import requests
from StringIO import StringIO
from PIL import Image
from pyproj import Proj, transform
import math
import socket
import urllib2
from functools import partial
from collections import deque
sys.path.append("../python")
import receiver_interface
import rpc_manager as rpc_manager_local
import gui_helpers

def check_OSM():
    try:
        header = {"pragma" : "no-cache"} # Tells the server to send fresh copy
        req = urllib2.Request("http://www.openstreetmap.org", headers=header)
        response=urllib2.urlopen(req,timeout=2)
        print "you are connected"
        return True
    except urllib2.URLError as err:
        print "Openstreetmap.org not availabe. Check your internet connection"
        return False

class gui(QtGui.QMainWindow):
    signal_error_set_map = QtCore.pyqtSignal()
    def __init__(self, window_name, options, parent=None):
        QtGui.QMainWindow.__init__(self, parent)

        self.thread = threading.current_thread()

        # give Ctrl+C back to system
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        self.gui = uic.loadUi(os.path.join(os.path.dirname(__file__),'../gui/main_window.ui'), self)

        self.update_timer = Qt.QTimer()

        self.hostname = os.uname()[1]
        self.gui_serial = self.hostname + str(options.id_gui)

        # socket addresses
        rpc_adr = "tcp://*:" + str(7775 + options.id_gui)
        fusion_center_adr = "tcp://" + options.fusion_center + ":6665"

        self.calibration_average = 10
        self.location_average_length = 10
        self.target_dynamic = 0.6
        self.max_acc = 3.5
        self.measurement_noise = 14
        self.samples_to_receive = 1000
        self.frequency = 2.4e9
        self.samp_rate = 10e6
        self.bw = 1e6
        self.sample_interpolation = 1
        self.correlation_interpolation = True
        self.lo_offset = 0
        self.samples_to_receive_calibration = 1000
        self.frequency_calibration = 2.4e9
        self.bw_calibration = 1e6
        self.lo_offset_calibration = 0
        self.acquisition_time = 0
        self.queue_tx_coordinates = deque()
        self.queue_tx_coordinates_kalman = deque()
        self.results = {}
        self.new_results = False
        self.trackplot_length = 180

        self.auto_calibrate = False
        self.receivers = {}
        self.ref_receiver = ""
        
        self.reference_selections = []
        self.reference_selection = ""
        
        self.transmitter_positions = {}
        self.filtering_types = []
        self.filtering_type = ""
        self.map_type = ""
        self.map_file = ""
        self.coordinates_type = ""

        self.alpha = 1.0
        self.init_stres = 10.0
        self.max_it = 99

        self.grid_based_active = False

        self.gui.comboBoxMapType.addItem("Online")
        self.gui.comboBoxMapType.addItem("Offline")
        self.gui.comboBoxCoordinatesType.addItem("Geographical")
        self.gui.comboBoxCoordinatesType.addItem("Projected")

        self.chats = ""
        # for delay history plots
        self.ymax_dh = 30
        
        self.num_anchor_position = 0
        self.num_anchors = 3

        # ZeroMQ
        self.rpc_manager = rpc_manager_local.rpc_manager()
        self.rpc_manager.set_reply_socket(rpc_adr)
        self.rpc_manager.set_request_socket(fusion_center_adr)
        self.rpc_manager.add_interface("new_chat",self.new_chat)
        self.rpc_manager.add_interface("sync_position",self.sync_position)
        self.rpc_manager.add_interface("sync_position_selfloc",self.sync_position_selfloc)
        self.rpc_manager.add_interface("register_receiver",self.register_receiver)
        self.rpc_manager.add_interface("register_another_gui",self.register_another_gui)
        self.rpc_manager.add_interface("get_results",self.get_results)
        self.rpc_manager.add_interface("set_gui_frequency",self.set_gui_frequency)
        self.rpc_manager.add_interface("set_gui_samp_rate",self.set_gui_samp_rate)
        self.rpc_manager.add_interface("set_gui_bw",self.set_gui_bw)
        self.rpc_manager.add_interface("set_gui_interpolation",self.set_gui_interpolation)
        self.rpc_manager.add_interface("set_gui_correlation_interpolation",self.set_gui_correlation_interpolation)
        self.rpc_manager.add_interface("set_gui_lo_offset",self.set_gui_lo_offset)
        self.rpc_manager.add_interface("set_gui_samples_to_receive",self.set_gui_samples_to_receive)
        self.rpc_manager.add_interface("set_gui_frequency_calibration",self.set_gui_frequency_calibration)
        self.rpc_manager.add_interface("set_gui_bw_calibration",self.set_gui_bw_calibration)
        self.rpc_manager.add_interface("set_gui_lo_offset_calibration",self.set_gui_lo_offset_calibration)
        self.rpc_manager.add_interface("set_gui_samples_to_receive_calibration",self.set_gui_samples_to_receive_calibration)
        self.rpc_manager.add_interface("set_gui_gain",self.set_gui_gain)
        self.rpc_manager.add_interface("set_gui_gain_calibration",self.set_gui_gain_calibration)
        self.rpc_manager.add_interface("set_gui_antenna",self.set_gui_antenna)
        self.rpc_manager.add_interface("set_gui_selected_position",self.set_gui_selected_position)
        self.rpc_manager.add_interface("set_gps_position",self.set_gps_position)
        self.rpc_manager.add_interface("set_gui_TDOA_grid_based_resolution",self.set_gui_TDOA_grid_based_resolution)
        self.rpc_manager.add_interface("set_gui_TDOA_grid_based_num_samples",self.set_gui_TDOA_grid_based_num_samples)
        self.rpc_manager.add_interface("set_gui_TDOA_grid_based_channel_model",self.set_gui_TDOA_grid_based_channel_model)
        self.rpc_manager.add_interface("set_gui_TDOA_grid_based_measurement_type",self.set_gui_TDOA_grid_based_measurement_type)
        self.rpc_manager.add_interface("set_gui_ref_receiver",self.set_gui_ref_receiver)
        self.rpc_manager.add_interface("set_gui_reference_selections",self.set_gui_reference_selections)
        self.rpc_manager.add_interface("set_gui_reference_selection",self.set_gui_reference_selection)
        self.rpc_manager.add_interface("set_gui_filtering_type",self.set_gui_filtering_type)
        self.rpc_manager.add_interface("set_gui_motion_model",self.set_gui_motion_model)
        self.rpc_manager.add_interface("set_gui_map_type",self.set_gui_map_type)
        self.rpc_manager.add_interface("set_gui_map_file",self.set_gui_map_file)
        self.rpc_manager.add_interface("set_gui_coordinates_type",self.set_gui_coordinates_type)
        self.rpc_manager.add_interface("set_gui_auto_calibrate",self.set_gui_auto_calibrate)
        self.rpc_manager.add_interface("set_gui_calibration_average",self.set_gui_calibration_average)
        self.rpc_manager.add_interface("set_gui_sample_average",self.set_gui_sample_average)
        self.rpc_manager.add_interface("set_gui_anchor_average",self.set_gui_anchor_average)
        self.rpc_manager.add_interface("set_gui_num_anchors",self.set_gui_num_anchors)
        self.rpc_manager.add_interface("set_gui_tx_gain",self.set_gui_tx_gain)
        self.rpc_manager.add_interface("set_gui_location_average_length",self.set_gui_location_average_length)
        self.rpc_manager.add_interface("set_gui_measurement_noise",self.set_gui_measurement_noise)
        self.rpc_manager.add_interface("set_gui_target_dynamic",self.set_gui_target_dynamic)
        self.rpc_manager.add_interface("set_gui_alpha",self.set_gui_alpha)
        self.rpc_manager.add_interface("set_gui_max_it",self.set_gui_max_it)
        self.rpc_manager.add_interface("set_gui_init_stress",self.set_gui_init_stress)
        self.rpc_manager.add_interface("set_gui_max_acc",self.set_gui_max_acc)
        self.rpc_manager.add_interface("set_gui_record_results",self.set_gui_record_results)
        self.rpc_manager.add_interface("set_gui_record_samples",self.set_gui_record_samples)
        self.rpc_manager.add_interface("set_gui_filtering_types",self.set_gui_filtering_types)
        self.rpc_manager.add_interface("set_gui_acquisition_time",self.set_gui_acquisition_time)
        self.rpc_manager.add_interface("set_gui_grid_based_active",self.set_gui_grid_based_active)
        self.rpc_manager.add_interface("set_gui_motion_models",self.set_gui_motion_models)
        self.rpc_manager.add_interface("init_map",self.init_map)
        self.rpc_manager.add_interface("calibration_loop",self.calibration_loop)
        self.rpc_manager.add_interface("calibration_status",self.calibration_status)
        self.rpc_manager.add_interface("start_anchoring", self.start_anchoring)
        self.rpc_manager.add_interface("set_anchor_position", self.set_anchor_position)
        self.rpc_manager.start_watcher()

        # Find out ip address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if not options.ssh_proxy:
            s.connect((options.fusion_center,6665))
        else:
            s.connect(("www.rwth-aachen.de",80))
        self.ip_addr = s.getsockname()[0]

        self.gui.setWindowTitle(window_name)

        self.chat_pending = False

        # map configuration
        self.bbox = ()
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = gui_helpers.NavigationToolbar(self.canvas, self)

        self.canvas.mpl_connect("button_release_event", self.set_position)
        self.canvas.mpl_connect("button_release_event", self.get_gps_coordinates)

        self.verticalLayoutMap.addWidget(self.toolbar)
        self.verticalLayoutMap.addWidget(self.canvas)


        # correlation and signals
        self.init_plot(self.gui.qwtPlotReceiver1)
        self.init_plot(self.gui.qwtPlotReceiver2)
        self.init_plot(self.gui.qwtPlotReceiver3)
        title = Qwt.QwtText("Delay")
        title.setFont(Qt.QFont("Helvetica", 14, Qt.QFont.Bold))
        self.gui.qwtPlotCorrelation.setAxisTitle(Qwt.QwtPlot.xBottom, title)
        title = Qwt.QwtText("Amplitude")
        title.setFont(Qt.QFont("Helvetica", 10, Qt.QFont.Bold))
        self.gui.qwtPlotCorrelation.setAxisTitle(Qwt.QwtPlot.yLeft, title)
        self.gui.qwtPlotCorrelation.setAxisScale(Qwt.QwtPlot.xBottom, -self.samples_to_receive * self.sample_interpolation, self.samples_to_receive * self.sample_interpolation)
        self.gui.qwtPlotCorrelation.insertLegend(Qwt.QwtLegend(), Qwt.QwtPlot.RightLegend)
        self.gui.qwtPlotDelayHistory.setAxisScale(Qwt.QwtPlot.yLeft, -10, 10)

        # create and set model for receivers position table view
        self.tmrp = gui_helpers.TableModelReceiversPosition(self)
        self.gui.tableViewReceiversPosition.setModel(self.tmrp)
        self.gui.tableViewReceiversPosition.setItemDelegateForColumn(1, gui_helpers.GpsComboDelegate(self))
        self.gui.tableViewReceiversPosition.setItemDelegateForColumn(2, gui_helpers.PushButtonPositionDelegate(self))
        self.set_delegate = False
        self.setting_pos_receiver = ""
        self.setting_calibration = False

        self.pending_receivers_to_plot = False

        # create and set model for receivers table view
        self.tmr = gui_helpers.TableModelReceivers(self)
        self.gui.tableViewReceivers.setModel(self.tmr)
        self.gui.tableViewReceivers.setItemDelegateForColumn(1, gui_helpers.SpinBoxDelegate(self.gui.tableViewReceivers))
        self.gui.tableViewReceivers.setItemDelegateForColumn(2, gui_helpers.ComboDelegate(self.gui.tableViewReceivers))
        self.gui.tableViewReceivers.setItemDelegateForColumn(3, gui_helpers.SpinBoxDelegate(self.gui.tableViewReceivers))
        self.set_delegate = False

        # create and set model for guis table view
        self.tmg = gui_helpers.TableModelGuis(self)
        self.gui.tableViewGuis.setModel(self.tmg)

        # Grid
        pen = Qt.QPen(Qt.Qt.DotLine)
        pen.setColor(Qt.Qt.black)
        pen.setWidth(0)
        grid_correlation = Qwt.QwtPlotGrid()
        grid_correlation.setPen(pen)
        grid_correlation.attach(self.gui.qwtPlotCorrelation)

        # Dialog box for calibration
        self.calibration_dialog = QtGui.QDialog(self)
        self.calibration_mBox = QtGui.QDialogButtonBox(self)
        self.calibration_setButton = self.calibration_mBox.addButton("Get coordinates from map", QtGui.QDialogButtonBox.AcceptRole)
        self.calibration_gpsInputButton = self.calibration_mBox.addButton("Set calibration", QtGui.QDialogButtonBox.AcceptRole)
        self.calibration_cancelButton = self.calibration_mBox.addButton("Cancel", QtGui.QDialogButtonBox.RejectRole)
        self.calibration_mBox.clicked.connect(self.set_tx_calibration)
        self.calibration_dialog.setWindowTitle("Calibrating system")
        self.waitText = QtGui.QLabel("Please wait until calibration process is complete.")
        layout = QtGui.QFormLayout()
        layout.addRow(self.waitText)
        self.latLabel_cal = QtGui.QLabel("latitude (+=N ; -=S)")
        self.lineEditLatitude_cal = QtGui.QLineEdit()
        layout.addRow(self.latLabel_cal,self.lineEditLatitude_cal)
        self.longLabel_cal = QtGui.QLabel("longitude (+=E ; -=W)")
        self.lineEditLongitude_cal = QtGui.QLineEdit()
        layout.addRow(self.longLabel_cal,self.lineEditLongitude_cal)
        layout.addRow(self.calibration_mBox)
        #self.gpsCheckBox = QtGui.QCheckBox("calibrate with gps coordinates")
        self.calibration_dialog.setLayout(layout)
        

        # Dialog box for receiver positions
        self.position_dialog = QtGui.QDialog(self)
        self.position_mBox_pos = QtGui.QDialogButtonBox(self)
        self.position_cancelButton = self.position_mBox_pos.addButton("Cancel", QtGui.QDialogButtonBox.RejectRole)
        self.position_gpsInputButton = self.position_mBox_pos.addButton("Set", QtGui.QDialogButtonBox.AcceptRole)
        self.position_setButton = self.position_mBox_pos.addButton("From map", QtGui.QDialogButtonBox.AcceptRole)
        self.position_mBox_pos.clicked.connect(self.manage_position)
        self.position_dialog.setWindowTitle("Set receiver position")
        self.waitText_pos = QtGui.QLabel("Provide receiver coordinates or select by clicking on the map.")

        # reuse long and lat label from calibration Dialog
        # add altitude
        self.altLabel = QtGui.QLabel("altitude (m)")
        self.lineEditAltitude_pos = QtGui.QLineEdit() 
        layout = QtGui.QFormLayout()
        layout.addRow(self.waitText_pos)
        self.latLabel_pos = QtGui.QLabel("latitude (+=N ; -=S)")
        self.lineEditLatitude_pos = QtGui.QLineEdit()
        layout.addRow(self.latLabel_pos,self.lineEditLatitude_pos)
        self.longLabel_pos = QtGui.QLabel("longitude (+=E ; -=W)")
        self.lineEditLongitude_pos = QtGui.QLineEdit()
        layout.addRow(self.longLabel_pos,self.lineEditLongitude_pos)
        layout.addRow(self.altLabel,self.lineEditAltitude_pos)
        layout.addRow(self.position_mBox_pos)
        #self.gpsCheckBox = QtGui.QCheckBox("calibrate with gps coordinates")
        self.position_dialog.setLayout(layout)

        

        # Dialog box for anchoring
        
        self.anchor_dialog = QtGui.QDialog(self)
        self.anchor_dialog.setWindowTitle("Self-localization")
        layout = QtGui.QFormLayout()
        #self.waitText3 = QtGui.QLabel("Please wait until the required samples are received.")
        #layout.addRow(self.waitText3)
        self.groupBoxDmds = QtGui.QGroupBox("1. Self-localization (relative)")
        self.pushButtonDMDS = QtGui.QPushButton("Start")
        helpLayout = QtGui.QHBoxLayout()
        self.waitText3 = QtGui.QLabel("Place sensors and press 'Start'")
        helpLayout.addWidget(self.waitText3)
        helpLayout.addWidget(self.pushButtonDMDS)
        self.groupBoxDmds.setLayout(helpLayout)
        layout.addRow(self.groupBoxDmds)
        self.groupBoxAnchoring = QtGui.QGroupBox("2. Anchoring (absolute)")
        layoutAnchoring = QtGui.QFormLayout()

        self.curr_anchor_text = QtGui.QLabel("Anchor Number:")
        self.curr_anchor_spin = QtGui.QSpinBox()
        self.curr_anchor_spin.setValue(1)
        layout_hor = QtGui.QHBoxLayout()
        layout_hor.addWidget(self.curr_anchor_text)
        layout_hor.addWidget(self.curr_anchor_spin)
        layoutAnchoring.addRow(layout_hor)
        
        self.waitText2 = QtGui.QLabel("Place anchor and press 'Start'")
        self.pushButtonOK = QtGui.QPushButton("Start")
        self.connect(self.pushButtonOK, QtCore.SIGNAL("clicked()"), self.start_anchoring_loop)
        
        layoutAnchoring.addRow(self.waitText2, self.pushButtonOK)
        self.latLabel_anc = QtGui.QLabel("Latitude (+=N ; -=S)")
        self.lineEditLatitude_anc = QtGui.QLineEdit()
        layoutAnchoring.addRow(self.latLabel_anc,self.lineEditLatitude_anc)
        self.longLabel_anc = QtGui.QLabel("Longitude (+=E ; -=W)")
        self.lineEditLongitude_anc = QtGui.QLineEdit()
        layoutAnchoring.addRow(self.longLabel_anc,self.lineEditLongitude_anc)
        self.anchor_mBox = QtGui.QDialogButtonBox(self)
        self.anchor_setButton = self.anchor_mBox.addButton("Get coordinates from map", QtGui.QDialogButtonBox.AcceptRole)
        self.anchor_gpsInputButton = self.anchor_mBox.addButton("Set anchor position", QtGui.QDialogButtonBox.AcceptRole)
        #self.anchor_cancelButton = self.anchor_mBox.addButton("Cancel", QtGui.QDialogButtonBox.RejectRole)
        self.anchor_mBox.clicked.connect(self.set_anchor_gt_position)
        layoutAnchoring.addRow(self.anchor_mBox)
        
        self.groupBoxAnchoring.setLayout(layoutAnchoring)
        self.anchor_cancelButton = QtGui.QPushButton("Cancel")
        self.anchor_doneButton = QtGui.QPushButton("Done")
        self.anchor_doneButton.setEnabled(False)
        layout.addRow(self.groupBoxAnchoring)
        layout_hor = QtGui.QHBoxLayout()
        layout_hor.addItem(QtGui.QSpacerItem(40,20, QtGui.QSizePolicy.Expanding,QtGui.QSizePolicy.Minimum))
        layout_hor.addWidget(self.anchor_cancelButton)
        layout_hor.addWidget(self.anchor_doneButton)
        layout.addRow(layout_hor)
        self.anchor_dialog.setLayout(layout)
        
        

        #Signals
        self.signal_error_set_map.connect(self.error_set_map)
        self.connect(self.update_timer, QtCore.SIGNAL("timeout()"), self.process_results)
        self.connect(self.gui.pushButtonChat, QtCore.SIGNAL("clicked()"), self.send_chat)
        self.connect(self.gui.pushButtonSelfLocalization, QtCore.SIGNAL("clicked()"), self.start_selfloc)
        self.connect(self.gui.pushButtonDMDS, QtCore.SIGNAL("clicked()"), self.start_selfloc_loop)
        self.connect(self.gui.pushButtonChat, QtCore.SIGNAL("clicked()"), self.send_chat)
        self.connect(self.gui.pushButtonRunReceivers, QtCore.SIGNAL("clicked()"), self.start_correlation)
        self.connect(self.gui.pushButtonRunReceiversLoop, QtCore.SIGNAL("clicked()"), self.start_correlation_loop)
        self.connect(self.gui.pushButtonStopReceiversLoop, QtCore.SIGNAL("clicked()"), self.stop_loop)
        self.connect(self.gui.pushButtonSetCalibration, QtCore.SIGNAL("clicked()"), self.set_calibration)
        self.connect(self.gui.checkBoxAutocalibrate, QtCore.SIGNAL("clicked()"), self.set_auto_calibrate)
        self.connect(self.gui.calibrationAverageSpin, QtCore.SIGNAL("valueChanged(int)"), self.set_calibration_average)
        self.connect(self.gui.spinBoxSampleAverage, QtCore.SIGNAL("valueChanged(int)"), self.set_sample_average)
        self.connect(self.gui.spinBoxAnchorAverage, QtCore.SIGNAL("valueChanged(int)"), self.set_anchor_average)
        self.connect(self.gui.spinBoxNumAnchors, QtCore.SIGNAL("valueChanged(int)"), self.set_num_anchors)
        self.connect(self.gui.spinBoxTransmitGain, QtCore.SIGNAL("valueChanged(int)"), self.set_tx_gain)
        self.connect(self.gui.spinBoxAverageLength, QtCore.SIGNAL("valueChanged(int)"), self.set_location_average_length)
        self.connect(self.gui.spinBoxMeasurementNoise, QtCore.SIGNAL("valueChanged(double)"), self.set_measurement_noise)
        self.connect(self.gui.spinBoxMaxIt, QtCore.SIGNAL("valueChanged(int)"), self.set_max_it)
        self.connect(self.gui.doubleSpinBoxDynamic, QtCore.SIGNAL("valueChanged(double)"), self.set_target_dynamic)
        self.connect(self.gui.doubleSpinBoxAlpha, QtCore.SIGNAL("valueChanged(double)"), self.set_alpha)
        self.connect(self.gui.doubleSpinBoxInitStress, QtCore.SIGNAL("valueChanged(double)"), self.set_init_stress)
        self.connect(self.gui.spinBoxMaxAcc, QtCore.SIGNAL("valueChanged(double)"), self.set_max_acc)
        self.connect(self.gui.spinBoxTrackPlotLength, QtCore.SIGNAL("valueChanged(int)"), self.set_trackplot_length)
        self.connect(self.gui.pushButtonUpdate, QtCore.SIGNAL("clicked()"), self.update_receivers)
        self.connect(self.gui.pushButtonLocalize, QtCore.SIGNAL("clicked()"), self.localize)
        self.connect(self.gui.pushButtonLocalizeContinuous, QtCore.SIGNAL("clicked()"), self.localize_loop)
        self.connect(self.gui.pushButtonLocalizeStop, QtCore.SIGNAL("clicked()"), self.stop_loop)
        self.connect(self.gui.checkBoxFFT1, QtCore.SIGNAL("clicked()"), partial(self.refresh_plot,1))
        self.connect(self.gui.checkBoxFFT2, QtCore.SIGNAL("clicked()"), partial(self.refresh_plot,2))
        self.connect(self.gui.checkBoxFFT3, QtCore.SIGNAL("clicked()"), partial(self.refresh_plot,3))
        self.connect(self.gui.checkBoxCorrelation, QtCore.SIGNAL("clicked()"), self.refresh_correlation)
        self.connect(self.gui.frequencySpin, QtCore.SIGNAL("valueChanged(double)"), self.set_frequency)
        self.connect(self.gui.bwSpin, QtCore.SIGNAL("valueChanged(double)"), self.set_bw)
        self.connect(self.gui.sampRateSpin, QtCore.SIGNAL("valueChanged(double)"), self.set_samp_rate)
        self.connect(self.gui.interpolationSpin, QtCore.SIGNAL("valueChanged(int)"), self.set_interpolation)
        self.connect(self.gui.checkBoxSplineInt, QtCore.SIGNAL("clicked()"), self.set_correlation_interpolation)
        self.connect(self.gui.loOffsetSpin, QtCore.SIGNAL("valueChanged(double)"), self.set_lo_offset)
        self.connect(self.gui.samplesToReceiveSpin, QtCore.SIGNAL("valueChanged(int)"), self.set_samples_to_receive)
        self.connect(self.gui.frequencyCalibrationSpin, QtCore.SIGNAL("valueChanged(double)"), self.set_frequency_calibration)
        self.connect(self.gui.bwCalibrationSpin, QtCore.SIGNAL("valueChanged(double)"), self.set_bw_calibration)
        self.connect(self.gui.loOffsetCalibrationSpin, QtCore.SIGNAL("valueChanged(double)"), self.set_lo_offset_calibration)
        self.connect(self.gui.samplesToReceiveCalibrationSpin, QtCore.SIGNAL("valueChanged(int)"), self.set_samples_to_receive_calibration)
        self.connect(self.gui.comboBoxRefReceiver, QtCore.SIGNAL("currentIndexChanged(int)"), self.set_ref_receiver)
        self.connect(self.gui.comboBoxReferenceSelection, QtCore.SIGNAL("currentIndexChanged(int)"), self.set_reference_selection)
        self.connect(self.gui.comboBoxFilteringType, QtCore.SIGNAL("currentIndexChanged(int)"), self.set_filtering_type)
        self.connect(self.gui.comboBoxMotionModel, QtCore.SIGNAL("currentIndexChanged(int)"), self.set_motion_model)
        self.connect(self.gui.comboBoxMapType, QtCore.SIGNAL("currentIndexChanged(int)"), self.set_map_type)
        self.connect(self.gui.comboBoxCoordinatesType, QtCore.SIGNAL("currentIndexChanged(int)"), self.set_coordinates_type)
        self.connect(self.gui.pushButtonSetMapFile, QtCore.SIGNAL("clicked()"), self.set_map_file)
        self.connect(self.gui.pushButtonSetBbox, QtCore.SIGNAL("clicked()"), self.set_bbox)
        self.connect(self.gui.checkBoxRecordResults, QtCore.SIGNAL("clicked()"), self.set_record_results)
        self.connect(self.gui.checkBoxRecordSamples, QtCore.SIGNAL("clicked()"), self.set_record_samples)
        self.connect(self.gui.acquisitionTimeSpin, QtCore.SIGNAL("valueChanged(double)"), self.set_acquisition_time)
        self.connect(self.gui.gridBasedCheckBox, QtCore.SIGNAL("clicked()"), self.set_grid_based_active)
        self.shortcut_start = QtGui.QShortcut(Qt.QKeySequence("Ctrl+S"), self.gui)
        self.shortcut_stop = QtGui.QShortcut(Qt.QKeySequence("Ctrl+C"), self.gui)
        self.shortcut_exit = QtGui.QShortcut(Qt.QKeySequence("Ctrl+D"), self.gui)
        self.connect(self.shortcut_exit, QtCore.SIGNAL("activated()"), self.gui.close)
        self.connect(self.anchor_cancelButton, QtCore.SIGNAL("clicked()"), self.cancel_all_selfloc)
        self.connect(self.anchor_doneButton, QtCore.SIGNAL("clicked()"), self.anchor_dialog.accept)

        # Grid based signals
        self.connect(self.gui.spinGridResolution, QtCore.SIGNAL("valueChanged(double)"), self.set_TDOA_grid_based_resolution)
        self.connect(self.gui.spinGridNumCompSamps, QtCore.SIGNAL("valueChanged(int)"), self.set_TDOA_grid_based_num_samples)
        #set the Spin Box value for track plot length in the gui. Integrated at this point, because it is not needed in fusion center.
        self.set_gui_trackplot_length(self.trackplot_length)
        # start update timer
        self.update_timer.start(33)
        self.timer_register = threading.Thread(target = self.register_gui)
        self.timer_register.daemon = True
        self.timer_register.start()

    def remove_trackplots(self):
        if self.track_plot:
            self.track_plot.pop(0).remove()
        self.queue_tx_coordinates_kalman = deque()
        self.queue_tx_coordinates = deque()

    def calibration_loop(self, status):
        if status:
            self.calibration_setButton.setEnabled(False)
            self.calibration_gpsInputButton.setEnabled(False)
        else:
            self.calibration_setButton.setEnabled(True)
            self.calibration_gpsInputButton.setEnabled(True)

    def set_anchor_position(self, position):
        self.anchor_setButton.setEnabled(True)
        self.anchor_gpsInputButton.setEnabled(True)
        self.anchor_positions.append(position)


    def calibration_status(self, status):
        if status:
            self.gui.pushButtonSetCalibration.setStyleSheet("background-color: green")
        else:
            self.gui.pushButtonSetCalibration.setStyleSheet("background-color: red")

    def set_tx_calibration(self, button):
        if button.text() == "Cancel":
            self.rpc_manager.request("remove_calibration")
            self.calibration_dialog.reject()
        elif button.text() == "Get coordinates from map" :
            if hasattr(self, "zp"):
                self.setting_calibration = True
                self.zp.enabled = False
        elif button.text() == "Set calibration" :
            # calibrate with gps coordinates from line inputs
            latitude = float(self.lineEditLatitude_cal.text())
            longitude = float(self.lineEditLongitude_cal.text())
            self.rpc_manager.request("calibrate",[self.basemap(longitude,latitude)])
            self.calibration_dialog.accept()

    def start_anchoring(self):
        self.num_anchor_position = 0
        self.anchor_positions = []
        self.curr_anchor_spin.setValue(1)
        self.pushButtonOK.setEnabled(True)

    def start_anchoring_loop(self):
        self.pushButtonOK.setEnabled(False)
        if self.num_anchor_position < self.num_anchors:
            self.rpc_manager.request("start_anchoring_loop",[self.num_anchor_position])
            self.anchor_setButton.setEnabled(False)
            self.anchor_gpsInputButton.setEnabled(False)
        else:
            self.pushButtonDMDS.setEnabled(True)
            self.curr_anchor_spin.setEnabled(True)

    def cancel_all_selfloc(self):
        self.rpc_manager.request("stop_selfloc")
        #reset all buttons to prevent deadlock
        self.pushButtonDMDS.setEnabled(True)
        self.pushButtonOK.setEnabled(True)
        self.anchor_setButton.setEnabled(True)
        self.anchor_gpsInputButton.setEnabled(True)
        self.curr_anchor_spin.setEnabled(True)
        self.anchor_dialog.reject()

    def set_anchor_gt_position(self, button):
        if button.text() == "Get coordinates from map" :
            if hasattr(self, "zp"):
                self.setting_calibration = True
                self.zp.enabled = False
        elif button.text() == "Set anchor position" :
            # calibrate with gps coordinates from line inputs
            latitude = float(self.lineEditLatitude_anc.text())
            longitude = float(self.lineEditLongitude_anc.text())
            self.rpc_manager.request("set_anchor_gt_position",[self.basemap(longitude,latitude)])
            self.pushButtonOK.setEnabled(True)
            self.anchor_setButton.setEnabled(False)
            self.anchor_gpsInputButton.setEnabled(False)
            self.num_anchor_position += 1
            if self.num_anchor_position == num_anchors:
                self.anchor_doneButton.setEnabled(True)
            self.curr_anchor_spin.setValue(self.num_anchor_position)
            
    def set_trackplot_length(self):
        self.trackplot_length = self.gui.spinBoxTrackPlotLength.value()
        #Directly update value
        self.set_gui_trackplot_length(self.trackplot_length)
    
    def set_calibration_average(self):
        self.calibration_average = self.gui.calibrationAverageSpin.value()
        self.rpc_manager.request("set_calibration_average",[self.calibration_average])

    def set_sample_average(self):
        self.sample_average = self.gui.spinBoxSampleAverage.value()
        self.rpc_manager.request("set_sample_average",[self.sample_average])

    def set_anchor_average(self):
        self.anchor_average = self.gui.spinBoxAnchorAverage.value()
        self.rpc_manager.request("set_anchor_average",[self.anchor_average])

    def set_num_anchors(self):
        self.num_anchors = self.gui.spinBoxNumAnchors.value()
        self.rpc_manager.request("set_num_anchors",[self.num_anchors])

    def set_tx_gain(self):
        self.tx_gain = self.gui.spinBoxTransmitGain.value()
        self.rpc_manager.request("set_tx_gain",[self.tx_gain])

    def set_location_average_length(self):
        self.location_average_length = self.gui.spinBoxAverageLength.value()
        self.rpc_manager.request("set_location_average_length",[self.location_average_length])
        
    def set_target_dynamic(self):
        self.target_dynamic = self.gui.doubleSpinBoxDynamic.value()
        self.rpc_manager.request("set_target_dynamic",[self.target_dynamic])

    def set_max_it(self):
        self.max_it = self.gui.spinBoxMaxIt.value()
        self.rpc_manager.request("set_max_it",[self.max_it])

    def set_alpha(self):
        self.alpha = self.gui.doubleSpinBoxAlpha.value()
        self.rpc_manager.request("set_alpha",[self.alpha])

    def set_init_stress(self):
        self.init_stress = self.gui.doubleSpinBoxInitStress.value()
        self.rpc_manager.request("set_init_stress",[self.init_stress])
        
    def set_max_acc(self):
        self.max_acc = self.gui.spinBoxMaxAcc.value()
        self.rpc_manager.request("set_max_acc",[self.max_acc])
        
    def set_measurement_noise(self):
        self.measurement_noise = self.gui.spinBoxMeasurementNoise.value()
        self.rpc_manager.request("set_measurement_noise",[self.measurement_noise])

    def set_calibration(self):
        # run system multiple times to average calibration
        calibration_started = self.rpc_manager.request("calibration_loop", [self.frequency, self.lo_offset, self.samples_to_receive, self.calibration_average])
        if calibration_started:
            self.calibration_dialog.show()

    def init_map(self, bbox, map_type, map_file, coordinates_type):
        self.bbox = bbox
        self.map_type = map_type
        self.map_file = map_file
        self.coordinates_type = coordinates_type
        threading.Thread(target = self.set_map, args = [bbox]).start()

    @QtCore.pyqtSlot()
    def error_set_map(self):
        msg = QtGui.QMessageBox(self)
        msg.setIcon(QtGui.QMessageBox.Warning)
        msg.setText("Map file not found. Please select a correct file.")
        msg.setWindowTitle("Error loading map")
        msg.show()

    def set_map(self, bbox):
        self.gui.lineEditLeft.setText(str(bbox[0]))
        self.gui.lineEditLeft.setCursorPosition(0)
        self.gui.lineEditBottom.setText(str(bbox[1]))
        self.gui.lineEditBottom.setCursorPosition(0)
        self.gui.lineEditRight.setText(str(bbox[2]))
        self.gui.lineEditRight.setCursorPosition(0)
        self.gui.lineEditTop.setText(str(bbox[3]))
        self.gui.lineEditTop.setCursorPosition(0)

        inProj = Proj(init='epsg:4326')
        outProj = Proj(init='epsg:3857')
        x0, y0 = transform(inProj,outProj,bbox[0],bbox[1])
        x1, y1 = transform(inProj,outProj,bbox[2],bbox[3])
        x = x1 - x0
        y = y1 - y0
        scale = math.ceil(math.sqrt(abs(x*y/0.3136))) * 2
        
        # check if OSM is available at first 

        if self.map_type == "Online" and check_OSM():

            print "Setting online map", bbox
            print "+".join(str(j).replace(".",",") for j in bbox)
            # search for existing map for this bounding box
            if not any(i.find("+".join(str(j).replace(".",",") for j in bbox))!= -1 for i in os.listdir("../maps/") ):
                # request only if no map can be found
                r = requests.get("http://render.openstreetmap.org/cgi-bin/export?bbox=" + str(bbox)[1:-1] + "&scale=" + str(scale) + "&format=png", stream=True)
                if r.status_code == 200:
                    img = Image.open(StringIO(r.content))
                    if not os.path.exists("../maps"):
                            os.makedirs("../maps")
                    img.save("../maps/map"+"+".join(str(i).replace(".",",") for i in bbox)+".png")
                else:
                    self.signal_error_set_map.emit()
            else:
                # if available, open offline map instead
                img = Image.open("../maps/map"+"+".join(str(i).replace(".",",") for i in bbox)+".png")
            


        else:
            print "Setting offline map", self.map_file
            try:
                img = Image.open(self.map_file)
            except:
                self.signal_error_set_map.emit()
                return

        if hasattr(self, "ax"):
            self.figure.delaxes(self.ax)
        self.ax = self.figure.add_subplot(111, xlim=(x0,x1), ylim=(y0,y1), autoscale_on=False)

        #
        # create basemap
        #

        # get reference UTM grid
        lon = bbox[0]
        lat = bbox[1]
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
            self.basemap = Basemap(llcrnrlon=bbox[0], llcrnrlat=bbox[1],
                          urcrnrlon=bbox[2], urcrnrlat=bbox[3],
                          projection='tmerc', ax=self.ax, lon_0=lon_0, lat_0=lat_0)
        else:
            self.basemap = Basemap(width=bbox[2], height=bbox[3],
                          lon_0=0,lat_0=0,
                          projection='tmerc', ax=self.ax,)

        if hasattr(self, "image"):
            self.image.remove()
        self.image = self.basemap.imshow(img, interpolation='lanczos', origin='upper')

        self.zp = gui_helpers.ZoomPan()
        figZoom = self.zp.zoom_factory(self.ax, base_scale = 1.5)
        figPan = self.zp.pan_factory(self.ax)

        self.figure.tight_layout(pad=0)
        #self.figure.patch.set_visible(False)
        self.ax.axis('off')
        self.canvas.draw()
        self.hyperbolas = {}

        self.pending_receivers_to_plot = True

    def init_plot(self, qwtPlot):
        title = Qwt.QwtText("Samples")
        title.setFont(Qt.QFont("Helvetica", 10, Qt.QFont.Bold))
        qwtPlot.setAxisTitle(Qwt.QwtPlot.xBottom, title)
        title = Qwt.QwtText("Amplitude")
        title.setFont(Qt.QFont("Helvetica", 10, Qt.QFont.Bold))
        qwtPlot.setAxisTitle(Qwt.QwtPlot.yLeft, title)
        qwtPlot.setAxisScale(Qwt.QwtPlot.xBottom, 0, self.samples_to_receive * self.sample_interpolation)
        pen = Qt.QPen(Qt.Qt.DotLine)
        pen.setColor(Qt.Qt.black)
        pen.setWidth(0)
        grid = Qwt.QwtPlotGrid()
        grid.setPen(pen)
        grid.attach(qwtPlot)

    def register_gui(self):
        first = True
        while(True):
            # register receiver [hostname, usrp_serial, rx_id]
            try:
                bbox, map_type, map_file, coordinates_type = self.rpc_manager.request("register_gui",[self.ip_addr, self.hostname, options.id_gui, first])
                if first and bbox != None:
                    self.bbox = bbox
                    self.init_map(bbox, map_type, map_file, coordinates_type)
                    first = False
                    print "Registerd to fusion center at "+str(self.ip_addr)
            except:
                print "Warning: fusion center not reachable, retrying"
            time.sleep(10)

    def sync_position(self, serial, coordinates):
        if serial not in self.receivers.keys():
            return
        receiver = self.receivers[serial]
        receiver.coordinates = coordinates[0],coordinates[1]
        # remove point from map if was set
        if hasattr(receiver, "scatter"):
            receiver.scatter.remove()
            receiver.annotation.remove()
        if hasattr(self, "ax"):
            # save scattered point into receiver properties
            receiver.scatter = self.ax.scatter(coordinates[0], coordinates[1],linewidths=2, marker='x', c='b', s=200, alpha=0.9)
            # set annotation Rxi
            text = ("Rx" + str(self.receivers.keys().index(serial) + 1)
                        + " "
                        + str(np.round(receiver.coordinates,2)))
                        
            if serial != self.ref_receiver:
                receiver.annotation = self.ax.annotate(text, receiver.coordinates,fontweight='bold',bbox=dict(facecolor='w', alpha=0.9, zorder=20))
            else:
                receiver.annotation = self.ax.annotate(text, receiver.coordinates,fontweight='bold',bbox=dict(facecolor='r', alpha=0.9, zorder=20))
            self.canvas.draw()
        else:
            # ax not rendered yet, so update position when available
            self.pending_receivers_to_plot = True

    def sync_position_selfloc(self, coordinates_procrustes_x,coordinates_procrustes_y):
        coordinates_procrustes = np.vstack((coordinates_procrustes_x,coordinates_procrustes_y)).T
        print coordinates_procrustes
        for idx, serial in enumerate(self.receivers.keys()):
            coordinates = coordinates_procrustes[idx].tolist()
            receiver = self.receivers[serial]
            receiver.coordinates_selfloc = coordinates[0],coordinates[1]
            # remove point from map if was set
            if hasattr(receiver, "scatter_selfloc"):
                receiver.scatter_selfloc.remove()
                receiver.annotation_selfloc.remove()
            if hasattr(self, "ax"):
                # save scattered point into receiver properties
                receiver.scatter_selfloc = self.ax.scatter(coordinates[0], coordinates[1],linewidths=2, marker='x', c='b', s=200, alpha=0.9)
                # set annotation Rxi
                text = ("Rx" + str(self.receivers.keys().index(serial) + 1)
                            + " "
                            + str(np.round(receiver.coordinates_selfloc,2)))
                            

                receiver.annotation_selfloc = self.ax.annotate(text, receiver.coordinates_selfloc,fontweight='bold',bbox=dict(facecolor='b', alpha=0.9, zorder=20))

                self.canvas.draw()
            else:
                # ax not rendered yet, so update position when available
                self.pending_receivers_to_plot = True

    def set_gps_position(self, serial, coordinates):
        if not hasattr(self, "basemap"):
            return
        receiver = self.receivers[serial]
        receiver.coordinates_gps = (coordinates[0],coordinates[1])
        # remove point from map if was set
        if hasattr(receiver, "scatter_gps"):
            receiver.scatter_gps.remove()
            receiver.annotation_gps.remove()
        if hasattr(self, "ax"):
            # save scattered point into receiver properties
            receiver.scatter_gps = self.ax.scatter(receiver.coordinates_gps[0], receiver.coordinates_gps[1],linewidths=2, marker='x', c='b', s=200, alpha=0.9)
            # set annotation Rxi
            text_gps = ("Rx" + str(self.receivers.keys().index(serial) + 1) 
                        + " " 
                        + str(np.round(receiver.coordinates_gps,2)))
                        
            if serial != self.ref_receiver:
                receiver.annotation_gps = self.ax.annotate(text_gps, receiver.coordinates_gps,fontweight='bold',bbox=dict(facecolor='#33ff33', alpha=0.9, zorder=20))
            else:
                receiver.annotation_gps = self.ax.annotate(text_gps, receiver.coordinates_gps,fontweight='bold',bbox=dict(facecolor='c', alpha=0.9, zorder=20))
            self.canvas.draw()      
        else:
            # ax not rendered yet, so update position when available
            self.pending_receivers_to_plot = True

    def set_tx_position(self, transmitter_positions):
        if not hasattr(self, "basemap"):
            return
        if self.hyperbolas.has_key("tdoa"):
            for h in self.hyperbolas["tdoa"]:
                h.remove()
        self.hyperbolas["tdoa"] = self.plot_hyperbolas()
        # check results for both algorithms from dict {<algorithm>:<all results>}
        for algorithm in transmitter_positions.items():
            # exception for first value
            if not self.transmitter_positions.has_key(algorithm[0]):
                # save value in queue
                self.queue_tx_coordinates.append(transmitter_position(algorithm[1]).coordinates["coordinates"])
                if self.filtering_type == "Moving average":
                    self.transmitter_positions[algorithm[0]] = transmitter_position(algorithm[1]["average_coordinates"])
                elif self.filtering_type == "Kalman filter":
                    self.transmitter_positions[algorithm[0]] = transmitter_position(algorithm[1]["kalman_coordinates"])
                    # save kalman value in queue
                    self.queue_tx_coordinates_kalman.append(self.transmitter_positions[algorithm[0]].coordinates)
                    # remove more than one value in plot if track length is changed -> while instead of if
                    while len(self.queue_tx_coordinates_kalman) > self.trackplot_length:
                        self.queue_tx_coordinates_kalman.popleft()
                else:
                    self.transmitter_positions[algorithm[0]] = transmitter_position(algorithm[1]["coordinates"])

                while len(self.queue_tx_coordinates) > self.trackplot_length:
                    self.queue_tx_coordinates.popleft()
            else:
                self.queue_tx_coordinates.append(algorithm[1]["coordinates"])
                if self.filtering_type == "Moving average":
                    self.transmitter_positions[algorithm[0]].coordinates = algorithm[1]["average_coordinates"]
                elif self.filtering_type == "Kalman filter":
                    self.transmitter_positions[algorithm[0]].coordinates = algorithm[1]["kalman_coordinates"]
                    # save value in queue
                    self.queue_tx_coordinates_kalman.append(self.transmitter_positions[algorithm[0]].coordinates)
                    # remove more than one value in plot if track length is changed -> while instead of if
                    while len(self.queue_tx_coordinates_kalman) > self.trackplot_length:
                        self.queue_tx_coordinates_kalman.popleft()
                else:
                    self.transmitter_positions[algorithm[0]].coordinates = algorithm[1]["coordinates"]            
                while len(self.queue_tx_coordinates) > self.trackplot_length:
                    self.queue_tx_coordinates.popleft()
            estimated_position = self.transmitter_positions[algorithm[0]]
            if hasattr(estimated_position, "scatter"):
                estimated_position.scatter.remove()
                estimated_position.annotation.remove()
            if hasattr(self, "track_plot"):
                # revove plot item; procedure differs from scatter item!
                if any(self.track_plot):
                    self.track_plot.pop(0).remove()

            if hasattr(self, "ax"):
                
                # save scattered point into receiver properties
                estimated_position.scatter = self.ax.scatter(estimated_position.coordinates[0], estimated_position.coordinates[1],linewidths=2,  marker='x', c='red', s=200, alpha=0.9, zorder=20)
                # plot target track (last 10 positions) if Kalman Filter is enabled:
                if self.filtering_type == "Kalman filter":
                    prev_coordinates_kalman = np.array(self.queue_tx_coordinates_kalman) 
                    self.track_plot = self.ax.plot(prev_coordinates_kalman [:,0], prev_coordinates_kalman [:,1], c='magenta',alpha=0.9, zorder=20,linestyle="-",linewidth=2)
                else:
                    prev_coordinates = np.array(self.queue_tx_coordinates) 
                    self.track_plot = self.ax.plot(prev_coordinates[:,0], prev_coordinates[:,1], c='magenta',alpha=0.9, zorder=20,linestyle="-",linewidth=2)
                # set annotation Rxi
                text = (algorithm[0] + " " 
                                    + str(np.round(estimated_position.coordinates,2)))
                estimated_position.annotation = self.ax.annotate(text, estimated_position.coordinates,fontweight='bold',bbox=dict(facecolor='w', alpha=0.9), zorder=20)
                if algorithm[0] == "chan":
                    if self.hyperbolas.has_key("chan"):
                        for h in self.hyperbolas["chan"]:
                            h.remove()
                    self.hyperbolas["chan"] = self.plot_hyperbolas(algorithm[1]["coordinates"],"blue")
                if algorithm[0] == "grid_based":
                    if self.hyperbolas.has_key("grid_based"):
                        for h in self.hyperbolas["grid_based"]:
                            h.remove()
                    self.plot_grid(algorithm[1]["grid"])
                    self.hyperbolas["grid_based"] = self.plot_hyperbolas(algorithm[1]["coordinates"],"green")
                self.canvas.draw()
            else:
                # ax not rendered yet, so update position when available
                self.pending_receivers_to_plot = True

        if not self.grid_based_active:
            if hasattr(self,"grid"):
                self.grid.remove()
                del self.grid
            if self.hyperbolas.has_key("grid_based"):
                for h in self.hyperbolas["grid_based"]:
                    h.remove()
                del self.hyperbolas["grid_based"]
            if self.transmitter_positions.has_key("grid_based"):
                estimated_position = self.transmitter_positions["grid_based"]
                estimated_position.scatter.remove()
                self.track_plot.remove()
                estimated_position.annotation.remove()
                del self.transmitter_positions["grid_based"]
            self.canvas.draw()


    def plot_hyperbolas(self, pos_tx=None, c="red"):
        pos_rx = []
        hyperbolas = []
        for receiver in self.receivers:
            if self.receivers[receiver].selected_position == "manual":
                if receiver == self.ref_receiver:
                    pos_rx.insert(0,self.receivers[receiver].coordinates)
                else:
                    pos_rx.append(self.receivers[receiver].coordinates)
            elif self.receivers[receiver].selected_position == "GPS":
                if receiver == self.ref_receiver:
                    pos_rx.insert(0,self.receivers[receiver].coordinates_gps)
                else:
                    pos_rx.append(self.receivers[receiver].coordinates_gps)
            else:
                if receiver == self.ref_receiver:
                    pos_rx.insert(0,self.receivers[receiver].coordinates_selfloc)
                else:
                    pos_rx.append(self.receivers[receiver].coordinates_selfloc)
        for i in range(1,len(pos_rx)):
            if pos_tx is None:
                hyperbola = self.get_hyperbola([pos_rx[0],pos_rx[i]], pos_tx, self.results["delay"][i-1])
            else:
                hyperbola = self.get_hyperbola([pos_rx[0],pos_rx[i]], pos_tx)

            if len(hyperbolas) < i:
                h = self.ax.scatter(hyperbola[0],hyperbola[1],c=c,s=10,zorder=10,edgecolors='none')
                hyperbolas.append(h)
            else:
                hyperbolas[i-1] = self.ax.scatter(hyperbola[0],hyperbola[1],c=c,zorder=10,edgecolors='none')
        return hyperbolas


    def get_hyperbola(self, pos_rx, pos_tx=None, delay=None):
        # Redefine receivers position and signals so that the signal arrives first
        # to the nearest receiver.
        pos_rx = np.array(pos_rx)
        if delay is None:
            if pos_tx is not None:
                pos_tx = np.array(pos_tx)
                d1 = np.linalg.norm(pos_rx[0]-pos_tx)
                d2 = np.linalg.norm(pos_rx[1]-pos_tx)
                # calculate expected delay for this position to use the same methods then for given delay
                delay = (d2 - d1)*(self.samp_rate * self.sample_interpolation)/ 299700000
            else:
                sys.exit("Neither transmitter position nor delay estimate are given!")
        if delay<0:
            pos_rx = np.flipud(pos_rx)
        
        # Baseline distance between sensors
        B = np.linalg.norm(pos_rx[1]-pos_rx[0])

        # Asign alpha0 angle depending on the sensors relative position
        try:
            if pos_rx[1][0]>pos_rx[0][0]:
                alpha0 = -np.arcsin((pos_rx[1][1]-pos_rx[0][1])/B)
            else:
                alpha0 = np.arcsin((pos_rx[1][1]-pos_rx[0][1])/B)+np.pi
        except:
            print "receiver positions are invalid!"

        # Calculate parametric points of the hyperbola
        Hx = []
        Hy = []
        delta_rx1_rx2 = delay / (self.samp_rate * self.sample_interpolation) * 299700000
        [max_x,max_y]=np.round(self.basemap(self.bbox[2],self.bbox[3]))
        for alpha in np.arange(0,2*np.pi,0.005):
            h = np.array(pos_rx[0])-(B*B-delta_rx1_rx2*delta_rx1_rx2)/(2*(-delta_rx1_rx2-B*np.cos(alpha)))*np.array([np.cos(alpha-alpha0),np.sin(alpha-alpha0)])
            # Get only points in the region of interest
            distance_1 = np.linalg.norm(h-pos_rx[0])
            distance_2 = np.linalg.norm(h-pos_rx[1])
            if ((distance_1 <= distance_2) and (-max_x < h[0] < 2*max_x) and (-max_y < h[1] < 2*max_y)):
                Hx.append(h[0])
                Hy.append(h[1])
        return [Hx,Hy]

    def plot_grid(self, s):
        if hasattr(s, "__getitem__"):
            if hasattr(self,"grid"):
                self.grid.remove()
        
            self.grid = self.ax.pcolor(np.array(s[0]),np.array(s[1]),np.array(s[2]), cmap='coolwarm', alpha=0.7)

    def get_gps_coordinates(self,mouse_event):
        if self.setting_calibration:
            map_long, map_lat = self.basemap(mouse_event.xdata,mouse_event.ydata,inverse=True)
            self.lineEditLongitude_cal.setText(str(map_long))
            self.lineEditLatitude_cal.setText(str(map_lat))
            self.lineEditLongitude_anc.setText(str(map_long))
            self.lineEditLatitude_anc.setText(str(map_lat))
            self.setting_calibration = False
            self.zp.enabled = True

    def set_position(self, mouse_event):
        if self.setting_pos_receiver is not "" and self.zp.enabled == False:
            receiver = self.receivers[self.setting_pos_receiver]
            self.rpc_manager.request("sync_position",[self.setting_pos_receiver, (mouse_event.xdata,mouse_event.ydata)])
            #self.rpc_manager.request("get_gui_gps_position",[self.setting_pos_receiver])
            self.setting_pos_receiver = ""
            self.zp.enabled = True

    def manage_position(self,button):
        if button.text() == "Cancel":
            self.position_dialog.reject()
        elif button.text() == "From map" :
            if hasattr(self, "zp"):
                self.zp.enabled = False
                self.position_dialog.accept()

        elif button.text() == "Set" :
            # calibrate with gps coordinates from line inputs
            latitude = float(self.lineEditLatitude_pos.text())
            longitude = float(self.lineEditLongitude_pos.text())
            altitude = float(self.lineEditAltitude_pos.text())
            # program gps coordinates for receiver that corresponds to the clicked button
            # see gui_helpers PushButtonPositionDelegate
            print "receiver:",self.setting_pos_receiver
            if self.setting_pos_receiver is not "": 
                self.rpc_manager.request("program_gps_receiver",[self.setting_pos_receiver,latitude, longitude, altitude])
                self.position_dialog.accept()
    def switch_transmitter(self):
        self.rpc_manager.request("switch_transmitter")

    def start_selfloc(self):
        self.anchor_dialog.show()        
        self.pushButtonOK.setEnabled(False)
        self.anchor_setButton.setEnabled(False)
        self.anchor_gpsInputButton.setEnabled(False)
        self.curr_anchor_spin.setEnabled(False)
    
    def start_selfloc_loop(self):
        self.pushButtonDMDS.setEnabled(False)
        self.rpc_manager.request("start_selfloc_loop")

    def stop_transmitter(self):
        self.start_anchoring()
        self.rpc_manager.request("stop_transmitter")
            
    def new_chat(self, chat):
        self.chat_pending = True
        self.chats = self.chats + chat

    def send_chat(self):
        if len(self.lineEdit.text()) > 0:
            chat = self.gui_serial + ": " + str(self.lineEdit.text()) + "\n"
            self.rpc_manager.request("forward_chat", [chat])
            self.lineEdit.setText("")

    def set_frequency(self):
        self.frequency = self.gui.frequencySpin.value()*1e6
        self.rpc_manager.request("set_frequency",[self.frequency])

    def set_lo_offset(self):
        self.lo_offset = self.gui.loOffsetSpin.value()*1e6
        self.rpc_manager.request("set_lo_offset",[self.lo_offset])

    def set_samples_to_receive(self):
        self.samples_to_receive = self.gui.samplesToReceiveSpin.value()
        self.rpc_manager.request("set_samples_to_receive",[self.samples_to_receive])

    def set_samp_rate(self):
        self.samp_rate = self.gui.sampRateSpin.value()*1e6
        self.rpc_manager.request("set_samp_rate",[self.samp_rate])

    def set_bw(self):
        self.bw = self.gui.bwSpin.value()*1e6
        self.rpc_manager.request("set_bw",[self.bw])

    def set_interpolation(self):
        self.sample_interpolation = self.gui.interpolationSpin.value()
        self.rpc_manager.request("set_interpolation",[self.sample_interpolation])

    def set_correlation_interpolation(self):
        self.correlation_interpolation = self.gui.checkBoxSplineInt.isChecked()
        self.rpc_manager.request("set_correlation_interpolation",[self.correlation_interpolation])

    def set_frequency_calibration(self):
        self.frequency_calibration = self.gui.frequencyCalibrationSpin.value()*1e6
        self.rpc_manager.request("set_frequency_calibration",[self.frequency_calibration])

    def set_lo_offset_calibration(self):
        self.lo_offset_calibration = self.gui.loOffsetCalibrationSpin.value()*1e6
        self.rpc_manager.request("set_lo_offset_calibration",[self.lo_offset_calibration])

    def set_samples_to_receive_calibration(self):
        self.samples_to_receive_calibration = self.gui.samplesToReceiveCalibrationSpin.value()
        self.rpc_manager.request("set_samples_to_receive_calibration",[self.samples_to_receive_calibration])

    def set_bw_calibration(self):
        self.bw_calibration = self.gui.bwCalibrationSpin.value()*1e6
        self.rpc_manager.request("set_bw_calibration",[self.bw_calibration])

    def set_gain(self, gain, serial):
        self.rpc_manager.request("set_gain",[gain, serial])

    def set_gain_calibration(self, gain, serial):
        self.rpc_manager.request("set_gain_calibration",[gain, serial])

    def set_antenna(self, antenna, serial):
        self.rpc_manager.request("set_antenna",[antenna, serial])

    def set_selected_position(self, selected_position, serial):
        self.rpc_manager.request("set_selected_position",[selected_position, serial])

    def set_ref_receiver(self):
        self.ref_receiver = self.gui.comboBoxRefReceiver.currentText()
        self.rpc_manager.request("set_ref_receiver",[str(self.ref_receiver)])
        
    def set_reference_selection(self):
        self.reference_selection = self.gui.comboBoxReferenceSelection.currentText()
        self.rpc_manager.request("set_reference_selection",[str(self.reference_selection)])

    def set_filtering_type(self):
        self.filtering_type = self.gui.comboBoxFilteringType.currentText()
        self.rpc_manager.request("set_filtering_type",[str(self.filtering_type)])
        
    def set_motion_model(self):
        self.motion_model = self.gui.comboBoxMotionModel.currentText()
        self.rpc_manager.request("set_motion_model",[str(self.motion_model)])

    def set_map_type(self):
        self.map_type = self.gui.comboBoxMapType.currentText()
        self.rpc_manager.request("set_map_type",[str(self.map_type)])

    def set_map_file(self):
        f = Qt.QFileDialog.getOpenFileName()
        self.gui.lineEditMapFile.setText("../maps/" + f.split("/")[-1])
        self.map_file = str(self.gui.lineEditMapFile.text())
        self.rpc_manager.request("set_map_file",[str(self.map_file)])

    def set_coordinates_type(self):
        self.coordinates_type = self.gui.comboBoxCoordinatesType.currentText()
        self.rpc_manager.request("set_coordinates_type",[str(self.coordinates_type)])

    def set_auto_calibrate(self):
        self.auto_calibrate = self.gui.checkBoxAutocalibrate.isChecked()
        self.rpc_manager.request("set_auto_calibrate",[self.auto_calibrate])

    def set_TDOA_grid_based_resolution(self, resolution):
        self.rpc_manager.request("set_TDOA_grid_based_resolution", [resolution])

    def set_TDOA_grid_based_num_samples(self, num_samples):
        self.rpc_manager.request("set_TDOA_grid_based_num_samples", [num_samples])

    def set_bbox(self):
        bbox = [float(self.gui.lineEditLeft.text()),float(self.gui.lineEditBottom.text()),float(self.gui.lineEditRight.text()),float(self.gui.lineEditTop.text())]
        self.rpc_manager.request("set_bbox",[bbox])

    def set_record_results(self):
        self.rpc_manager.request("set_record_results",[self.gui.checkBoxRecordResults.isChecked()])

    def set_record_samples(self):
        self.rpc_manager.request("set_record_samples",[self.gui.checkBoxRecordSamples.isChecked()])

    def set_acquisition_time(self):
        self.acquisition_time = self.gui.acquisitionTimeSpin.value()
        self.rpc_manager.request("set_acquisition_time",[self.acquisition_time])

    def set_grid_based_active(self):
        self.grid_based_active = self.gui.gridBasedCheckBox.isChecked()
        self.rpc_manager.request("set_grid_based_active",[self.grid_based_active])

    def set_gui_calibration_average(self, calibration_average):
        self.calibration_average = calibration_average
        self.gui.calibrationAverageSpin.setValue(calibration_average)

    def set_gui_num_anchors(self, num_anchors):
        self.num_anchors = num_anchors
        self.gui.spinBoxNumAnchors.setValue(num_anchors)

    def set_gui_sample_average(self, sample_average):
        self.sample_average = sample_average
        self.gui.spinBoxSampleAverage.setValue(sample_average)

    def set_gui_anchor_average(self, anchor_average):
        self.anchor_average = anchor_average
        self.gui.spinBoxAnchorAverage.setValue(anchor_average)

    def set_gui_tx_gain(self, tx_gain):
        self.tx_gain = tx_gain
        self.gui.spinBoxTransmitGain.setValue(tx_gain)

    def set_gui_location_average_length(self, location_average_length):
        self.location_average_length = location_average_length
        self.gui.spinBoxAverageLength.setValue(location_average_length)
        
    def set_gui_measurement_noise(self, measurement_noise):
        self.measurement_noise = measurement_noise
        self.gui.spinBoxMeasurementNoise.setValue(measurement_noise)
        
    def set_gui_target_dynamic(self, target_dynamic):
        self.target_dynamic = target_dynamic
        self.gui.doubleSpinBoxDynamic.setValue(target_dynamic)

    def set_gui_init_stress(self, init_stress):
        self.init_stress = init_stress
        self.gui.doubleSpinBoxInitStress.setValue(init_stress)

    def set_gui_max_it(self, max_it):
        self.max_it = max_it
        self.gui.spinBoxMaxIt.setValue(max_it)

    def set_gui_alpha(self, alpha):
        self.alpha = alpha
        self.gui.doubleSpinBoxAlpha.setValue(alpha)
     
    def set_gui_max_acc(self, max_acc):
        self.max_acc = max_acc
        self.gui.spinBoxMaxAcc.setValue(max_acc)    
        
    def set_gui_trackplot_length(self, trackplot_length):
        self.trackplot_length = trackplot_length
        self.gui.spinBoxTrackPlotLength.setValue(trackplot_length)
        
    def set_gui_frequency(self, frequency):
        self.gui.frequencySpin.setValue(frequency/1e6)

    def set_gui_lo_offset(self, lo_offset):
        self.gui.loOffsetSpin.setValue(lo_offset/1e6)

    def set_gui_samples_to_receive(self, samples_to_receive):
        self.gui.samplesToReceiveSpin.setValue(samples_to_receive)

    def set_gui_samp_rate(self, samp_rate):
        self.gui.sampRateSpin.setValue(samp_rate/1e6)

    def set_gui_bw(self, bw):
        self.gui.bwSpin.setValue(bw/1e6)

    def set_gui_interpolation(self, interpolation):
        self.gui.interpolationSpin.setValue(interpolation)

    def set_gui_correlation_interpolation(self, correlation_interpolation):
        if correlation_interpolation:
            self.gui.checkBoxSplineInt.setChecked(1)
        else:
            self.gui.checkBoxSplineInt.setChecked(0)

    def set_gui_frequency_calibration(self, frequency):
        self.gui.frequencyCalibrationSpin.setValue(frequency/1e6)

    def set_gui_lo_offset_calibration(self, lo_offset):
        self.gui.loOffsetCalibrationSpin.setValue(lo_offset/1e6)

    def set_gui_samples_to_receive_calibration(self, samples_to_receive):
        self.gui.samplesToReceiveCalibrationSpin.setValue(samples_to_receive)

    def set_gui_bw_calibration(self, bw):
        self.gui.bwCalibrationSpin.setValue(bw/1e6)

    def set_gui_gain(self, gain, serial):
        self.tmr.set_gain(gain, serial)

    def set_gui_gain_calibration(self, gain, serial):
        self.tmr.set_gain_calibration(gain, serial)

    def set_gui_antenna(self, antenna, serial):
        self.tmr.set_antenna(antenna, serial)

    def set_gui_selected_position(self, selected_position, serial):
        self.tmrp.set_selected_position(selected_position, serial)

    def set_gui_ref_receiver(self, ref_receiver):
        for i in range(0,len(self.receivers)):
            if self.receivers.keys()[i] == ref_receiver:
                self.gui.comboBoxRefReceiver.setCurrentIndex(i)
              
    def set_gui_reference_selection(self, reference_selection):
        for i in range(0,len(self.reference_selections)):
            if self.reference_selections[i] == reference_selection:
                self.gui.comboBoxReferenceSelection.setCurrentIndex(i)
                
             
    def set_gui_filtering_type(self, filtering_type):
        for i in range(0,len(self.filtering_types)):
            if self.filtering_types[i] == filtering_type:
                self.gui.comboBoxFilteringType.setCurrentIndex(i)
                
                
    def set_gui_motion_model(self, motion_model):
        for i in range(0,len(self.motion_models)):
            if self.motion_models[i] == motion_model:
                self.gui.comboBoxMotionModel.setCurrentIndex(i)

    def set_gui_map_type(self, map_type):
        if map_type == "Online":
            self.gui.comboBoxMapType.setCurrentIndex(0)
            self.gui.comboBoxCoordinatesType.setEnabled(False)
            self.gui.lineEditMapFile.setEnabled(False)
            self.gui.pushButtonSetMapFile.setEnabled(False)
        else:
            self.gui.comboBoxMapType.setCurrentIndex(1)
            self.gui.comboBoxCoordinatesType.setEnabled(True)
            self.gui.lineEditMapFile.setEnabled(True)
            self.gui.pushButtonSetMapFile.setEnabled(True)

    def set_gui_map_file(self, map_file):
        self.map_file = map_file
        self.gui.lineEditMapFile.setText(str(map_file))

    def set_gui_coordinates_type(self, coordinates_type):
        if coordinates_type == "Geographical":
            self.gui.comboBoxCoordinatesType.setCurrentIndex(0)
            self.gui.lineEditLeft.setEnabled(True)
            self.gui.lineEditBottom.setEnabled(True)
        else:
            self.gui.comboBoxCoordinatesType.setCurrentIndex(1)
            self.gui.lineEditLeft.setEnabled(False)
            self.gui.lineEditBottom.setEnabled(False)

    def set_gui_auto_calibrate(self, auto_calibrate):
        if auto_calibrate:
            self.gui.checkBoxAutocalibrate.setChecked(1)
        else:
            self.gui.checkBoxAutocalibrate.setChecked(0)

    def set_gui_TDOA_grid_based_resolution(self, resolution):
        self.gui.spinGridResolution.setValue(resolution)

    def set_gui_TDOA_grid_based_num_samples(self, num_samples):
        self.gui.spinGridNumCompSamps.setValue(num_samples)

    def set_gui_TDOA_grid_based_measurement_type(self, measurement_type):
        return
        #self.gui.comboBoxGridMeasMatrixType

    def set_gui_TDOA_grid_based_channel_model(self, channel_model):
        return
        #self.gui.comboBoxGridMeasMatrixType

    def set_gui_record_results(self, record_results):
        if record_results:
            self.gui.checkBoxRecordResults.setChecked(1)
        else:
            self.gui.checkBoxRecordResults.setChecked(0)

    def set_gui_record_samples(self, record_samples):
        if record_samples:
            self.gui.checkBoxRecordSamples.setChecked(1)
        else:
            self.gui.checkBoxRecordSamples.setChecked(0)

    def set_gui_reference_selections(self, reference_selections):
        self.reference_selections = reference_selections
        self.gui.comboBoxReferenceSelection.clear()
        for f_type in self.reference_selections:
            self.gui.comboBoxReferenceSelection.addItem(f_type)
        self.gui.comboBoxReferenceSelection.setCurrentIndex(0)

    def set_gui_filtering_types(self, filtering_types):
        self.filtering_types = filtering_types
        self.gui.comboBoxFilteringType.clear()
        for f_type in self.filtering_types:
            self.gui.comboBoxFilteringType.addItem(f_type)
        self.gui.comboBoxFilteringType.setCurrentIndex(0)
        
    def set_gui_motion_models(self, motion_models):
        self.motion_models = motion_models
        self.gui.comboBoxMotionModel.clear()
        for motion_model in self.motion_models:
            self.gui.comboBoxMotionModel.addItem(motion_model)
        self.gui.comboBoxMotionModel.setCurrentIndex(0)

    def set_gui_acquisition_time(self, acquisition_time):
        self.acquisition_time = acquisition_time
        self.gui.acquisitionTimeSpin.setValue(acquisition_time)

    def set_gui_grid_based_active(self, grid_based_active):
        self.grid_based_active = grid_based_active
        if grid_based_active:
            self.gui.gridBasedCheckBox.setChecked(1)
        else:
            self.gui.gridBasedCheckBox.setChecked(0)

    def get_results(self, results):
        self.results = results
        self.new_results = True

    def process_results(self):
        # new reference selected => update plot 
        if "ref_receiver" in self.results:
            if self.ref_receiver != self.results["ref_receiver"]:
                self.pending_receivers_to_plot = True
                self.ref_receiver = self.results["ref_receiver"]
        if hasattr(self, "ax") and self.pending_receivers_to_plot:
            for key in self.receivers:
                receiver = self.receivers[key]
                # save scattered point into receiver properties
                if hasattr(receiver, "scatter"):
                    receiver.scatter.remove()
                    receiver.annotation.remove()
                if hasattr(receiver, "scatter_gps"):
                    receiver.scatter_gps.remove()
                    receiver.annotation_gps.remove()
                receiver.scatter = self.ax.scatter(receiver.coordinates[0], receiver.coordinates[1], marker='x',linewidths=2, c='b', s=200, alpha=0.9, zorder=20)
                receiver.scatter_gps = self.ax.scatter(receiver.coordinates_gps[0], receiver.coordinates_gps[1],linewidths=2, marker='x', c='b', s=200, alpha=0.9, zorder=20)
                # set annotation Rxi
                
                text = ("Rx" + str(self.receivers.keys().index(key) + 1)
                        + " " 
                        + str(np.round(receiver.coordinates,2)))
                        
                if key != self.ref_receiver:
                    receiver.annotation = self.ax.annotate(text, receiver.coordinates,fontweight='bold',bbox=dict(facecolor='w', alpha=0.9, zorder=20))
                else:
                    receiver.annotation = self.ax.annotate(text, receiver.coordinates,fontweight='bold',bbox=dict(facecolor='r', alpha=0.9, zorder=20))
                    
                text_gps = ("Rx" + str(self.receivers.keys().index(key) + 1)
                        + " " 
                        + str(np.round(receiver.coordinates_gps,2)))
                        
                if key != self.ref_receiver:
                    receiver.annotation_gps = self.ax.annotate(text_gps, receiver.coordinates_gps,fontweight='bold',bbox=dict(facecolor='#33ff33', alpha=0.9, zorder=20))
                else:
                    receiver.annotation_gps = self.ax.annotate(text_gps, receiver.coordinates_gps,fontweight='bold',bbox=dict(facecolor='b', alpha=0.9, zorder=20))
                    
            self.canvas.draw()
            self.pending_receivers_to_plot = False

        if self.chat_pending:
            self.textEdit.setText(self.chats)
            sb = self.textEdit.verticalScrollBar()
            sb.setValue(sb.maximum())
            self.chat_pending = False

        if self.set_delegate:
            for row in range(0, self.tmr.rowCount()):
                self.gui.tableViewReceivers.openPersistentEditor(self.tmr.index(row, 1))
                self.gui.tableViewReceivers.openPersistentEditor(self.tmr.index(row, 2))
                self.gui.tableViewReceivers.openPersistentEditor(self.tmr.index(row, 3))
                self.gui.tableViewReceiversPosition.openPersistentEditor(self.tmrp.index(row, 1))
                self.gui.tableViewReceiversPosition.openPersistentEditor(self.tmrp.index(row, 2))
            self.set_delegate = False

        if self.new_results:
            if self.results["correlation"] != None:
                print "Delay:",self.results["delay"]
                if self.gui.checkBoxCorrelation.isChecked():
                    self.gui.qwtPlotCorrelation.setAxisScale(Qwt.QwtPlot.xBottom, -20*self.sample_interpolation,20*self.sample_interpolation)
                else:
                    self.gui.qwtPlotCorrelation.setAxisScale(Qwt.QwtPlot.xBottom, -self.samples_to_receive * self.sample_interpolation, self.samples_to_receive * self.sample_interpolation)
                self.gui.qwtPlotCorrelation.setAxisTitle(Qwt.QwtPlot.xBottom, "Delay: " + str([round(delay,4) for delay in self.results["delay"]]) + " samples")
                # clear the previous points from the plot
                self.gui.qwtPlotCorrelation.clear()
                self.plot_correlation_delay(self.gui.qwtPlotCorrelation, self.results["correlation"][0], self.results["delay"][0],Qt.Qt.blue, self.results["correlation_labels"][0])
                if len(self.results["correlation"]) > 1:
                    self.plot_correlation_delay(self.gui.qwtPlotCorrelation, self.results["correlation"][1], self.results["delay"][1],Qt.Qt.red, self.results["correlation_labels"][1])
                if len(self.results["correlation"]) > 2:
                    self.plot_correlation_delay(self.gui.qwtPlotCorrelation, self.results["correlation"][2], self.results["delay"][2],Qt.Qt.green, self.results["correlation_labels"][2])
                self.gui.qwtPlotCorrelation.replot()
                # clear the previous points from the plot
                self.gui.qwtPlotDelayHistory.clear()
                if len(self.results["delay_history"]) > 0:
                    self.plot_delay_history(self.gui.qwtPlotDelayHistory, self.results["delay_history"][0],Qt.Qt.blue)
                    print "delay hist "+str(self.results["delay_history"][0][-1])
                    delay_history_max = self.results["delay_history"][0][-1]
                    delay_history_min = self.results["delay_history"][0][-1]
                    if len(self.results["delay_history"]) > 1:
                        self.plot_delay_history(self.gui.qwtPlotDelayHistory, self.results["delay_history"][1],Qt.Qt.red)
                        delay_history_max = max( delay_history_max, np.max(self.results["delay_history"][1][-1]) )
                        delay_history_min = min( delay_history_min, np.min(self.results["delay_history"][1][-1]) )
                        if len(self.results["delay_history"]) > 2:
                            self.plot_delay_history(self.gui.qwtPlotDelayHistory, self.results["delay_history"][2],Qt.Qt.green)
                            delay_history_max = max( delay_history_max, np.max(self.results["delay_history"][2][-1]) )
                            delay_history_min = min( delay_history_min, np.min(self.results["delay_history"][2][-1]) )
                    self.gui.qwtPlotDelayHistory.setAxisScale(Qwt.QwtPlot.yLeft, delay_history_min-5, delay_history_max+5)                        
                    self.gui.qwtPlotDelayHistory.replot()

            if len(self.results["receivers"]) > 0:
                self.plot_receiver(self.gui.qwtPlotReceiver1, self.gui.checkBoxFFT1, self.results["receivers"][self.gui.comboBoxReceiver1.currentIndex()])
            if len(self.results["receivers"]) > 1:
                self.plot_receiver(self.gui.qwtPlotReceiver2, self.gui.checkBoxFFT2, self.results["receivers"][self.gui.comboBoxReceiver2.currentIndex()])
            if len(self.results["receivers"]) > 2:
                self.plot_receiver(self.gui.qwtPlotReceiver3, self.gui.checkBoxFFT3, self.results["receivers"][self.gui.comboBoxReceiver3.currentIndex()])
            if self.results["estimated_positions"] != None:
                self.set_tx_position(self.results["estimated_positions"])
        self.new_results = False

    def register_receiver(self, serial, gain, antenna, gain_calibration):
        if not self.receivers.has_key(serial):
            self.receivers[serial] = gui_helpers.receiver_item(gain, antenna, gain_calibration)
            self.tmr.rowsInserted.emit(QtCore.QModelIndex(),0,0)
            self.tmrp.rowsInserted.emit(QtCore.QModelIndex(),0,0)
            self.set_delegate = True
            # populate Reference receiver combo box
            self.gui.comboBoxRefReceiver.clear()
            for serial in self.receivers.keys():
                self.gui.comboBoxRefReceiver.addItem(serial)
            self.gui.comboBoxRefReceiver.setCurrentIndex(0)
            # populate Combo Boxes for sample plots
            self.gui.comboBoxReceiver1.clear()
            self.gui.comboBoxReceiver2.clear()
            self.gui.comboBoxReceiver3.clear()
            for serial in self.receivers.keys():
                self.gui.comboBoxReceiver1.addItem(serial)
                self.gui.comboBoxReceiver2.addItem(serial)
                self.gui.comboBoxReceiver3.addItem(serial)
            self.gui.comboBoxReceiver1.setCurrentIndex(0)
            if len(self.receivers.keys())>1:
                self.gui.comboBoxReceiver2.setCurrentIndex(1)
            if len(self.receivers.keys())>2:
                self.gui.comboBoxReceiver3.setCurrentIndex(2)

    def register_another_gui(self, serial):
        self.tmg.registerGui(serial)

    def localize(self):
        self.rpc_manager.request("localize", [self.frequency, self.lo_offset, self.samples_to_receive])

    def localize_loop(self):
        if hasattr(self, "track_plot"):
            # remove plot item; procedure differs from scatter item!
            self.remove_trackplots()
        self.rpc_manager.request("localize_loop", [self.frequency, self.lo_offset, self.samples_to_receive])

    def start_correlation(self):
        self.rpc_manager.request("start_correlation", [self.frequency, self.lo_offset, self.samples_to_receive])


    def start_correlation_loop(self):
        if hasattr(self, "track_plot"):
        # revove plot item; procedure differs from scatter item!
            self.remove_trackplots()
        self.rpc_manager.request("start_correlation_loop", [self.frequency, self.lo_offset, self.samples_to_receive])

    def stop_loop(self):
        self.rpc_manager.request("stop_loop")
        

    def reset_receivers(self):
        self.rpc_manager.request("reset_receivers")

    def update_receivers(self):
        self.gui.tableViewReceivers.hide()
        self.gui.tableViewReceivers.show()
        def runit():
            self.rpc_manager.request("update_receivers")
        threading.Thread(target=runit).start()

    # plot cross correlation
    def plot_correlation_delay(self, plot, values, delay, color, label):
        correlation_length = (len(values) + 1)/2
        x = range(-correlation_length+1,correlation_length,1)
        y = values
        # draw curve with new points and plot
        curve = Qwt.QwtPlotCurve(label)
        curve.setPen(Qt.QPen(color, 2))
        curve.attach(plot)
        curve.setData(x, y)
        marker_max = Qwt.QwtPlotMarker()
        marker_max.setLineStyle(Qwt.QwtPlotMarker.VLine)
        marker_max.setLinePen(Qt.QPen(color, 1))
        marker_max.attach(plot)
        marker_max.setXValue(delay)
        
    def plot_delay_history(self, plot, samples, color):
        if len(samples) > 0:
            x = range(0,len(samples),1)
            y = samples
            # draw curve with new points and plot
            curve = Qwt.QwtPlotCurve()
            curve.setPen(Qt.QPen(color, 2))
            curve.attach(plot)
            curve.setData(x, y)

    def plot_receiver(self, qwtPlot, checkBoxFFT, samples):
        if checkBoxFFT.isChecked():
            y = 10*np.log10(np.absolute(np.fft.fftshift(np.fft.fft(samples))))
            x = np.linspace((self.frequency-self.samp_rate*self.sample_interpolation/2)/1000000000,(self.frequency+self.samp_rate*self.sample_interpolation/2)/1000000000,len(y))
            title = Qwt.QwtText("Frequency [GHz]")
            title.setFont(Qt.QFont("Helvetica", 10, Qt.QFont.Bold))
            qwtPlot.setAxisTitle(Qwt.QwtPlot.xBottom, title)
            qwtPlot.setAxisScale(Qwt.QwtPlot.xBottom, (self.frequency-self.samp_rate/2)/1000000000, (self.frequency+self.samp_rate/2)/1000000000)
        else:
            y = np.real(samples)
            x = range(0,len(y),1)
            title = Qwt.QwtText("Samples")
            title.setFont(Qt.QFont("Helvetica", 10, Qt.QFont.Bold))
            qwtPlot.setAxisTitle(Qwt.QwtPlot.xBottom, title)
            qwtPlot.setAxisScale(Qwt.QwtPlot.xBottom, 0, len(x))

        # clear the previous points from the plot
        qwtPlot.clear()
        # draw curve wicth new points and plot
        curve = Qwt.QwtPlotCurve()
        curve.setPen(Qt.QPen(Qt.Qt.blue, 2))
        curve.attach(qwtPlot)
        curve.setData(x, y)
        qwtPlot.replot()

    def refresh_correlation(self):
        if self.gui.checkBoxCorrelation.isChecked():
            self.gui.qwtPlotCorrelation.setAxisScale(Qwt.QwtPlot.xBottom, -20*self.sample_interpolation,20*self.sample_interpolation)
        else:
            self.gui.qwtPlotCorrelation.setAxisScale(Qwt.QwtPlot.xBottom, -self.samples_to_receive * self.sample_interpolation, self.samples_to_receive * self.sample_interpolation)
        self.gui.qwtPlotCorrelation.replot()

    def refresh_plot(self, receiver):
        if receiver == self.gui.comboBoxReceiver1.currentIndex()+1 and self.results.has_key("receivers") and len(self.results["receivers"]) > 0:
            self.plot_receiver(self.gui.qwtPlotReceiver1, self.gui.checkBoxFFT1, self.results["receivers"][self.gui.comboBoxReceiver1.currentIndex()])
        if receiver == self.gui.comboBoxReceiver2.currentIndex()+1 and self.results.has_key("receivers") and len(self.results["receivers"]) > 1:
            self.plot_receiver(self.gui.qwtPlotReceiver2, self.gui.checkBoxFFT2, self.results["receivers"][self.gui.comboBoxReceiver2.currentIndex()])
        if receiver == self.gui.comboBoxReceiver3.currentIndex()+1 and self.results.has_key("receivers") and len(self.results["receivers"]) > 2:
            self.plot_receiver(self.gui.qwtPlotReceiver3, self.gui.checkBoxFFT3, self.results["receivers"][self.gui.comboBoxReceiver3.currentIndex()])

class transmitter_position():
    def __init__(self, coordinates):
        self.coordinates = coordinates

###############################################################################
# Options Parser
###############################################################################
def parse_options():
    """ Options parser. """
    parser = OptionParser(option_class=eng_option, usage="%prog: [options]")
    parser.add_option("", "--fusion-center", type="string", default="localhost",
                      help="Fusion center address")
    parser.add_option("-i", "--id-gui", type="int", default="1",
                      help="GUI ID")
    parser.add_option("", "--ssh-proxy", action="store_true", default=False,
                      help="Activate when using a ssh proxy")
    (options, args) = parser.parse_args()
    return options

###############################################################################
# Main
###############################################################################
if __name__ == "__main__":
    options = parse_options()
    qapp = Qt.QApplication(sys.argv)
    qapp.main_window = gui("GNU Radio localization GUI",options)
    qapp.main_window.show()
    qapp.exec_()

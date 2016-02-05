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
from functools import partial
sys.path.append("../python")
import receiver_interface
import rpc_manager as rpc_manager_local
import gui_helpers

class gui(QtGui.QMainWindow):
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

        self.samples_to_receive = 1000
        self.frequency = 2.4e9
        self.samp_rate = 10e6
        self.bw = 1e6
        self.interpolation = 10
        self.lo_offset = 0

        self.results = {}
        self.new_results = False

        self.receivers = {}
        self.ref_receiver = ""
        self.transmitter_positions = {}

        self.chats = ""

        # ZeroMQ
        self.rpc_manager = rpc_manager_local.rpc_manager()
        self.rpc_manager.set_reply_socket(rpc_adr)
        self.rpc_manager.set_request_socket(fusion_center_adr)
        self.rpc_manager.add_interface("new_chat",self.new_chat)
        self.rpc_manager.add_interface("sync_position",self.sync_position)
        self.rpc_manager.add_interface("register_receiver",self.register_receiver)
        self.rpc_manager.add_interface("register_another_gui",self.register_another_gui)
        self.rpc_manager.add_interface("get_results",self.get_results)
        self.rpc_manager.add_interface("set_gui_frequency",self.set_gui_frequency)
        self.rpc_manager.add_interface("set_gui_samp_rate",self.set_gui_samp_rate)
        self.rpc_manager.add_interface("set_gui_bw",self.set_gui_bw)
        self.rpc_manager.add_interface("set_gui_interpolation",self.set_gui_interpolation)
        self.rpc_manager.add_interface("set_gui_lo_offset",self.set_gui_lo_offset)
        self.rpc_manager.add_interface("set_gui_samples_to_receive",self.set_gui_samples_to_receive)
        self.rpc_manager.add_interface("set_gui_gain",self.set_gui_gain)
        self.rpc_manager.add_interface("set_gui_antenna",self.set_gui_antenna)
        self.rpc_manager.add_interface("set_gui_selected_position",self.set_gui_selected_position)
        self.rpc_manager.add_interface("set_gps_position",self.set_gps_position)
        self.rpc_manager.add_interface("set_gui_TDOA_grid_based_resolution",self.set_gui_TDOA_grid_based_resolution)
        self.rpc_manager.add_interface("set_gui_TDOA_grid_based_num_samples",self.set_gui_TDOA_grid_based_num_samples)
        self.rpc_manager.add_interface("set_gui_TDOA_grid_based_channel_model",self.set_gui_TDOA_grid_based_channel_model)
        self.rpc_manager.add_interface("set_gui_TDOA_grid_based_measurement_type",self.set_gui_TDOA_grid_based_measurement_type)
        self.rpc_manager.add_interface("set_gui_ref_receiver",self.set_gui_ref_receiver)
        self.rpc_manager.start_watcher()

        # Find out ip address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if not options.fusion_center == "localhost":
            s.connect((options.fusion_center,22))
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
        self.canvas.mpl_connect("button_release_event", self.calibrate)

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
        self.gui.qwtPlotCorrelation.setAxisScale(Qwt.QwtPlot.xBottom, -self.samples_to_receive * self.interpolation, self.samples_to_receive * self.interpolation)
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

        #Signals
        self.connect(self.update_timer, QtCore.SIGNAL("timeout()"), self.process_results)
        self.connect(self.gui.pushButtonChat, QtCore.SIGNAL("clicked()"), self.send_chat)
        self.connect(self.gui.lineEdit, QtCore.SIGNAL("returnPressed()"), self.send_chat)
        self.connect(self.gui.pushButtonRunReceivers, QtCore.SIGNAL("clicked()"), self.start_correlation)
        self.connect(self.gui.pushButtonRunReceiversLoop, QtCore.SIGNAL("clicked()"), self.start_correlation_loop)
        self.connect(self.gui.pushButtonStopReceiversLoop, QtCore.SIGNAL("clicked()"), self.stop_loop)
        self.connect(self.gui.pushButtonSetCalibration, QtCore.SIGNAL("clicked()"), self.set_calibration)
        self.connect(self.gui.pushButtonRemoveCalibration, QtCore.SIGNAL("clicked()"), self.remove_calibration)
        self.connect(self.gui.pushButtonUpdate, QtCore.SIGNAL("clicked()"), self.update_receivers)
        self.connect(self.gui.pushButtonLocalize, QtCore.SIGNAL("clicked()"), self.localize)
        self.connect(self.gui.pushButtonLocalizeContinuous, QtCore.SIGNAL("clicked()"), self.localize_loop)
        self.connect(self.gui.pushButtonLocalizeStop, QtCore.SIGNAL("clicked()"), self.stop_loop)
        self.connect(self.gui.checkBoxFFT1, QtCore.SIGNAL("clicked()"), partial(self.refresh_plot,1))
        self.connect(self.gui.checkBoxFFT2, QtCore.SIGNAL("clicked()"), partial(self.refresh_plot,2))
        self.connect(self.gui.checkBoxFFT3, QtCore.SIGNAL("clicked()"), partial(self.refresh_plot,3))
        self.connect(self.gui.checkBoxCorrelation, QtCore.SIGNAL("clicked()"), self.refresh_correlation)
        self.connect(self.gui.frequencySpin, QtCore.SIGNAL("valueChanged(double)"), self.set_frequency)
        self.connect(self.gui.sampRateSpin, QtCore.SIGNAL("valueChanged(double)"), self.set_samp_rate)
        self.connect(self.gui.interpolationSpin, QtCore.SIGNAL("valueChanged(int)"), self.set_interpolation)
        self.connect(self.gui.loOffsetSpin, QtCore.SIGNAL("valueChanged(double)"), self.set_lo_offset)
        self.connect(self.gui.samplesToReceiveSpin, QtCore.SIGNAL("valueChanged(int)"), self.set_samples_to_receive)
        self.connect(self.gui.comboBoxRefReceiver, QtCore.SIGNAL("currentIndexChanged(int)"), self.set_ref_receiver)
        self.shortcut_start = QtGui.QShortcut(Qt.QKeySequence("Ctrl+S"), self.gui)
        self.shortcut_stop = QtGui.QShortcut(Qt.QKeySequence("Ctrl+C"), self.gui)
        self.shortcut_exit = QtGui.QShortcut(Qt.QKeySequence("Ctrl+D"), self.gui)
        self.connect(self.shortcut_exit, QtCore.SIGNAL("activated()"), self.gui.close)

        # Grid based signals
        self.connect(self.gui.spinGridResolution, QtCore.SIGNAL("valueChanged(int)"), self.set_TDOA_grid_based_resolution)
        self.connect(self.gui.spinGridNumCompSamps, QtCore.SIGNAL("valueChanged(int)"), self.set_TDOA_grid_based_num_samples)

        # start update timer
        self.update_timer.start(33)
        self.timer_register = threading.Thread(target = self.register_gui)
        self.timer_register.daemon = True
        self.timer_register.start()

    def set_calibration(self):
        if hasattr(self, "zp"):
            self.setting_calibration = True
            self.zp.enabled = False

    def remove_calibration(self):
        self.rpc_manager.request("remove_calibration")

    def init_map(self):
        bbox = self.bbox

        inProj = Proj(init='epsg:4326')
        outProj = Proj(init='epsg:3857')
        x0, y0 = transform(inProj,outProj,bbox[0],bbox[1])
        x1, y1 = transform(inProj,outProj,bbox[2],bbox[3])
        x = x1-x0
        y = y1-y0
        scale = math.ceil(math.sqrt(abs(x*y/0.3136))) * 2

        r = requests.get("http://render.openstreetmap.org/cgi-bin/export?bbox=" + str(bbox)[1:-1] + "&scale=" + str(scale) + "&format=png", stream=True)

        if r.status_code == 200:
            img = Image.open(StringIO(r.content))
            if not os.path.exists("../maps"):
                    os.makedirs("../maps")
            img.save("../maps/map.png")

        #img = Image.open("../maps/ict_cubes.png")

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

        self.basemap = Basemap(llcrnrlon=bbox[0], llcrnrlat=bbox[1],
                      urcrnrlon=bbox[2], urcrnrlat=bbox[3],
                      projection='tmerc', ax=self.ax, lon_0=lon_0, lat_0=lat_0)

        self.basemap.imshow(img, interpolation='lanczos', origin='upper')

        self.zp = gui_helpers.ZoomPan()
        figZoom = self.zp.zoom_factory(self.ax, base_scale = 1.5)
        figPan = self.zp.pan_factory(self.ax)

        self.figure.tight_layout(pad=0)
        #self.figure.patch.set_visible(False)
        self.ax.axis('off')
        self.canvas.draw()
        self.hyperbolas = {}

    def init_plot(self, qwtPlot):
        title = Qwt.QwtText("Samples")
        title.setFont(Qt.QFont("Helvetica", 10, Qt.QFont.Bold))
        qwtPlot.setAxisTitle(Qwt.QwtPlot.xBottom, title)
        title = Qwt.QwtText("Amplitude")
        title.setFont(Qt.QFont("Helvetica", 10, Qt.QFont.Bold))
        qwtPlot.setAxisTitle(Qwt.QwtPlot.yLeft, title)
        qwtPlot.setAxisScale(Qwt.QwtPlot.xBottom, 0, self.samples_to_receive * self.interpolation)
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
            bbox = self.rpc_manager.request("register_gui",[self.ip_addr, self.hostname, options.id_gui, first])
            if first and bbox != None:
                self.bbox = bbox
                threading.Thread(target = self.init_map).start()
                first = False
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
            # set annotation RXx
            text = "RX" + str(self.receivers.keys().index(serial) + 1)
            receiver.annotation = self.ax.annotate(text, coordinates,fontweight='bold',bbox=dict(facecolor='w', alpha=0.9))
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
            # set annotation RXx
            text = "RX" + str(self.receivers.keys().index(serial) + 1)
            receiver.annotation_gps = self.ax.annotate(text, receiver.coordinates_gps,fontweight='bold',bbox=dict(facecolor='#33ff33', alpha=0.9))
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
        for algorithm in transmitter_positions.items():
            if not self.transmitter_positions.has_key(algorithm[0]):
                self.transmitter_positions[algorithm[0]] = transmitter_position(algorithm[1]["coordinates"])
            else:
                self.transmitter_positions[algorithm[0]].coordinates = algorithm[1]["coordinates"]
            estimated_position = self.transmitter_positions[algorithm[0]]
            if hasattr(estimated_position, "scatter"):
                estimated_position.scatter.remove()
                estimated_position.annotation.remove()
            if hasattr(self, "ax"):
                # save scattered point into receiver properties
                estimated_position.scatter = self.ax.scatter(estimated_position.coordinates[0], estimated_position.coordinates[1],linewidths=2,  marker='x', c='red', s=200, alpha=0.9, zorder=20)
                # set annotation RXx
                text = algorithm[0]
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

    def plot_hyperbolas(self, pos_tx=None, c="red"):
        pos_rx = []
        hyperbolas = []
        for receiver in self.receivers:
            if self.receivers[receiver].selected_position == "manual":
                if receiver == self.ref_receiver:
                    pos_rx.insert(0,self.receivers[receiver].coordinates)
                else:
                    pos_rx.append(self.receivers[receiver].coordinates)
            else:
                if receiver == self.ref_receiver:
                    pos_rx.insert(0,self.receivers[receiver].coordinates_gps)
                else:
                    pos_rx.append(self.receivers[receiver].coordinates_gps)
        for i in range(1,len(pos_rx)):
            if pos_tx is None:
                hyperbola = self.get_hyperbola([pos_rx[0],pos_rx[i]], pos_tx, self.results["delay"][i-1])
            else:
                hyperbola = self.get_hyperbola([pos_rx[0],pos_rx[i]], pos_tx)

            if len(hyperbolas) < i:
                h = self.ax.scatter(hyperbola[0],hyperbola[1],c=c,zorder=10,edgecolors='none')
                hyperbolas.append(h)
            else:
                hyperbolas[i-1] = self.ax.scatter(hyperbola[0],hyperbola[1],c=c,zorder=10,edgecolors='none')
        return hyperbolas


    def get_hyperbola(self, pos_rx, pos_tx=None, delay=None):
        # Redefine receivers position and signals so that the signal arrives first
        # to the nearest receiver.
        pos_rx = np.array(pos_rx)
        if pos_tx is not None:
            pos_tx = np.array(pos_tx)
            d1 = np.linalg.norm(pos_rx[0]-pos_tx)
            d2 = np.linalg.norm(pos_rx[1]-pos_tx)

        if delay is not None:
            if delay<0:
                pos_rx = np.flipud(pos_rx)
        else:
            if d1>d2:
                pos_rx = np.flipud(pos_rx)

        # Baseline distance between sensors
        B = np.linalg.norm(pos_rx[1]-pos_rx[0])

        # Asign alpha0 angle depending on the sensors relative position
        if pos_rx[1][0]>pos_rx[0][0]:
            alpha0 = -np.arcsin((pos_rx[1][1]-pos_rx[0][1])/B)
        else:
            alpha0 = np.arcsin((pos_rx[1][1]-pos_rx[0][1])/B)+np.pi

        # Calculate parametric points of the hyperbola
        Hx = []
        Hy = []
        if delay is not None:
            delta_rx1_rx2 = delay / (self.samp_rate * self.interpolation) * 299700000
        else:
            delta_rx1_rx2 = np.linalg.norm(pos_tx-pos_rx[0])-np.linalg.norm(pos_tx-pos_rx[1])
        [max_x,max_y]=np.round(self.basemap(self.bbox[2],self.bbox[3]))
        for alpha in np.arange(0,2*np.pi,0.005):
            h = np.array(pos_rx[0])-(B*B-delta_rx1_rx2*delta_rx1_rx2)/(2*(-delta_rx1_rx2-B*np.cos(alpha)))*np.array([np.cos(alpha-alpha0),np.sin(alpha-alpha0)])
            # Get only points in the region of interest
            distance_1 = np.linalg.norm(h-pos_rx[0])
            distance_2 = np.linalg.norm(h-pos_rx[1])
            if ((distance_1 <= distance_2) and (0 < h[0] < max_x) and (0 < h[1] < max_y)):
                Hx.append(h[0])
                Hy.append(h[1])
        return [Hx,Hy]

    def plot_grid(self, s):
        if hasattr(self,"grid"):
            self.grid.remove()
        self.grid = self.ax.pcolor(np.array(s[0]),np.array(s[1]),np.array(s[2]), cmap='coolwarm', alpha=0.7)

    def calibrate(self,mouse_event):
        if self.setting_calibration:
            self.rpc_manager.request("calibrate",[(mouse_event.xdata,mouse_event.ydata)])
            self.setting_calibration = False
            self.zp.enabled = True

    def set_position(self, mouse_event):
        if self.setting_pos_receiver is not "":
            receiver = self.receivers[self.setting_pos_receiver]
            self.rpc_manager.request("sync_position",[self.setting_pos_receiver, (mouse_event.xdata,mouse_event.ydata)])
            self.rpc_manager.request("get_gui_gps_position",[self.setting_pos_receiver])
            self.setting_pos_receiver = ""
            self.zp.enabled = True

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
        self.interpolation = self.gui.interpolationSpin.value()
        self.rpc_manager.request("set_interpolation",[self.interpolation])

    def set_gain(self, gain, serial):
        self.rpc_manager.request("set_gain",[gain, serial])

    def set_antenna(self, antenna, serial):
        self.rpc_manager.request("set_antenna",[antenna, serial])

    def set_selected_position(self, selected_position, serial):
        self.rpc_manager.request("set_selected_position",[selected_position, serial])

    def set_ref_receiver(self):
        self.ref_receiver = self.gui.comboBoxRefReceiver.currentText()
        self.rpc_manager.request("set_ref_receiver",[str(self.ref_receiver)])

    def set_TDOA_grid_based_resolution(self, resolution):
        self.rpc_manager.request("set_TDOA_grid_based_resolution", [resolution])

    def set_TDOA_grid_based_num_samples(self, num_samples):
        self.rpc_manager.request("set_TDOA_grid_based_num_samples", [num_samples])

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

    def set_gui_gain(self, gain, serial):
        self.tmr.set_gain(gain, serial)

    def set_gui_antenna(self, antenna, serial):
        self.tmr.set_antenna(antenna, serial)

    def set_gui_selected_position(self, selected_position, serial):
        self.tmrp.set_selected_position(selected_position, serial)

    def set_gui_ref_receiver(self, ref_receiver):
        return

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

    def get_results(self, results):
        self.results = results
        self.new_results = True

    def process_results(self):
        if hasattr(self, "ax") and self.pending_receivers_to_plot:
            for key in self.receivers:
                receiver = self.receivers[key]
                # save scattered point into receiver properties
                receiver.scatter = self.ax.scatter(receiver.coordinates[0], receiver.coordinates[1], marker='x',linewidths=2, c='b', s=200, alpha=0.9, zorder=20)
                receiver.scatter_gps = self.ax.scatter(receiver.coordinates_gps[0], receiver.coordinates_gps[1],linewidths=2, marker='x', c='b', s=200, alpha=0.9, zorder=20)
                # set annotation RXx
                text = "RX" + str(self.receivers.keys().index(key) + 1)
                receiver.annotation = self.ax.annotate(text, receiver.coordinates,fontweight='bold',bbox=dict(facecolor='w', alpha=0.9, zorder=20))
                receiver.annotation_gps = self.ax.annotate(text, receiver.coordinates_gps,fontweight='bold',bbox=dict(facecolor='#33ff33', alpha=0.9, zorder=20))

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
                self.gui.tableViewReceiversPosition.openPersistentEditor(self.tmrp.index(row, 1))
                self.gui.tableViewReceiversPosition.openPersistentEditor(self.tmrp.index(row, 2))
            self.set_delegate = False

        if self.new_results:
            if self.results["correlation"] != None:
                print "Delay:",self.results["delay"]
                if self.gui.checkBoxCorrelation.isChecked():
                    self.gui.qwtPlotCorrelation.setAxisScale(Qwt.QwtPlot.xBottom, -100,100)
                else:
                    self.gui.qwtPlotCorrelation.setAxisScale(Qwt.QwtPlot.xBottom, -self.samples_to_receive * self.interpolation, self.samples_to_receive * self.interpolation)
                self.gui.qwtPlotCorrelation.setAxisTitle(Qwt.QwtPlot.xBottom, "Delay: " + str(self.results["delay"]) + " samples")
                # clear the previous points from the plot
                self.gui.qwtPlotCorrelation.clear()
                self.plot_correlation(self.gui.qwtPlotCorrelation, self.results["correlation"][0],Qt.Qt.blue)
                if len(self.results["correlation"]) > 1:
                    self.plot_correlation(self.gui.qwtPlotCorrelation, self.results["correlation"][1],Qt.Qt.red)
                # clear the previous points from the plot
                self.gui.qwtPlotDelayHistory.clear()
                if len(self.results["delay_history"]) > 0:
                    self.plot_delay_history(self.gui.qwtPlotDelayHistory, self.results["delay_history"][0],Qt.Qt.blue)
                    if len(self.results["delay_history"]) > 1:
                        self.plot_delay_history(self.gui.qwtPlotDelayHistory, self.results["delay_history"][1],Qt.Qt.red)
            if len(self.results["receivers"]) > 0:
                self.plot_receiver(self.gui.qwtPlotReceiver1, self.gui.checkBoxFFT1, self.results["receivers"][0])
            if len(self.results["receivers"]) > 1:
                self.plot_receiver(self.gui.qwtPlotReceiver2, self.gui.checkBoxFFT2, self.results["receivers"][1])
            if len(self.results["receivers"]) > 2:
                self.plot_receiver(self.gui.qwtPlotReceiver3, self.gui.checkBoxFFT3, self.results["receivers"][2])
            if self.results["estimated_positions"] != None:
                self.set_tx_position(self.results["estimated_positions"])
        self.new_results = False

    def register_receiver(self, serial, gain, antenna):
        if not self.receivers.has_key(serial):
            self.receivers[serial] = gui_helpers.receiver_item(gain, antenna)
            self.tmr.rowsInserted.emit(QtCore.QModelIndex(),0,0)
            self.tmrp.rowsInserted.emit(QtCore.QModelIndex(),0,0)
            self.set_delegate = True
            # populate Reference receiver combo box
            self.gui.comboBoxRefReceiver.addItem(serial)
            self.gui.comboBoxRefReceiver.setCurrentIndex(0)

    def register_another_gui(self, serial):
        self.tmg.registerGui(serial)

    def localize(self):
        self.rpc_manager.request("localize", [self.frequency, self.lo_offset, self.samples_to_receive])

    def localize_loop(self):
        self.rpc_manager.request("localize_loop", [self.frequency, self.lo_offset, self.samples_to_receive])

    def start_correlation(self):
        self.rpc_manager.request("start_correlation", [self.frequency, self.lo_offset, self.samples_to_receive])

    def start_correlation_loop(self):
        self.rpc_manager.request("start_correlation_loop", [self.frequency, self.lo_offset, self.samples_to_receive])

    def stop_loop(self):
        self.rpc_manager.request("stop_loop")

    def send_correlation_command(self):
        self.rpc_manager.request("start_correlation", [receiver1, receiver2, self.frequency, self.lo_offset, self.samples_to_receive])
        while self.run_loop:
            self.rpc_manager.request("start_correlation", [receiver1, receiver2, self.frequency, self.lo_offset, self.samples_to_receive])
            time.sleep(1.5)

    def reset_receivers(self):
        self.rpc_manager.request("reset_receivers")

    def update_receivers(self):
        self.gui.tableViewReceivers.hide()
        self.gui.tableViewReceivers.show()
        def runit():
            self.rpc_manager.request("update_receivers")
        threading.Thread(target=runit).start()

    # plot cross correlation
    def plot_correlation(self, plot, samples, colour):
        num_corr_samples = (len(samples) + 1)/2
        x = range(-num_corr_samples+1,num_corr_samples,1)
        y = samples
        # draw curve with new points and plot
        curve = Qwt.QwtPlotCurve()
        curve.setPen(Qt.QPen(colour, 2))
        curve.attach(plot)
        curve.setData(x, y)
        plot.replot()

    def plot_delay_history(self, plot, samples, colour):
        if len(samples) > 0:
            y_max = np.max(np.abs(samples))
            self.gui.qwtPlotDelayHistory.setAxisScale(Qwt.QwtPlot.yLeft, -y_max-5, y_max+5)
            num_corr_samples = (len(samples) + 1)/2
            x = range(0,len(samples),1)
            y = samples
            # draw curve with new points and plot
            curve = Qwt.QwtPlotCurve()
            curve.setPen(Qt.QPen(colour, 2))
            curve.attach(plot)
            curve.setData(x, y)
            plot.replot()

    def plot_receiver(self, qwtPlot, checkBoxFFT, samples):
        if checkBoxFFT.isChecked():
            y = 10*np.log10(np.absolute(np.fft.fftshift(np.fft.fft(samples))))
            x = np.linspace((self.frequency-self.samp_rate/2)/1000000000,(self.frequency+self.samp_rate/2)/1000000000,len(y))
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
            self.gui.qwtPlotCorrelation.setAxisScale(Qwt.QwtPlot.xBottom, -100,100)
        else:
            self.gui.qwtPlotCorrelation.setAxisScale(Qwt.QwtPlot.xBottom, -self.samples_to_receive * self.interpolation, self.samples_to_receive * self.interpolation)
        self.gui.qwtPlotCorrelation.replot()

    def refresh_plot(self, receiver):
        if receiver == 1 and self.results.has_key("receivers") and len(self.results["receivers"]) > 0:
            self.plot_receiver(self.gui.qwtPlotReceiver1, self.gui.checkBoxFFT1, self.results["receivers"][0])
        if receiver == 2 and self.results.has_key("receivers") and len(self.results["receivers"]) > 1:
            self.plot_receiver(self.gui.qwtPlotReceiver2, self.gui.checkBoxFFT2, self.results["receivers"][1])
        if receiver == 3 and self.results.has_key("receivers") and len(self.results["receivers"]) > 2:
            self.plot_receiver(self.gui.qwtPlotReceiver3, self.gui.checkBoxFFT3, self.results["receivers"][2])

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
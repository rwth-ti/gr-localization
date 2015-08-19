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
import receiver_interface
import time
import threading
import rpc_manager as rpc_manager_local
import gui_helpers
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap
import requests
from StringIO import StringIO
from PIL import Image
from pyproj import Proj, transform
import math

class gui(QtGui.QMainWindow):
    def __init__(self, window_name, options, parent=None):
        QtGui.QMainWindow.__init__(self, parent)

        self.thread = threading.current_thread()

        # give Ctrl+C back to system
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        self.gui = uic.loadUi(os.path.join(os.path.dirname(__file__),'main_window.ui'), self)

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
        self.lo_offset = 0

        self.results = {}

        self.chats = ""

        # ZeroMQ
        fusion_center_adr = "tcp://" + options.fusion_center + ":6665"

        self.rpc_manager = rpc_manager_local.rpc_manager()
        self.rpc_manager.set_reply_socket(rpc_adr)
        self.rpc_manager.set_request_socket(fusion_center_adr)
        self.rpc_manager.add_interface("new_chat",self.new_chat)
        self.rpc_manager.add_interface("register_receiver",self.register_receiver)
        self.rpc_manager.add_interface("register_another_gui",self.register_another_gui)
        self.rpc_manager.add_interface("get_results",self.get_results)
        self.rpc_manager.add_interface("set_gui_frequency",self.set_gui_frequency)
        self.rpc_manager.add_interface("set_gui_samp_rate",self.set_gui_samp_rate)
        self.rpc_manager.add_interface("set_gui_bw",self.set_gui_bw)
        self.rpc_manager.add_interface("set_gui_lo_offset",self.set_gui_lo_offset)
        self.rpc_manager.add_interface("set_gui_samples_to_receive",self.set_gui_samples_to_receive)
        self.rpc_manager.add_interface("set_gui_gain",self.set_gui_gain)
        self.rpc_manager.add_interface("set_gui_antenna",self.set_gui_antenna)
        self.rpc_manager.start_watcher()

        self.gui.setWindowTitle(window_name)

        self.chat_pending = False

        # map configuration
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)

        self.verticalLayoutMap.addWidget(self.toolbar)
        self.verticalLayoutMap.addWidget(self.canvas)
        threading.Thread(target = self.init_map).start()

        # correlation and signals
        self.init_plot(self.gui.qwtPlotReceiver1)
        self.init_plot(self.gui.qwtPlotReceiver2)
        self.gui.qwtPlotCorrelation.setTitle("Cross correlation")
        self.gui.qwtPlotCorrelation.setAxisTitle(Qwt.QwtPlot.xBottom, "Delay")
        self.gui.qwtPlotCorrelation.setAxisTitle(Qwt.QwtPlot.yLeft, "Amplitude")
        self.gui.qwtPlotCorrelation.setAxisScale(Qwt.QwtPlot.xBottom, -self.samples_to_receive, self.samples_to_receive)
        self.gui.qwtPlotCorrelation.setAxisScale(Qwt.QwtPlot.xBottom, -100, 100)

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
        self.connect(self.gui.pushButtonResetReceivers, QtCore.SIGNAL("clicked()"), self.reset_receivers)
        self.connect(self.gui.pushButtonUpdate, QtCore.SIGNAL("clicked()"), self.update_receivers)
        self.connect(self.gui.frequencySpin, QtCore.SIGNAL("valueChanged(double)"), self.set_frequency)
        self.connect(self.gui.sampRateSpin, QtCore.SIGNAL("valueChanged(double)"), self.set_samp_rate)
        self.connect(self.gui.bwSpin, QtCore.SIGNAL("valueChanged(double)"), self.set_bw)
        self.connect(self.gui.loOffsetSpin, QtCore.SIGNAL("valueChanged(double)"), self.set_lo_offset)
        self.connect(self.gui.samplesToReceiveSpin, QtCore.SIGNAL("valueChanged(int)"), self.set_samples_to_receive)
        self.shortcut_start = QtGui.QShortcut(Qt.QKeySequence("Ctrl+S"), self.gui)
        self.shortcut_stop = QtGui.QShortcut(Qt.QKeySequence("Ctrl+C"), self.gui)
        self.shortcut_exit = QtGui.QShortcut(Qt.QKeySequence("Ctrl+D"), self.gui)
        self.connect(self.shortcut_exit, QtCore.SIGNAL("activated()"), self.gui.close)

        # start update timer
        self.update_timer.start(33)
        self.timer_register = threading.Thread(target = self.register_gui)
        self.timer_register.daemon = True
        self.timer_register.start()

    def init_map(self):
        bbox = 6.0600,50.7775,6.0670,50.7810

        inProj = Proj(init='epsg:4326')
        outProj = Proj(init='epsg:3857')
        x0, y0 = transform(inProj,outProj,bbox[0],bbox[1])
        x1, y1 = transform(inProj,outProj,bbox[2],bbox[3])
        x = x1-x0
        y = y1-y0
        scale = math.ceil(math.sqrt(x*y/0.3136)) * 2

        r = requests.get("http://render.openstreetmap.org/cgi-bin/export?bbox=" + str(bbox)[1:-1] + "&scale=" + str(scale) + "&format=png", stream=True)

        if r.status_code == 200:
            img = Image.open(StringIO(r.content))

        ax = self.figure.add_subplot(111)

        #
        # create basemap
        #
        map = Basemap(llcrnrlon=bbox[0], llcrnrlat=bbox[1],
                      urcrnrlon=bbox[2], urcrnrlat=bbox[3],
                      projection='merc', ax=ax)

        map.imshow(img, interpolation='lanczos', origin='upper')
        #
        # plot custom points
        #
        x0, y0 = 6.0631, 50.77925 # TI
        x1, y1 = 6.0653, 50.7790 # UMIC
        x2, y2 = 6.0612, 50.7782 # Informatik
        x, y = map((x0, x1, x2), (y0, y1, y2))
        ax.scatter(x, y, c='red', edgecolor='none', s=50, alpha=0.9)

        x0, y0 = 6.0622, 50.7787 # TX
        #x0, y0 = 6.06274333333, 50.7792233333 # TX
        x, y = map(x0, y0)
        ax.scatter(x, y, c='blue', edgecolor='none', s=100, alpha=0.9)

        self.canvas.draw()


    def init_plot(self, qwtPlot):
        qwtPlot.setTitle("Signal Scope")
        qwtPlot.setAxisTitle(Qwt.QwtPlot.xBottom, "Samples")
        qwtPlot.setAxisTitle(Qwt.QwtPlot.yLeft, "Amplitude")
        qwtPlot.setAxisScale(Qwt.QwtPlot.xBottom, 0, self.samples_to_receive)
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
            self.rpc_manager.request("register_gui",[self.hostname, options.id_gui, first])
            first = False
            print "Parameters:",self.frequency, self.samp_rate, self.bw, self.samples_to_receive, self.lo_offset, self.tmr.receivers
            for receiver in self.tmr.receivers.values():
                print receiver.gain
                print receiver.antenna
            time.sleep(10)

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

    def set_gain(self, gain, serial):
        self.rpc_manager.request("set_gain",[gain, serial])

    def set_antenna(self, antenna, serial):
        self.rpc_manager.request("set_antenna",[antenna, serial])

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

    def set_gui_gain(self, gain, serial):
        self.tmr.set_gain(gain, serial)

    def set_gui_antenna(self, antenna, serial):
        self.tmr.set_antenna(antenna, serial)

    def get_results(self, results):
        self.results = results

    def process_results(self):
        if self.chat_pending:
            self.textEdit.setText(self.chats)
            sb = self.textEdit.verticalScrollBar()
            sb.setValue(sb.maximum())
            self.chat_pending = False

        if self.set_delegate:
            for row in range(0, self.tmr.rowCount()):
                self.gui.tableViewReceivers.openPersistentEditor(self.tmr.index(row, 1))
                self.gui.tableViewReceivers.openPersistentEditor(self.tmr.index(row, 2))
            self.set_delegate = False

        if len(self.results.items()) > 0:
            print "Delay:",self.results["delay"]
            self.plot_correlation(self.gui.qwtPlotCorrelation, self.results["correlation"])
            self.plot_receiver(self.gui.qwtPlotReceiver1, self.results["receiver1"])
            self.plot_receiver(self.gui.qwtPlotReceiver2, self.results["receiver2"])
        self.results = {}

    def register_receiver(self, serial, gain, antenna):
        if self.tmr.registerReceiver(serial, gain, antenna):
            self.set_delegate = True
            # populate cross-correlation combo boxes
            self.gui.comboBoxReceiver1.addItem(serial)
            self.gui.comboBoxReceiver2.addItem(serial)

    def register_another_gui(self, serial):
        self.tmg.registerGui(serial)

    def start_correlation(self):
        receiver1 = str(self.gui.comboBoxReceiver1.currentText())
        receiver2 = str(self.gui.comboBoxReceiver2.currentText())
        self.rpc_manager.request("start_correlation", [receiver1, receiver2, self.frequency, self.lo_offset, self.samples_to_receive])

    def reset_receivers(self):
        self.rpc_manager.request("reset_receivers")

    def update_receivers(self):
        self.gui.tableViewReceivers.hide()
        self.gui.tableViewReceivers.show()
        def runit():
            self.rpc_manager.request("update_receivers")
        threading.Thread(target=runit).start()

    # plot cross correlation
    def plot_correlation(self, plot, samples):
        num_corr_samples = (len(samples) + 1)/2
        x = range(-num_corr_samples+1,num_corr_samples,1)
        y = samples
        # clear the previous points from the plot
        plot.clear()
        # draw curve with new points and plot
        curve = Qwt.QwtPlotCurve()
        curve.setPen(Qt.QPen(Qt.Qt.blue, 2))
        curve.attach(plot)
        curve.setData(x, y)
        plot.replot()

    def plot_receiver(self, qwtPlot, samples):
        x = range(0,len(samples),1)
        y = np.absolute(samples)
        qwtPlot.setAxisScale(Qwt.QwtPlot.xBottom, 0, self.samples_to_receive)
        # clear the previous points from the plot
        qwtPlot.clear()
        # draw curve with new points and plot
        curve = Qwt.QwtPlotCurve()
        curve.setPen(Qt.QPen(Qt.Qt.blue, 2))
        curve.attach(qwtPlot)
        curve.setData(x, y)
        qwtPlot.replot()

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
    qapp.main_window = gui("Remote GNU Radio GUI",options)
    qapp.main_window.show()
    qapp.exec_()

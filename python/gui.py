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

class gui(QtGui.QMainWindow):
    def __init__(self, window_name, options, parent=None):
        QtGui.QMainWindow.__init__(self, parent)

        self.thread = threading.current_thread()

        # give Ctrl+C back to system
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        self.gui = uic.loadUi(os.path.join(os.path.dirname(__file__),'main_window.ui'), self)

        self.update_timer = Qt.QTimer()

        # socket addresses
        rpc_adr = "tcp://*:" + str(7775 + options.id_gui)
        fusion_center_adr = "tcp://" + options.fusion_center + ":6665"

        self.samples_to_receive = 1000
        self.frequency = 2.4e9
        self.samp_rate = 10e6
        self.bw = 1e6
        self.lo_offset = 0

        self.results = {}

        # ZeroMQ
        fusion_center_adr = "tcp://" + options.fusion_center + ":6665"

        self.rpc_manager = rpc_manager_local.rpc_manager()
        self.rpc_manager.set_reply_socket(rpc_adr)
        self.rpc_manager.set_request_socket(fusion_center_adr)
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
            self.rpc_manager.request("register_gui",[os.uname()[1], options.id_gui, first])
            first = False
            print "Parameters:",self.frequency, self.samp_rate, self.bw, self.samples_to_receive, self.lo_offset, self.tmr.receivers
            for receiver in self.tmr.receivers.values():
                print receiver.gain
                print receiver.antenna
            time.sleep(10)

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

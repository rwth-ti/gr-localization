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

class gui(QtGui.QMainWindow):
    def __init__(self, window_name, options, parent=None):
        QtGui.QMainWindow.__init__(self, parent)

        # give Ctrl+C back to system
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        self.gui = uic.loadUi(os.path.join(os.path.dirname(__file__),'main_window.ui'), self)

        self.update_timer = Qt.QTimer()

        # socket addresses
        rpc_adr = "tcp://*:6665"

        self.samples_to_receive = options.num_samples

        self.receivers = {}

        # ZeroMQ
        self.probe_manager = zeromq.probe_manager()
        self.rpc_manager = zeromq.rpc_manager()
        self.rpc_manager.set_reply_socket(rpc_adr)
        self.rpc_manager.add_interface("register_receiver",self.register_receiver)
        self.rpc_manager.start_watcher()

        self.gui.setWindowTitle(window_name)
        self.init_plot(self.gui.qwtPlotUsrp1)
        self.init_plot(self.gui.qwtPlotUsrp2)
        self.gui.qwtPlotCorrelation.setTitle("Cross correlation")
        self.gui.qwtPlotCorrelation.setAxisTitle(Qwt.QwtPlot.xBottom, "Delay")
        self.gui.qwtPlotCorrelation.setAxisTitle(Qwt.QwtPlot.yLeft, "Amplitude")
        self.gui.qwtPlotCorrelation.setAxisScale(Qwt.QwtPlot.xBottom, -self.samples_to_receive, self.samples_to_receive)
        self.gui.qwtPlotCorrelation.setAxisScale(Qwt.QwtPlot.xBottom, -100, 100)

        # Grid
        pen = Qt.QPen(Qt.Qt.DotLine)
        pen.setColor(Qt.Qt.black)
        pen.setWidth(0)
        grid_correlation = Qwt.QwtPlotGrid()
        grid_correlation.setPen(pen)
        grid_correlation.attach(self.gui.qwtPlotCorrelation)

        #Signals
        self.connect(self.update_timer, QtCore.SIGNAL("timeout()"), self.probe_manager.watcher)
        self.connect(self.update_timer, QtCore.SIGNAL("timeout()"), self.process_results)
        self.connect(self.gui.pushButtonRunReceivers, QtCore.SIGNAL("clicked()"), self.start_receivers)
        self.connect(self.gui.pushButtonResetReceivers, QtCore.SIGNAL("clicked()"), self.reset_receivers)
        self.shortcut_start = QtGui.QShortcut(Qt.QKeySequence("Ctrl+S"), self.gui)
        self.shortcut_stop = QtGui.QShortcut(Qt.QKeySequence("Ctrl+C"), self.gui)
        self.shortcut_exit = QtGui.QShortcut(Qt.QKeySequence("Ctrl+D"), self.gui)
        self.connect(self.shortcut_exit, QtCore.SIGNAL("activated()"), self.gui.close)

        # start update timer
        self.update_timer.start(33)

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

    def register_receiver(self, hostname, serial, id_rx):
        if not self.receivers.has_key(serial):
            rpc_adr = "tcp://" + hostname + ":" + str(6665 + id_rx)
            print rpc_adr
            probe_adr = "tcp://" + hostname + ":" + str(5555 + id_rx)
            print probe_adr
            self.receivers[serial] = receiver_interface.receiver_interface(rpc_adr, probe_adr)
            self.receivers[serial].samples_to_receive = self.samples_to_receive
            self.probe_manager.add_socket(self.receivers[serial].probe_address, 'complex64', self.receivers[serial].receive_samples)
            print serial, "registered"
            #threading.Thread(target = self.finish_register(serial)).start()

    def finish_register(self, serial):
        time.sleep(5)
        self.reset_receivers()
        print serial, "registered"

    def start_receivers(self):
        for key in self.receivers:
            receiver = self.receivers[key]
            receiver.request_samples()

    def reset_receivers(self):
        self.update_timer.stop()
        self.probe_manager = zeromq.probe_manager()
        for key in self.receivers:
            receiver = self.receivers[key]
            self.probe_manager.add_socket(receiver.probe_address, 'complex64', receiver.receive_samples)
            receiver.samples = []
            receiver.first_packet = True
            receiver.reception_complete = False
        self.connect(self.update_timer, QtCore.SIGNAL("timeout()"), self.probe_manager.watcher)
        self.update_timer.start(33)
        self.probe_manager.watcher()

    def process_results(self):
        if all(self.receivers[key].reception_complete for key in self.receivers) and len(self.receivers.items()) > 0:
            for key in self.receivers:
                receiver = self.receivers[key]
                receiver.first_packet = True
                receiver.reception_complete = False
            self.plot_receiver(self.gui.qwtPlotUsrp1, self.receivers["F57197"].samples)
            self.plot_receiver(self.gui.qwtPlotUsrp2, self.receivers["F571B0"].samples)
            correlation = self.correlate(self.receivers["F57197"],self.receivers["F571B0"])

    # plot cross correlation
    def plot_correlation(self, plot, samples):
        num_corr_samples = (len(samples) + 1)/2
        self.x = range(-num_corr_samples+1,num_corr_samples,1)
        self.y = samples
        # clear the previous points from the plot
        plot.clear()
        # draw curve with new points and plot
        curve = Qwt.QwtPlotCurve()
        curve.setPen(Qt.QPen(Qt.Qt.blue, 2))
        curve.attach(plot)
        curve.setData(self.x, self.y)
        plot.replot()

    def plot_receiver(self, qwtPlot, samples):
        x = range(0,len(samples),1)
        y = np.absolute(samples)
        # clear the previous points from the plot
        qwtPlot.clear()
        # draw curve with new points and plot
        curve = Qwt.QwtPlotCurve()
        curve.setPen(Qt.QPen(Qt.Qt.blue, 2))
        curve.attach(qwtPlot)
        curve.setData(x, y)
        qwtPlot.replot()

    def correlate(self, receiver1, receiver2):
        self.correlation = np.absolute(np.correlate(receiver1.samples, receiver2.samples, "full", False)).tolist()
        delay = self.correlation.index(np.max(self.correlation)) - self.samples_to_receive + 1
        self.gui.qwtPlotCorrelation.setAxisTitle(Qwt.QwtPlot.xBottom, "Delay: " + str(delay) + " samples")
        #print "Delay:", delay, "samples"
        self.plot_correlation(self.gui.qwtPlotCorrelation, self.correlation)

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
    parser.add_option("", "--num-samples", type="int", default="1600",
                      help="Number of samples in burst")
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


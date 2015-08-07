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

class gui(QtGui.QMainWindow):
    def __init__(self, window_name, options, parent=None):
        QtGui.QMainWindow.__init__(self, parent)

        # give Ctrl+C back to system
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        self.gui = uic.loadUi(os.path.join(os.path.dirname(__file__),'main_window.ui'), self)

        self.update_timer = Qt.QTimer()

        # socket addresses
        rpc_adr_usrp1 = "tcp://"+options.servername+":6666"
        rpc_adr_usrp2 = "tcp://"+options.clientname+":6667"
        probe_adr_usrp1 = "tcp://"+options.servername+":5556"
        probe_adr_usrp2 = "tcp://"+options.clientname+":5557"

        self.samples_to_receive = options.num_samples

        self.receivers = []
        self.receivers.append(receiver_interface.receiver_interface(rpc_adr_usrp1, probe_adr_usrp1))
        self.receivers.append(receiver_interface.receiver_interface(rpc_adr_usrp2, probe_adr_usrp2))

        # set samples to receive property of the class

        for receiver in self.receivers:
            receiver.samples_to_receive = self.samples_to_receive

        # ZeroMQ
        self.probe_manager = zeromq.probe_manager()
        for receiver in self.receivers:
            self.probe_manager.add_socket(receiver.probe_address, 'complex64', receiver.receive_samples)

        self.gui.setWindowTitle(window_name)
        self.init_plot(self.gui.qwtPlotUsrp1)
        self.init_plot(self.gui.qwtPlotUsrp2)
        self.gui.qwtPlotCorrelation.setTitle("Cross correlation")
        self.gui.qwtPlotCorrelation.setAxisTitle(Qwt.QwtPlot.xBottom, "Delay")
        self.gui.qwtPlotCorrelation.setAxisTitle(Qwt.QwtPlot.yLeft, "Amplitude")
        self.gui.qwtPlotCorrelation.setAxisScale(Qwt.QwtPlot.xBottom, -self.samples_to_receive, self.samples_to_receive)

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

    def start_receivers(self):
        for receiver in self.receivers:
            receiver.request_samples()

    def reset_receivers(self):
        self.update_timer.stop()
        self.probe_manager = zeromq.probe_manager()
        for receiver in self.receivers:
            self.probe_manager.add_socket(receiver.probe_address, 'complex64', receiver.receive_samples)
            receiver.samples = []
            receiver.first_packet = True
            receiver.reception_complete = False
        self.connect(self.update_timer, QtCore.SIGNAL("timeout()"), self.probe_manager.watcher)
        self.update_timer.start(33)
        self.probe_manager.watcher()

    def process_results(self):
        if all(receiver.reception_complete for receiver in self.receivers):
            for receiver in self.receivers:
                receiver.first_packet = True
                receiver.reception_complete = False
            self.plot_receiver(self.gui.qwtPlotUsrp1, self.receivers[0].samples)
            self.plot_receiver(self.gui.qwtPlotUsrp2, self.receivers[1].samples)
            correlation = self.correlate(self.receivers[0],self.receivers[1])

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


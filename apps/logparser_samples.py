#!/usr/bin/env python

from optparse import OptionParser
import matplotlib.pyplot as plt
import numpy as np
import sys
import math
import requests
from StringIO import StringIO
import time

###############################################################################
# Options Parser
###############################################################################
def parse_options():
    """ Options parser. """
    parser = OptionParser(usage="%prog: [options] file")
    parser.add_option("-a", "--acquisition", type="int", default=1,
                      help="Select acquisition")
    parser.add_option("-f", "--fft", action="store_true", default=False,
                      help="Activates FFT")
    parser.add_option("-c", "--cross-correlation", action="store_true", default=False,
                      help="Plot cross-correlation")
    parser.add_option("-s", "--save", action="store_true", default=False,
                      help="Save plots to files")
    (options, args) = parser.parse_args()
    if len(args)<1:   # if filename is not given
        parser.error('Filename not given')
    return options,args

###############################################################################
# Main
###############################################################################
if __name__ == "__main__":
    options,args = parse_options()

    f = open(args[0],"r")

    line_number = 1
    for line in f:
        if line_number == options.acquisition:
            acquisition = eval(eval(line))
            correlation = acquisition[0]
            receivers = acquisition[1]
        line_number+=1
    f.close()

    filename = args[0].split("/")[-1].split(".")[0]

    plt.rc('text', usetex=True)
    #plt.rc('font',**{'family':'serif','serif':['Helvetica']})
    plt.rcParams['text.latex.preamble'] = [
       r'\usepackage{siunitx}',   # i need upright \micro symbols, but you need...
       r'\sisetup{detect-all}',   # ...this to force siunitx to actually use your fonts
       r'\usepackage{amssymb, amsmath}',
       r'\usepackage[EULERGREEK]{sansmath}',  # load up the sansmath so that math -> helvet
       r'\sansmath'               # <- tricky! -- gotta actually tell tex to use!
       ]


    if options.cross_correlation:
        figure_correlation = plt.figure()
        figure_correlation.canvas.set_window_title(filename + "_correlation")
        ax_correlation = figure_correlation.add_subplot(111)
        ax_correlation.set_ylabel(r'Amplitude')
        ax_correlation.set_xlabel(r'Cross correlation')

        num_corr_samples = (len(correlation[0])+1)/2
        x = range(-num_corr_samples+1,num_corr_samples,1)
        ax_correlation.plot(x,correlation[0],"k")
        ax_correlation.plot(-np.array(x),correlation[1],"r")
        if options.save:
            plt.savefig(args[0].split(".")[0] + "_correlation.pdf", dpi=150)

    figure_samples = plt.figure()
    figure_samples.canvas.set_window_title(filename + "_samples")

    subplot_setup = str(len(receivers)) + "1"
    i = 1
    for receiver in receivers:
        s = subplot_setup + str(i)
        ax_samples = figure_samples.add_subplot(s)
        ax_samples.set_ylabel(r'Amplitude')
        ax_samples.set_xlabel(r'Samples')
        ax_samples.plot(receiver)
        i += 1
    if options.save:
        plt.savefig(args[0].split(".")[0] + "_samples.pdf", dpi=150)


    if options.fft:
        figure_samples_fft = plt.figure()
        figure_samples_fft.canvas.set_window_title(filename + "_samples_fft")

        subplot_setup = str(len(receivers)) + "1"
        i = 1
        for receiver in receivers:
            s = subplot_setup + str(i)
            ax_samples_fft = figure_samples_fft.add_subplot(s)
            ax_samples_fft.set_ylabel(r'Amplitude')
            ax_samples_fft.set_xlabel(r'Frequency[Hz]')
            y = 10*np.log10(np.fft.fftshift(np.fft.fft(receiver)))
            x = np.linspace(2510000000-25000000,2510000000+25000000,len(y))
            ax_samples_fft.plot(x,np.real(y))
            i += 1
        if options.save:
            plt.savefig(args[0].split(".")[0] + "_samples_fft.pdf", dpi=150)


    plt.show()


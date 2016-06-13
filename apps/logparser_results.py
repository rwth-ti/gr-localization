#!/usr/bin/env python

from optparse import OptionParser
import matplotlib.pyplot as plt
import numpy as np
from numpy import array
import sys
sys.path.append("../python")
#import gui_helpers
from pyproj import Proj, transform
from PIL import Image
import math
import requests
from StringIO import StringIO
from mpl_toolkits.basemap import Basemap
import time
from kalman import kalman_filter
import ConfigParser
from ConfigSectionMap import ConfigSectionMap

class parser():
    def __init__(self,bbox,filename,options):
        self.options = options
        if options.map:
            # map configuration
            self.figure_map = plt.figure(figsize=(16,10))
            self.figure_map.canvas.set_window_title(filename + "_map")
            self.init_map(bbox)


    def init_map(self,bbox):

        inProj = Proj(init='epsg:4326')
        outProj = Proj(init='epsg:3857')
        x0, y0 = transform(inProj,outProj,bbox[0],bbox[1])
        x1, y1 = transform(inProj,outProj,bbox[2],bbox[3])
        x = x1-x0
        y = y1-y0
        scale = math.ceil(math.sqrt(abs(x*y/0.3136)))

        r = requests.get("http://render.openstreetmap.org/cgi-bin/export?bbox=" + str(bbox)[1:-1] + "&scale=" + str(scale) + "&format=png", stream=True)

        if r.status_code == 200:
            img = Image.open(StringIO(r.content))

        #img = Image.open("../maps/ict_cubes.png")

        self.ax = self.figure_map.add_subplot(111, xlim=(x0,x1), ylim=(y0,y1), autoscale_on=False)

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

        #self.zp = gui_helpers.ZoomPan()
        #figZoom = self.zp.zoom_factory(self.ax, base_scale = 1.5)
        #figPan = self.zp.pan_factory(self.ax)

        self.figure_map.tight_layout(pad=0)
        #self.figure_map.patch.set_visible(False)
        self.ax.axis('off')
        #plt.show(block=False)

    def make_tikz_plot_delays(self, delay, correction, filename):
        filename = filename.split(".")[0] + ".tex"
        f = open(filename,"w")

        f.write("\\documentclass[conference]{IEEEtran}\n\\usepackage{graphicx}\n\\usepackage{amssymb, amsmath}\n\\usepackage[hyphens]{url}\n\\usepackage{pgfplots}\n%\\pgfplotsset{compat=1.10}\n\\begin{document}\n\\begin{figure}\n\\small\n\\centering\n\\newlength\\figureheight\n\\newlength\\figurewidth\n\\setlength\\figureheight{6.8cm}\n\\setlength\\figurewidth{0.9\\columnwidth}\n% defining custom colors\n\\definecolor{mycolor1}{rgb}{0.00000,0.75000,0.75000}%\n\\definecolor{mycolor2}{rgb}{0.75000,0.00000,0.75000}%\n\\definecolor{mycolor3}{rgb}{0.75000,0.75000,0.00000}%\n%\n\\begin{tikzpicture}\n\\begin{axis}[%\nwidth=\\figurewidth,\nheight=\\figureheight,\nscale only axis,\n%xmin=3,\n%xmax=8,\n%xtick={3, 4, 5, 6, 7, 8},\nxlabel={Acquisitions},\nxmajorgrids,\n%ymin=-14,\n%ymax=14,\nyminorticks=true,\nylabel={$\\Delta\\tau$[samples]},\ny label style={at={(0.07,0.5)}},\nymajorgrids,\nyminorgrids,\nlegend style={draw=black,fill=white,legend cell align=left}\n]\n")

        plot_1 = ""
        plot_2 = ""
        plot_3 = ""
        plot_4 = ""

        for i in range(1,len(delay)):
            plot_1 += str(i) + "\t" + str(int(delay[i-1][0])) + "\\\\\n"
            plot_2 += str(i) + "\t" + str(int(correction[i-1][0])) + "\\\\\n"
            if correction is not None:
                plot_3 += str(i) + "\t" + str(int(delay[i-1][1])) + "\\\\\n"
                plot_4 += str(i) + "\t" + str(int(correction[i-1][1])) + "\\\\\n"

        f.write("\\addplot [color=blue]\ntable[row sep=crcr]{%\n")
        f.write(plot_1)
        f.write("};\n\\addlegendentry{$\\Delta\\tau_{21}$(corrected)};\n")
        f.write("\\addplot [color=red]\ntable[row sep=crcr]{%\n")
        f.write(plot_2)
        f.write("};\n\\addlegendentry{$\\Delta\\tau_{21}$};\n")
        if correction is not None:
            f.write("\\addplot [color=green]\ntable[row sep=crcr]{%\n")
            f.write(plot_3)
            f.write("};\n\\addlegendentry{$\\Delta\\tau_{31}$(corrected)};\n")
            f.write("\\addplot [color=black]\ntable[row sep=crcr]{%\n")
            f.write(plot_4)
            f.write("};\n\\addlegendentry{$\\Delta\\tau_{31}$};\n")

        f.write("\n\\end{axis}\n\\end{tikzpicture}%\n\\end{figure}\n\\end{document}")
        f.close()

###############################################################################
# Options Parser
###############################################################################
def parse_options():
    """ Options parser. """
    parser = OptionParser(usage="%prog: [options] file")

    parser.add_option("", "--map", action="store_true", default=False,
                      help="Activate map plot")
    parser.add_option("", "--redo-kalman", action="store_true", default=False,
                      help="Do posterior Kalman filtering. Will overwrite real time filtered values.")
    parser.add_option("-c", "--config", type="str", default="./cfg_kalman_parser.cfg",
                      help="Configuration File for Kalman Filter")
    parser.add_option("", "--mapplot_mode", type="str", default="compare",
                      help="Points that should be plotted on the map.\ Possible options:\nfinal: just plot filtered Locations.\nraw:just Plot algorithm results without filtering. \ncompare: plot raw and filtered locations.")
    parser.add_option("", "--histogram-delays", action="store_true", default=False,
                      help="Activate histogram plot")
    parser.add_option("", "--histogram-location", action="store_true", default=False,
                      help="Activate histogram in meters plot")
    parser.add_option("", "--delay", action="store_true", default=False,
                      help="Activate delay history plot")
    parser.add_option("-s", "--save", action="store_true", default=False,
                      help="Save plots to files")
    parser.add_option("", "--delay-threshold", type="int", default="0",
                                  help="Threshold to filter TDOAs")
    (options, args) = parser.parse_args()
    if len(args)<1:   # if filename is not given
        parser.error('Filename not given')
    return options,args
    

    
    
###############################################################################
# Main
###############################################################################
if __name__ == "__main__":
    options,args = parse_options()
    if options.mapplot_mode not in ["final","raw","compare"]:
        sys.exit("invalid option %s:check logparser_results.py --help"%options.mapplot_mode)
    f = open(args[0],"r")
    f.readline()
    f.readline()
    f.readline()
    chan_x = []
    chan_y = []
    chan_x_kalman=[]
    chan_y_kalman=[]
    grid_x = []
    grid_y = []
    grid_x_kalman=[]
    grid_y_kalman=[]
    delays_list=[]
    delays_calibration_list = []
    delays_auto_calibration_list = []
    for line in f:
        acquisition = eval(eval(line))
        timestamp = acquisition[0]
        delays = acquisition[1]
        delays_calibration = acquisition[2]
        delays_auto_calibration = acquisition[3]
        sampling_rate = acquisition[4]
        frequency = acquisition[5]
        frequency_calibration = acquisition[6]
        calibration_position = acquisition[7]
        interpolation = acquisition[8]
        bandwidth = acquisition[9]
        samples_to_receive = acquisition[10]
        lo_offset = acquisition[11]
        bbox = acquisition[12]
        receivers_positions = acquisition[13]
        selected_positions = acquisition[14]
        receivers_gps = acquisition[15]
        receivers_antenna = acquisition[16]
        receivers_gain = acquisition[17]
        estimated_positions = acquisition[18]
        if len(acquisition) > 19:
            ref_receiver = acquisition[19]
        if (options.delay_threshold > 0):
            if all(abs(d)<options.delay_threshold for d in delays):
                if all(abs(d_c)<options.delay_threshold for d_c in delays_calibration):
                    if all(abs(d_ac)<options.delay_threshold for d_ac in delays_auto_calibration):
                        if estimated_positions.has_key("chan"):
                            chan_x.append(estimated_positions["chan"]["coordinates"][0])
                            chan_y.append(estimated_positions["chan"]["coordinates"][1])
                            if estimated_positions["chan"].has_key("kalman_coordinates"):
                                chan_x_kalman.append(estimated_positions["chan"]["kalman_coordinates"][0])
                                chan_y_kalman.append(estimated_positions["chan"]["kalman_coordinates"][1])
                        if estimated_positions.has_key("grid_based"):
                            grid_x.append(estimated_positions["grid_based"]["coordinates"][0])
                            grid_y.append(estimated_positions["grid_based"]["coordinates"][1])
                            if estimated_positions["grid_based"].has_key("kalman_coordinates"):
                                grid_x_kalman.append(estimated_positions["grid_based"]["kalman_coordinates"][0])
                                grid_y_kalman.append(estimated_positions["grid_based"]["kalman_coordinates"][1])
                        delays_list.append(delays)
                        delays_calibration_list.append(delays_calibration)
                        delays_auto_calibration_list.append(delays_auto_calibration)
        else:
            if estimated_positions.has_key("chan"):
                chan_x.append(estimated_positions["chan"]["coordinates"][0])
                chan_y.append(estimated_positions["chan"]["coordinates"][1])
                if estimated_positions["chan"].has_key("kalman_coordinates"):
                    chan_x_kalman.append(estimated_positions["chan"]["kalman_coordinates"][0])
                    chan_y_kalman.append(estimated_positions["chan"]["kalman_coordinates"][1])
            if estimated_positions.has_key("grid_based"):
                grid_x.append(estimated_positions["grid_based"]["coordinates"][0])
                grid_y.append(estimated_positions["grid_based"]["coordinates"][1])
                if estimated_positions["grid_based"].has_key("kalman_coordinates"):
                    grid_x_kalman.append(estimated_positions["grid_based"]["kalman_coordinates"][0])
                    grid_y_kalman.append(estimated_positions["grid_based"]["kalman_coordinates"][1])
            delays_list.append(delays)
            delays_calibration_list.append(delays_calibration)
            delays_auto_calibration_list.append(delays_auto_calibration)
    
    f.close()
    
    if options.redo_kalman: 
        #Kalman filtering the returned data
        cfg=ConfigParser.ConfigParser()
        
        cfg.read(options.config)
        init_kalman=ConfigSectionMap(cfg,"sectionOne")
        #init_kalman={"delta_t":1.5,"noise_factor":0.08,"filter_receivers":False,"noise_var_x":20,"noise_var_y":20,"model":"maneuvering","measurement_noise_chan":160*0.08,"measurement_noise_grid":160*0.08}
        if chan_x :
            init_kalman['algorithm']='chan'
            kalman=kalman_filter(init_kalman)
            measurements = np.column_stack((chan_x,chan_y))
            
            xk_1= np.hstack((measurements[0,:],np.zeros(kalman.get_state_size()-2)))#init state
            kalman_states= xk_1
            Pk_1=kalman.get_init_cov()
            for i in range(len(measurements)):
                xk_1,Pk_1=kalman.kalman_fltr(measurements[i-1,:],Pk_1,xk_1,"chan")
                if i>0:
                    kalman_states=np.vstack((kalman_states,xk_1))
                chan_x_kalman.append=kalman_states[:,0] 
                chan_y_kalman.append=kalman_states[:,1]         
        
        if grid_x:        
            init_kalman['algorithm']='grid_based'
            kalman=kalman_filter(init_kalman)
            measurements = np.column_stack((chan_x,chan_y))
            xk_1= np.hstack((measurements[0,:],np.zeros(kalman.get_state_size()-2)))#init state
            kalman_states= xk_1
            Pk_1=kalman.get_init_cov()
            for i in range(len(measurements)):
                xk_1,Pk_1=kalman.kalman_fltr(measurements[i-1,:],Pk_1,xk_1,"grid_based")
                if i>0:
                    kalman_states=np.vstack((kalman_states,xk_1))
                grid_x_kalman.append=kalman_states[:,0] 
                grid_y_kalman.append=kalman_states[:,1] 
        delays_calibrated = np.array(delays_list)
        delays_not_calibrated = np.array(delays_list) - np.array(delays_auto_calibration_list) - np.array(delays_calibration_list)

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
    params = {'text.usetex' : True,
          'font.size' : 16,
          }
    plt.rcParams.update(params) 

    p = parser(bbox,filename,options)

    if options.map:
        i = 1
        for rx in receivers_positions:
            p.ax.scatter(rx[0], rx[1],linewidths=2, marker='x', c='b', s=200, alpha=0.9)
            # set annotation RXx
            text = "RX" + str(i)
            i += 1
            p.ax.annotate(text, rx,fontweight='bold',bbox=dict(facecolor='w', alpha=0.9))

        p.figure_map.canvas.draw()

        #for i in range(0,len(chan_x)):
        #    p.ax.scatter(chan_x[i],chan_y[i],color="blue",marker="x")
        #    p.ax.scatter(grid_x[i],grid_y[i],color="red",marker="x")
        #    time.sleep(0.1)
        #    p.figure_map.canvas.draw()
        if options.mapplot_mode in ["raw","compare"]:
            p.ax.plot(chan_x,chan_y,color="blue",marker="x",linestyle="--",linewidth=0.2)
        if len (chan_x_kalman)>0 and options.mapplot_mode in ["final","compare"]:
            p.ax.plot(chan_x_kalman,chan_y_kalman,color="red",marker="x",linestyle="-",linewidth=0.2)
        #for i in range(len(measurements)):
        #    p.ax.annotate(i,(chan_x[i],chan_y[i]),fontsize=6)
        #    p.ax.annotate(i,(estimated_positions_kalman_chan[i,0],estimated_positions_kalman_chan[i,1]),fontsize=6)
        if len(grid_x)>0:
            p.ax.scatter(grid_x,grid_y,color="red",marker="x")
            if len (grid_x_kalman)>0:
                p.ax.scatter(grid_x_kalman,grid_y_kalman,color="red",marker="x")

        if options.save:
            plt.savefig(args[0].split(".")[0] + "_map.pdf", dpi=150)

    if options.histogram_location:
        chan_x_mean = np.mean(chan_x)
        chan_x_variance = np.var(chan_x)
        chan_y_mean = np.mean(chan_y)
        chan_y_variance = np.var(chan_y)
        label_chan_x= r'$chan_x$, $\mu=' + "{0:.2f}".format(chan_x_mean) + '$, $\sigma^2=' + "{0:.2f}".format(chan_x_variance) + '$'
        grid_y_variance = np.var(grid_y)
        label_chan_y= r'$chan_y$, $\mu=' + "{0:.2f}".format(chan_y_mean) + '$, $\sigma^2=' + "{0:.2f}".format(chan_y_variance) + '$'
        if len(grid_x)>0:
            grid_x_mean = np.mean(grid_x)
            grid_x_variance = np.var(grid_x)
            grid_y_mean = np.mean(grid_y)
            label_grid_x= r'$grid_x$, $\mu=' + "{0:.2f}".format(grid_x_mean) + '$, $\sigma^2=' + "{0:.2f}".format(grid_x_variance) + '$'
            label_grid_y= r'$grid_y$, $\mu=' + "{0:.2f}".format(grid_y_mean) + '$, $\sigma^2=' + "{0:.2f}".format(grid_y_variance) + '$'

        figure_hist_m = plt.figure()
        figure_hist_m.canvas.set_window_title(filename + "_histogram_location")
        ax_hist_m_x = figure_hist_m.add_subplot(211)
        ax_hist_m_x.set_xlabel(r'x[m]')
        # the histogram of the data
        offset=0.5
        bins = np.arange(np.min(chan_x)-1,np.max(chan_x)+1,299700000/(sampling_rate*interpolation))
        n, bins, patches = ax_hist_m_x.hist(chan_x, bins=bins, histtype='stepfilled', facecolor='blue', alpha=0.75, label=label_chan_x)
        if len(grid_x)>0:
            bins = np.arange(np.min(grid_x)-1,np.max(grid_x)+1,estimated_positions["grid_based"]["resolution"])
            n, bins, patches = ax_hist_m_x.hist(grid_x, bins=bins+offset, histtype='stepfilled', facecolor='red', alpha=0.75, label=label_grid_x)
        plt.legend()
        ax_hist_m_y = figure_hist_m.add_subplot(212)
        ax_hist_m_y.set_xlabel(r'y[m]')
        bins = np.arange(np.min(chan_y)-1,np.max(chan_y)+1,299700000/(sampling_rate*interpolation))
        n, bins, patches = ax_hist_m_y.hist(chan_y, bins=bins, histtype='stepfilled', facecolor='blue', alpha=0.75, label=label_chan_y)
        if len(grid_x)>0:
            bins = np.arange(np.min(grid_y)-1,np.max(grid_y)+1,estimated_positions["grid_based"]["resolution"])
            n, bins, patches = ax_hist_m_y.hist(grid_y, bins=bins+offset, histtype='stepfilled', facecolor='red', alpha=0.75, label=label_grid_y)
        plt.legend()

        if options.save:
            plt.savefig(args[0].split(".")[0] + "_histogram_location.pdf")

    if options.histogram_delays:
        d21mean = np.mean(delays_calibrated[:,0])
        d21variance = np.var(delays_calibrated[:,0])
        d31mean = np.mean(delays_calibrated[:,1])
        d31variance = np.var(delays_calibrated[:,1])
        labeld21 = r'$\Delta\tau_{21}$, $\mu=' + "{0:.2f}".format(d21mean) + '$, $\sigma^2=' + "{0:.2f}".format(d21variance) + '$'
        labeld31 = r'$\Delta\tau_{31}$, $\mu=' + "{0:.2f}".format(d31mean) + '$, $\sigma^2=' + "{0:.2f}".format(d31variance) + '$'

        figure_hist = plt.figure()
        figure_hist.canvas.set_window_title(filename + "_histogram_delays")
        ax_hist = figure_hist.add_subplot(111)
        ax_hist.set_xlabel(r'$\Delta\tau$[samples]')
        # the histogram of the data
        offset=0.5
        bins = np.arange(np.min(delays_calibrated[:,0])-1,np.max(delays_calibrated[:,0])+1)
        ax_hist.hist(delays_calibrated[:,0], bins=bins+offset, histtype='stepfilled', facecolor='green', alpha=0.75, label=labeld21)
        bins = np.arange(np.min(delays_calibrated[:,1])-1,np.max(delays_calibrated[:,1])+1)
        ax_hist.hist(delays_calibrated[:,1], bins=bins+offset, histtype='stepfilled', facecolor='red', alpha=0.75, label=labeld31)
        plt.legend()

        if options.save:
            plt.savefig(args[0].split(".")[0] + "_histogram_delays.pdf")

    if options.delay:
        figure_delay = plt.figure()
        figure_delay.canvas.set_window_title(filename + "_delay")
        ax_delay = figure_delay.add_subplot(111)
        ax_delay.set_ylabel(r'$\Delta\tau$[samples]')
        ax_delay.set_xlabel(r'Acquisitions')

        ax_delay.plot(delays_calibrated)
        ax_delay.plot(delays_not_calibrated)
        if options.save:
            p.make_tikz_plot_delays(delays_calibrated, delays_not_calibrated, args[0])

    plt.show()

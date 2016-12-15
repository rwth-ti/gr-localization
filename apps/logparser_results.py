#!/usr/bin/env python

from optparse import OptionParser
import matplotlib
matplotlib.rcParams['text.usetex'] = True
matplotlib.rcParams['text.latex.unicode'] = True
import matplotlib.pyplot as plt
from matplotlib import patches
import numpy as np
from numpy import array
import sys, warnings
from pyproj import Proj, transform
from PIL import Image
import math
import datetime
import calendar
import requests
from StringIO import StringIO
from mpl_toolkits.basemap import Basemap
import time
from kalman import kalman_filter
import ConfigParser
from ConfigSectionMap import ConfigSectionMap
from procrustres import procrustes


# print with approopriate resolution
np.set_printoptions(precision=20)
plt.rc('text', usetex=True)
plt.rc('font', family='serif')

class parser():
    def __init__(self,bbox,filename,options):
        self.options = options
        if options.map or options.plot_live:
            # map configuration
            self.figure_map = plt.figure(figsize=(14,10))
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
                      projection='tmerc', ax=self.ax, lon_0=lon_0, lat_0=lat_0,suppress_ticks=False)
        
        self.basemap.imshow(img, interpolation='lanczos', origin='upper')
        self.figure_map.tight_layout(pad=0)

        self.ax.axis('off')
        

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
        
def proj_basemap(bbox):
# for coordinate transformation
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

    basemap = Basemap(llcrnrlon=bbox[0], llcrnrlat=bbox[1],
                  urcrnrlon=bbox[2], urcrnrlat=bbox[3],
                  projection='tmerc', lon_0=lon_0, lat_0=lat_0)    
    return basemap
               
def gngga_reader(message):
    """ extract information from $GNGGA NMEA mesaage"""
    values = message.strip().split(",")[1:] # ommit the message type prefix and space, if existing; seperate values
    timing_info = values[0]
    #convert timing info into seconds (see ../GNGGAINFO.txt) 
    time_seconds = 60*(60*float(timing_info[:2])+float(timing_info[2:4]))+float(timing_info[4:])
    latitude = float(values[1][:2]) + float(values[1][2:])/60
    if values[2] != "N":
         latitude = -latitude
    longitude = float(values[3][:3]) + float(values[3][3:])/60 
    if values[4] != "E":
         longitude = -longitude
    fix_quality_lvl = int(values[5])
    fix_quality  = ""
    if fix_quality_lvl == 1:
        fix_quality = "fix"
    elif fix_quality_lvl == 4:
        fix_quality = "kinematic"
    elif fix_quality_lvl == 5:
        fix_quality = "float"
    else:
        fix_quality = "invalid"
    return latitude, longitude, time_seconds, fix_quality


def rmc_to_epoch_time(message):
    #print message
    values = message.strip().split(",")[1:]
    date = values[8]
    #print date
    # years 20xx
    day  =  int(date[:2])
    month = int(date[2:4])
    year = 2000+int(date[4:6])
    
    time = values[0]
    #print time
    hours = int(time[:2])
    minutes = int(time[2:4])
    seconds = int(time[4:6])
    
    dt = datetime.datetime(year, month, day, hours, minutes, seconds)
    #print dt
    # add milisecionds
    ms = float(time[6:])
    utime = calendar.timegm(dt.timetuple())
    #print utime 
    #print ms
    return utime + ms 

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
    parser.add_option("", "--mapplot-mode", type="str", default="compare",
                      help="Points that should be plotted on the map.\ Possible options:\nfinal: just plot filtered Locations.\nraw:just Plot algorithm results without filtering. \ncompare: plot raw and filtered locations.")
    parser.add_option("", "--ground-truth-log", type="str", default="",
                      help="log file (.ubx) from neo-m8p rtk-gps as ground truth reference. Will be included in map plot if given")
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
    parser.add_option("", "--histogram-errors", action="store_true", default=False,
                      help="Activate error histogram")
    parser.add_option("", "--procrustes", action="store_true", default=False,
                      help="error calculation with procrustes analysis")
    parser.add_option("", "--lineplot-velocity", action="store_true", default=False,
                      help="Activate velocity plot")
    parser.add_option("", "--lineplot-cov", action="store_true", default=False,
                      help="Activate velocity plot")
    parser.add_option("", "--plot-live", action="store_true", default=False,
                      help="real time plotting of measurements and ground-truth(if passed)")
    parser.add_option("", "--skip-acquisitions", type="int", default=0,
                      help="skip the first x acquisitons.")
    parser.add_option("", "--crop-ict", action="store_true", default= True, 
                      help="hack to crop the default gui map to the region in front of the receiers")
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
    chan_x_kalman = []
    chan_y_kalman = []
    velocity_estimates = []
    grid_x = []
    grid_y = []
    grid_x_kalman = []
    grid_y_kalman = []
    t_list = []
    gt_x_list = []
    gt_y_list = []
    gt_t_list = []
    gt_fix_list = []
    cov_x_list = []
    cov_y_list = []
    delays_list=[]
    delays_calibration_list = []
    delays_auto_calibration_list = []
    handles = []
    labels = []
    for line in f.readlines()[options.skip_acquisitions:]:
        acquisition = eval(eval(line))
        timestamp = acquisition[0]
        t_list.append(timestamp)
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
        ref_receiver = acquisition[19]
        if len(acquisition) > 20:
            try:
                auto_calibrate = acquisition[20]
                acquisition_time = acquisition[21]
                kalman_states = acquisition[22]
                init_settings_kalman = acquisition[23]
                if len(acquisition) > 23:
                    try:
                        cov_x_list.append(acquisition[25])
                        cov_y_list.append(acquisition[26])
                    except:
                        pass
            except:
                pass
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
        if kalman_states and options.lineplot_velocity:
            v=np.array((kalman_states[2:4]))
            velocity_estimates.append(np.sum(np.square(v)))
    f.close()
    
    
        
        
        
    if options.redo_kalman: 
        #Kalman filtering the returned data
        cfg=ConfigParser.ConfigParser()
        
        cfg.read(options.config)
        init_kalman = ConfigSectionMap(cfg,"sectionOne")
        if chan_x :
            init_kalman['algorithm']='chan'
            kalman = kalman_filter(init_kalman)
            measurements = np.column_stack((chan_x,chan_y))
            
            xk_1 = np.array(np.hstack((measurements[0,:],np.zeros(kalman.get_state_size()-2))))#init state
            
            kalman_states = xk_1
            Pk_1 = kalman.get_init_cov()
            for i in range(len(measurements)):
                xk_1,Pk_1 = kalman.kalman_fltr(measurements[i-1,:],Pk_1,xk_1,"chan")
                if i > 0:
                    kalman_states = np.vstack((kalman_states,xk_1))
                chan_x_kalman.append(xk_1[0] )
                chan_y_kalman.append(xk_1[1] )       
        
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
                grid_x_kalman.append(xk_1[:,0] )
                grid_y_kalman.append(xk_1[:,1] )  
    delays_not_calibrated=np.array([])
    delays_calibrated = np.array(delays_list)
    # check if measurements done with calibration or not
    if any(delays_calibration_list) and any(delays_auto_calibration_list):
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
    
    t_list=np.array(t_list)
    if any(np.diff(t_list) != acquisition_time):
        print "warning: measurements missing"
        print t_list[np.where(np.diff(t_list) != acquisition_time)]
        print t_list[np.add(np.where(np.diff(t_list) != acquisition_time),1)]
        print np.diff(t_list)  

    p = parser(bbox,filename,options)
    
    basemap = proj_basemap(bbox)
    if options.ground_truth_log != "":
        #try:
        fg = open(options.ground_truth_log)
        for line in fg.readlines():
            message_type = line.split(",")[0].strip()
            if message_type == "$GNGGA":
                gt_lat, gt_long, gt_time, gt_fix= gngga_reader(line)
                gt_x, gt_y = basemap(gt_long,gt_lat)
                gt_x_list.append(gt_x)
                gt_y_list.append(gt_y)
                gt_fix_list.append(gt_fix)
            elif message_type == "$GNRMC":
                timestamp = rmc_to_epoch_time(line)
                gt_t_list.append(timestamp)
        
        gt_t_list=np.array(gt_t_list)
        if any(np.diff(gt_t_list) != acquisition_time):
            print "warning: references missing"
            print np.where(np.diff(gt_t_list) != acquisition_time)
        # determine start and end time of overlapping timestamp vectors
        start_time = max(t_list[0],gt_t_list[0])
        end_time = min(t_list[-1],gt_t_list[-1])
        # find indices
        time_aligned_idx = np.where(np.logical_and(t_list>start_time, t_list<end_time))
        gt_time_aligned_idx = np.where(np.logical_and(gt_t_list>start_time, gt_t_list<end_time))
        # cut vectors (timestamps and locations)
        chan_x = np.array(chan_x)[time_aligned_idx]
        chan_y = np.array(chan_y)[time_aligned_idx]
        gt_x_list = np.array(gt_x_list)[gt_time_aligned_idx]
        gt_y_list = np.array(gt_y_list)[gt_time_aligned_idx]
        gt_fix_list = np.array(gt_fix_list)[gt_time_aligned_idx]
        if any(chan_x_kalman):
            chan_x_kalman = np.array(chan_x_kalman)[time_aligned_idx]
            chan_y_kalman = np.array(chan_y_kalman)[time_aligned_idx]
        if len(chan_x) == 0 and len(grid_x) == 0:
            sys.exit('no timestamp matches with ground-truth!')       
        if options.procrustes:
            d, Z_chan, tform = procrustes( np.vstack((gt_x_list,gt_y_list)).T,np.vstack((chan_x,chan_y)).T)
            print d
        else:
            Z_chan = np.vstack((chan_x,chan_y)).T
        xdiff_chan = Z_chan[:,0] -gt_x_list
        ydiff_chan = Z_chan[:,1] -gt_y_list
        err_chan = np.square(xdiff_chan) + np.square(ydiff_chan)
        rmse_chan = np.sqrt(np.mean(err_chan))
        
        err_chan_kalman = np.array([])
        if any(chan_x_kalman):
            if options.procrustes:
                d, Z_chan_kalman, tform = procrustes( np.vstack((gt_x_list,gt_y_list)).T,np.vstack((chan_x_kalman,chan_y_kalman)).T)
                Z_chan = np.dot(np.dot(tform["scale"],np.vstack((chan_x,chan_y)).T),tform["rotation"])+tform["translation"]
                print d
            else:
                Z_chan_kalman = np.vstack((chan_x_kalman,chan_y_kalman)).T
            xdiff_chan = Z_chan[:,0] -gt_x_list
            ydiff_chan = Z_chan[:,1] -gt_y_list
            err_chan = np.square(xdiff_chan) + np.square(ydiff_chan)
            rmse_chan = np.sqrt(np.mean(err_chan))

            xdiff_chan_kalman = Z_chan_kalman[:,0] -gt_x_list
            ydiff_chan_kalman = Z_chan_kalman[:,1] -gt_y_list
            err_chan_kalman = np.square(xdiff_chan_kalman) + np.square(ydiff_chan_kalman)
            rmse_chan_kalman = np.sqrt(np.mean(err_chan_kalman) )

        if options.histogram_errors:
            err_handles = []
            err_labels = []
            figure_errors = plt.figure()
            figure_errors.canvas.set_window_title(filename + "_histogram_errors")
            ax_errors = figure_errors.add_subplot(111)
            ax_errors.set_ylabel(r'$(\Delta s)^2 [m^2]$')
            ax_errors.set_xlabel(r'Acquisitions')
            ax_errors.set_ylim([0,20])
            chan_error_plot,=ax_errors.plot(err_chan,label="chan square error"+ '\n$RMSE =' + "{0:.3f}".format(rmse_chan)+"m$")
            
            handles.append(chan_error_plot)
            labels.append(chan_error_plot.get_label())
            if err_chan_kalman.any():
                chan_error_plot_kalman,=ax_errors.plot(err_chan_kalman,color="red",label="Kalman filtered square error(chan)"+ '\n$RMSE =' + "{0:.3f}".format(rmse_chan_kalman)+"m$")
                handles.append(chan_error_plot_kalman)
                labels.append(chan_error_plot_kalman.get_label())
            ax_errors.legend(handles,labels)
            if options.save:
                if options.procrustes:
                    plt.savefig(args[0].split("/")[-1].split(".")[0]+ "_procrustes"+ "_histogram_rmse.pdf")
                else:
                    plt.savefig(args[0].split("/")[-1].split(".")[0] + "_histogram_rmse.pdf")
    
    if options.plot_live:
        i = 1 
        if options.crop_ict:
            p.ax.axis([80,180,30,130])

            
        for rx in receivers_positions:
            p.ax.scatter(rx[0], rx[1],linewidths=2, marker='x', c='b', s=200, alpha=0.9)
            # set annotation RXx
            text = "Rx" + str(i)
            # index of logged reference receiver starts at 0 not at 1
            
            if i != (ref_receiver+1): #and options.reference_selection == "Manual":
                p.ax.annotate(text, rx,fontweight='bold', fontsize = 18,bbox=dict(facecolor='w', alpha=0.9))
            else:
                p.ax.annotate(text, rx,fontweight='bold', fontsize = 18,bbox=dict(facecolor='r', alpha=0.9, color="red"))
            i += 1
            
        p.figure_map.canvas.draw()

        plt.ion()
        for it in range(len(chan_x)):
            if options.mapplot_mode in ["raw","compare"]:
                # , for assigning the plots as line objects and not as tuples
                chan_scatter = p.ax.scatter(chan_x[it],chan_y[it],color="blue",marker="x")
            if len (chan_x_kalman)>0 and options.mapplot_mode in ["final","compare"]:
                # , for assigning the plots as line objects and not as tuples
                chan_kalman_scatter=p.ax.scatter(chan_x_kalman[it],chan_y_kalman[it],color="red",marker="x")
            if len(grid_x)>0:
                if options.mapplot_mode in ["raw","compare"]:
                    # , for assigning the plots as line objects and not as tuples
                    grid_scatter=p.ax.scatter(grid_x[it],grid_y[it],color="black",marker="x")

                if len (grid_x_kalman)>0 and options.mapplot_mode in ["final","compare"]:
                    # , for assigning the plots as line objects and not as tuples
                    grid_kalman_scatter = p.ax.scatter(grid_x_kalman[it],grid_y_kalman[it],color="yellow",marker="x")
            if len(gt_x_list)>0:
                # , for assigning the plots as line objects and not as tuples
                ground_truth_scatter=p.ax.scatter(gt_x_list[it],gt_y_list[it],color="cyan",marker="x")
            
            plt.pause(0.001)
    
    if options.map:
        i = 1 
        if options.crop_ict:
            p.ax.axis([88,172,57,114])
            p.basemap.drawmapscale(lon=6.06201, lat=50.77874, lon0=6.06201, lat0=50.77874, length=20,  units='m',barstyle='fancy',fontsize = 30, yoffset=1.2)
        else:
            p.basemap.drawmapscale(lon=bbox[0]+0.0002, lat=bbox[1]+0.00015, lon0=bbox[0]+0.0002, lat0=bbox[1]+0.00015, length=20,  units='m',barstyle='fancy',fontsize = 30,yoffset=1.2)
            
        """
        p.ax.set_xticks(np.linspace(80,180,int(10)))
        p.ax.set_yticks(np.linspace(30,130,int(10)))
        """
        
        for rx in receivers_positions:
            p.ax.scatter(rx[0], rx[1],linewidths=2, marker='x', c='b', s=200, alpha=0.9)
            # set annotation RXx
            text = "Rx" + str(i)
            # index of logged reference receiver starts at 0 not at 1
            rx = (rx[0]+2,rx[1]-0.5)
            if i != (ref_receiver+1): #and options.reference_selection == "Manual":
                p.ax.annotate(text, rx,fontweight='bold', fontsize = 30,bbox=dict(facecolor='w', alpha=0.9))
            else:
                p.ax.annotate(text, rx,fontweight='bold', fontsize = 30,bbox=dict(facecolor='r', alpha=0.7, color="red"))
            i += 1
            
                

        p.figure_map.canvas.draw()

        if options.mapplot_mode in ["raw","compare"]:
            p.ax.plot(chan_x[0],chan_y[0],color="blue",marker="x",markersize=10)
            # , for assigning the plots as line objects and not as tuples
            chan_plot = p.ax.plot(chan_x,chan_y,color="blue",marker="x",linestyle="--",linewidth=0.5,label = 'Raw locations (TDOA)')[0]
            handles.append(chan_plot)
            labels.append(chan_plot.get_label())
        if len (chan_x_kalman)>0 and options.mapplot_mode in ["final","compare"]:
            # , for assigning the plots as line objects and not as tuples
            chan_kalman_plot=p.ax.plot(chan_x_kalman,chan_y_kalman,color="red",marker="x",linestyle="--",linewidth=0.5,label = 'Kalman filtered')[0]

            handles.append(chan_kalman_plot)
            labels.append(chan_kalman_plot.get_label())
        if len(grid_x)>0:
            if options.mapplot_mode in ["raw","compare"]:
                # , for assigning the plots as line objects and not as tuples
                grid_plot=p.ax.plot(grid_x,grid_y,color="black",marker="x",linestyle="--",linewidth=0.5,label='Raw locations (grid-based)')[0]
                handles.append(grid_plot)
                labels.append(grid_plot.get_label())
            if len (grid_x_kalman)>0 and options.mapplot_mode in ["final","compare"]:
                # , for assigning the plots as line objects and not as tuples
                grid_kalman_plot, = p.ax.plot(grid_x_kalman,grid_y_kalman,color="yellow",marker="x",linestyle="-",linewidth=0.2,label='Kalman filtered (grid-based)')[0]
                handles.append(grid_kalman_plot)
                labels.append(grid_kalman_plot.get_label())
        if len(gt_x_list)>0:
            # , for assigning the plots as line objects and not as tuples
            ground_truth_plot=p.ax.plot(gt_x_list,gt_y_list,color="cyan",marker="x",linestyle="-",linewidth=0.5, label='Ground truth (GPS-RTK)')[0]
            handles.append(ground_truth_plot)
            labels.append(ground_truth_plot.get_label())
            for i in range(len(gt_fix_list)):
                if gt_fix_list[i] not in ["fix","kinematic"]:
                    print "rtk not fixed!"
                    #float_scatter = p.ax.scatter(gt_x_list[i],gt_y_list[i],color="m",marker="x")
        p.ax.legend(handles,labels,fontsize = 30)
        if options.save:
            #p.figure_map.tight_layout()
            plt.savefig(args[0].split("/")[-1].split(".")[0]+options.mapplot_mode + "_map.pdf", dpi=150)




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
        plt.autoscale(enable=True, axis='x', tight=True)
        if options.save:
            plt.savefig(args[0].split("/")[-1].split(".")[0] + "_histogram_location.pdf")

    if options.histogram_delays:
        #delays in ns for comparison purposes:
        print interpolation
        delays_calibrated_ns = np.true_divide(delays_calibrated,sampling_rate*interpolation*10**-9)
        d21mean = np.mean(delays_calibrated_ns[:,0])
        d21variance = np.var(delays_calibrated_ns[:,0])
        d31mean = np.mean(delays_calibrated_ns[:,1])
        d31variance = np.var(delays_calibrated_ns[:,1])
        labeld21 = r'$\Delta\tau_{21}$, $\mu=' + "{0:.5f}".format(d21mean) + '$, $\sigma^2=' + "{0:.5f}".format(d21variance) + '$'
        labeld31 = r'$\Delta\tau_{31}$, $\mu=' + "{0:.5f}".format(d31mean) + '$, $\sigma^2=' + "{0:.5f}".format(d31variance) + '$'

        figure_hist = plt.figure()
        figure_hist.canvas.set_window_title(filename + "_histogram_delays")
        ax_hist = figure_hist.add_subplot(111)
        ax_hist.set_xlabel(r'$\Delta\tau$[ns]')
        # the histogram of the data
        offset=0.5
        bins = np.arange(np.min(delays_calibrated_ns[:,0])-1,np.max(delays_calibrated_ns[:,0])+1,1/(sampling_rate*10**-9*interpolation))
        ax_hist.hist(delays_calibrated_ns[:,0], bins=bins+offset, histtype='stepfilled', facecolor='green', alpha=0.75, label=labeld21)
        bins = np.arange(np.min(delays_calibrated_ns[:,1])-1,np.max(delays_calibrated[:,1])+1,1/(sampling_rate*10**-9*interpolation))
        ax_hist.hist(delays_calibrated_ns[:,1], bins=bins+offset, histtype='stepfilled', facecolor='red', alpha=0.75, label=labeld31)
        plt.legend()
        plt.autoscale(enable=True, axis='x', tight=True)
        if options.save:
            plt.savefig(args[0].split("/")[-1].split(".")[0] + "_histogram_delays.pdf")

    if options.delay:
        figure_delay = plt.figure()
        figure_delay.canvas.set_window_title(filename + "_delay")
        ax_delay = figure_delay.add_subplot(111)
        ax_delay.set_ylabel(r'$\Delta\tau$[samples]')
        ax_delay.set_xlabel(r'Acquisitions')
        # check if measurements done with calibration or not
        if delays_not_calibrated.any():
            ax_delay.plot(delays_not_calibrated)
        if delays_calibrated.any():
            ax_delay.plot(delays_calibrated)
        plt.autoscale(enable=True, axis='x', tight=True)
        if options.save:
            p.make_tikz_plot_delays(delays_calibrated, delays_not_calibrated, args[0])
            
            
            
    if options.lineplot_velocity:
        figure_velocity = plt.figure()
        figure_velocity.canvas.set_window_title(filename + "_velocity")
        ax_velocity = figure_velocity.add_subplot(111)
        ax_velocity.set_ylabel(r'$v$[m/s]')
        ax_velocity.set_xlabel(r'Acquisitions')
        # check if measurements done with calibration or not
        if velocity_estimates:
            
            ax_velocity.plot(velocity_estimates)
        plt.autoscale(enable=True, axis='x', tight=True)
        if options.save:
            plt.savefig(args[0].split("/")[-1].split(".")[0] + "_lineplot_velocity.pdf")
            pass
            
    if options.lineplot_cov:
        handles_cov=[]
        labels_cov=[]
        figure_cov = plt.figure()
        figure_cov.canvas.set_window_title(filename + "_cov")
        ax_cov = figure_cov.add_subplot(111)
        ax_cov.set_ylabel(r'$\sigma^2[m^2]$')
        ax_cov.set_xlabel(r'Acquisitions')
        print cov_x_list
        print cov_y_list
        # check if measurements done with calibration or not
        if cov_x_list:
            
            cov_x_plot, = ax_cov.plot(cov_x_list, label =r"$\sigma_x^2$ estimate")
            handles_cov.append(cov_x_plot)
            labels_cov.append(cov_x_plot.get_label())
            cov_y_plot, = ax_cov.plot(cov_y_list,color="red",label =r"$\sigma_y^2$ estimate")
            handles_cov.append(cov_y_plot)
            labels_cov.append(cov_y_plot.get_label())
            ax_cov.legend(handles_cov,labels_cov)
        plt.autoscale(enable=True, axis='x', tight=True)
        if options.save:
            plt.savefig(args[0].split("/")[-1].split(".")[0] + "_lineplot_cov.pdf")
            pass
    '''        
    if options.histogram_errors:
        # histogram of errors
        xdiff_chan = chan_x - gt_x 
        ydiff_chan = chan_y - gt_y
        err_chan = np.sqrt(np.square(xdiff_chan) + np.square(ydiff_chan))
        mean_err_chan = np.mean(err_chan)
        print "chan error: ",err_chan
        if any(chan_x_kalman):
            xdiff_chan_kalman = chan_x_kalman - gt_x 
            ydiff_chan_kalman = chan_y_kalman - gt_y
            err_chan_kalman = np.sqrt(np.square(xdiff_chan_kalman) + np.square(ydiff_chan_kalman))
            mean_err_chan_kalman = np.mean(err_chan_kalman) 
            print "chan error with kalman filter: ",mean_err_chan_kalman
        if any(grid_x):
            xdiff_grid = grid_x - gt_x 
            ydiff_grid = grid_y - gt_y
            err_grid = np.sqrt(np.square(xdiff_grid) + np.square(ydiff_grid))
            mean_err_grid = np.mean(err_grid)
            print "chan error: ",err_chan
        if any(grid_x_kalman):
            xdiff_grid_kalman = grid_x_kalman - gt_x 
            ydiff_grid_kalman = grid_y_kalman - gt_y
            err_grid = np.sqrt(np.square(xdiff_grid_kalman) + np.square(ydiff_grid_kalman))
            mean_err_grid_kalman = np.mean(err_grid_kalman)
            print "chan error with kalman filter: ",mean_err_chan_kalman'''
    if not (options.map or options.plot_live):
        plt.autoscale(enable=True, axis='x', tight=True)
    plt.show()

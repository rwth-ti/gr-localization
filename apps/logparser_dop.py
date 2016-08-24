#!/usr/bin/env python

from optparse import OptionParser
import numpy as np
from scipy import interpolate
from numpy import array
import sys,os
import math
import requests
from StringIO import StringIO
import matplotlib.pyplot as plt
from matplotlib import patches
from pyproj import Proj, transform
from PIL import Image
import time
import pdb
import pprint
import ConfigParser
from joblib import Parallel, delayed
import multiprocessing
from mpl_toolkits.basemap import Basemap
sys.path.append("../python")
import receiver_interface,chan94_algorithm_filtered,chan94_algorithm,grid_based_algorithm    
import dop
       
       
def parse_options():
    """ Options parser. """
    parser = OptionParser(usage="%prog: [options] file")
    #parser.add_option("-a", "--algorithm", type="str", default="chan",
    #                  help="Select algoritm")
    parser.add_option("-g", "--geographical", action="store_true", default=True,
                      help="Geographical Map")
    #parser.add_option("", "--cov", action="store_true", default=False,
    #                  help="calculate positonal covariances")
    parser.add_option("", "--plot-dop", action="store_true", default=True,
                      help="calculate positonal covariances")
    parser.add_option("", "--reference-selection", type = "str", default="Min-DOP",
                      help="reference selection algorithm. Implemented options are: Manual, Min-DOP ")
    parser.add_option("", "--input-type", type = "str", default="log",
                      help="select input method: by default, a log of gr-localization is required.\n Option manual enables input of file containing receiver and bounding-box gps coordiantes.\n ((Rx1x,Rx1y),(Rx2x,...)...) (lonlowerBound, latLowerBound, lonUpperBound, latUpperBound)")
    parser.add_option("-s", "--save", action="store_true", default=True,
                      help="Save plots to files")
    (options, args) = parser.parse_args()
    if len(args) != 1 and optinon.input_type == "log":   # 
        parser.error('Filename not given')
    return options,args       


if __name__ == "__main__":
    options,args = parse_options()
    receivers_steps=[]

    if options.input_type == "log":
        f_results=open(args[0],"r")
        line=f_results.readline()
        line=f_results.readline()
        line=f_results.readline()
        line=f_results.readline()
        
        acquisition = eval(eval(line))
        timestamps = []
        ref_receivers = []
        timestamps.append(acquisition[0])
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
        # receivers_positions[1] = np.array(receivers_positions[1])#-array([2.5,0.5])
        selected_positions = acquisition[14]
        receivers_gps = acquisition[15]
        receivers_antenna = acquisition[16]
        receivers_gain = acquisition[17]
        estimated_positions = acquisition[18]
        ref_receivers.append(acquisition[19])
        ref_receiver = acquisition[19]
        try:
            auto_calibrate = acquisition[20]
            acquisition_time = acquisition[21]
            kalman_states = acquisition[22]
            init_settings_kalman = acquisition[23]
        except:
            pass
        idx = 1
        for line_number,line in enumerate(f_results):
            if line_number >= 20:
                break
            acquisition = eval(eval(line))
            timestamps.append(acquisition[0])
            ref_receivers.append(acquisition[19])

        f_results.close()
    elif options.input_type == "manual":
        f=open(args[0],"r")
        receivers_positions = eval(f.readline())
        gps_flag = True
        bbox = eval(f.readline())
        ref_receiver = 1
    filename = "dop_" + time.strftime("%d_%m_%y-%H:%M:%S")
    
    inProj = Proj(init='epsg:4326')
    outProj = Proj(init='epsg:3857')
    x0, y0 = transform(inProj,outProj,bbox[0],bbox[1])
    x1, y1 = transform(inProj,outProj,bbox[2],bbox[3])
    receivers_positions = np.array(receivers_positions)
    x = x1-x0
    y = y1-y0
    '''
    x0 = np.amin(receivers_positions,axis=0)[0] -15
    y0 = np.amin(receivers_positions,axis=0)[1] -15
    x1 = np.amax(receivers_positions,axis=0)[0] +15
    y1 = np.amax(receivers_positions,axis=0)[1] +15
    '''

    
    figure_map = plt.figure(figsize=(16,10))

    x_steps = np.linspace(0,x,x/100)
    y_steps = np.linspace(0,y,y/100)

    scale = math.ceil(math.sqrt(abs(x*y/0.3136)))

    r = requests.get("http://render.openstreetmap.org/cgi-bin/export?bbox=" + str(bbox)[1:-1] + "&scale=" + str(scale) + "&format=png", stream=True)

    if r.status_code == 200:
        img = Image.open(StringIO(r.content))

    #img = Image.open("../maps/ict_cubes.png")

    ax = figure_map.add_subplot(111, xlim=(x0,x1), ylim=(y0,y1), autoscale_on=False)
    
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

    basemap = Basemap(llcrnrlon=bbox[0], llcrnrlat=bbox[1],
                  urcrnrlon=bbox[2], urcrnrlat=bbox[3],
                  projection='tmerc', ax=ax, lon_0=lon_0, lat_0=lat_0,suppress_ticks=False)
    
    basemap.imshow(img, interpolation='lanczos', origin='upper')
    
    if options.plot_dop:
    
        reference_selection = options.reference_selection
        # TODO: axis ticks -> bad documentation
        #ax.set_xticks(np.linspace(x0,x1,int(x/10)))
        #ax.set_yticks(np.linspace(y0,y1,int(y/10)))
        #zp = gui_helpers.ZoomPan()
        #figZoom = zp.zoom_factory(ax, base_scale = 1.5)
        #figPan = zp.pan_factory(ax)

        figure_map.tight_layout(pad=0)
        #figure_map.patch.set_visible(False)
        ax.axis('off')
        #plt.show(block=False)
        
        
        receivers=dict()
        if gps_flag:
            for i in range(len(receivers_positions)):
                receivers_positions[i] = basemap(receivers_positions[i][1],receivers_positions[i][0])
        for receiver_idx in range(len(receivers_positions)):
            receivers[receiver_idx]=receiver_interface.receiver_interface(None,None,receiver_idx)
            receivers[receiver_idx].coordinates=receivers_positions[receiver_idx]
            receivers[receiver_idx].serial = receiver_idx
            
        i = 1 
        
        
        for rx in receivers_positions:
            
            ax.scatter(rx[0], rx[1],linewidths=2, marker='x', c='b', s=200, alpha=0.9)
            # set annotation RXx
            text = "RX" + str(i)
            # index of logged reference receiver starts at 0 not at 1
            
            if i != (ref_receiver+1) or options.reference_selection == "Min-DOP":
                ax.annotate(text, rx,fontweight='bold', fontsize = 7,bbox=dict(facecolor='w', alpha=0.9))
            else:
                ax.annotate(text, rx,fontweight='bold', fontsize = 7,bbox=dict(facecolor='r', alpha=0.9, color="red"))
            i += 1
        figure_map.canvas.draw()

        x_steps,y_steps = np.meshgrid(x_steps,y_steps)
        dops = np.ndarray(shape=np.shape(x_steps))
        references = np.ndarray(shape=np.shape(x_steps))
        
        for (x_it,y_it), value in np.ndenumerate(x_steps):
            if options.reference_selection == "Min-DOP":
                dops[x_it,y_it] = 10*np.log(dop.get_min_dop([x_steps[x_it,y_it],y_steps[x_it,y_it]],receivers))
                figure_map.canvas.set_window_title(filename + "_min-dop")
            else:
                dops[x_it,y_it]= 10*np.log(dop.calc_dop([x_steps[x_it,y_it],y_steps[x_it,y_it]],receivers, ref_receiver)[0])
                figure_map.canvas.set_window_title(filename + "_manual")
            #references[x_it,y_it] = dop.reference_selection_dop([x_steps[x_it,y_it],y_steps[x_it,y_it]],receivers)
        #dop_diff = dops_manual - dops_min
        
        xmin,ymin = np.unravel_index(dops.argmin(), dops.shape)
        xmax,ymax = np.unravel_index(dops.argmax(), dops.shape)
        print x_steps[xmin,ymin], y_steps[xmin,ymin],np.amin(dops)
        print x_steps[xmax,ymax], y_steps[xmax,ymax],np.amax(dops)
        

        grid = ax.pcolor(x_steps,y_steps,dops, cmap='coolwarm', alpha=0.7)
        # grid = ax.pcolor(x_steps,y_steps,references, cmap='prism', alpha=0.7)
        grid.set_clim(-3,30)
    
        if options.save:
            plt.savefig(filename + ".png", dpi=150, edgecolor='none')
    
        




        
    '''
    # num_cores = multiprocessing.cpu_count()
    if options.cov:
        f_samples = open(args[1],"r")

        
        for line_number, line in enumerate(f_samples):  
            #if line_number >= len(timestamps):
            #   break
                
            if line_number >= 20:
                break
            acquisition_samples = eval(eval(line))
            receivers_samples = acquisition_samples[0]
            receivers=dict()
            for receiver_idx in range(len(receivers_samples)):
                receivers[receiver_idx] = receiver_interface.receiver_interface(None,None,receiver_idx)
                receivers[receiver_idx].coordinates = receivers_positions[receiver_idx]
                receivers[receiver_idx].serial = receiver_idx
                receivers[receiver_idx].frequency = frequency
                receivers[receiver_idx].interpolation = interpolation
                receivers[receiver_idx].samp_rate = sampling_rate
                receivers[receiver_idx].samples_to_receive = samples_to_receive
                receivers[receiver_idx].samples = receivers_samples[receiver_idx] 
            for receiver in receivers.values():
                x = np.linspace(0,len(receiver.samples),len(receiver.samples))
                f = interpolate.interp1d(x, receiver.samples)
                x_interpolated = np.linspace(0,len(receiver.samples),len(receiver.samples) * interpolation)
                receiver.samples = f(x_interpolated)
            for receiver in receivers.values():
                if receiver != ref_receiver:
                    correlation = np.absolute(np.correlate(receiver.samples, receivers[ref_receiver].samples, "full", False))
                
            receivers_steps.append(receivers)
        f_samples.close()
            
        cov_file = "../log/results_cov_" + time.strftime("%d_%m_%y-%H:%M:%S") + ".txt"
        f = open(cov_file,"w")
        for x in x_steps:
            for y in y_steps:
                receivers = receivers_steps[0]
                coordinates = array([x,y])
                delay_calibration = [0,0]
                index_delay = 0
                print "iteration"

                pos_ref = receivers[ref_receiver].coordinates
                for i in range(0,len(receivers)):
                        if not ref_receiver == receivers.keys()[i]:
                            print receivers.keys()[i]
                            pos_receiver = receivers[i].coordinates
                            d_ref = np.linalg.norm(np.array(coordinates)-pos_ref)
                            d_receiver = np.linalg.norm(np.array(coordinates)-pos_receiver)
                            delay_true = (d_receiver-d_ref) * sampling_rate * interpolation / 299700000
                            delay_calibration[index_delay] = int(np.floor(delay_true))
                            index_delay += 1
                
                
                for i in range(len(receivers_steps)):
                    if options.algorithm=="chan"or options.algorithm=="both":
                        index_delay = 0
                        for j in range(0,len(receivers)):
                            if not ref_receiver == receivers_steps[i].keys()[j]:
                                print "true delay",delay_calibration[index_delay]
                                receivers_steps[i].values()[j].samples = np.roll(receivers_steps[i].values()[j].samples,delay_calibration[index_delay])
                                index_delay += 1
                        estimated_positions["chan"]=chan94_algorithm.localize(receivers_steps[i],ref_receiver,np.round(basemap(bbox[2],bbox[3])))
                        if i == 0:
                            chan_x = np.array(estimated_positions["chan"]["coordinates"][0])
                            chan_y = np.array(estimated_positions["chan"]["coordinates"][1])
                        else:
                            chan_x = np.hstack((chan_x,np.array(estimated_positions["chan"]["coordinates"][0])))
                            chan_y = np.hstack((chan_y,np.array(estimated_positions["chan"]["coordinates"][1])))
                chan_x_mean = np.mean(chan_x)
                chan_x_variance = np.var(chan_x)
                chan_y_mean = np.mean(chan_y)
                chan_y_variance = np.var(chan_y)
                chan_cov = np.cov([chan_x,chan_y])
                f.write(str(chan_cov.tolist()) + "," + str(chan_x_mean) + "," + str(chan_y_mean)+"\n")
        f.close()
        print "results written to: " +cov_file
        '''
    plt.show()

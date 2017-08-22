#!/usr/bin/env python

from optparse import OptionParser
import numpy as np
from scipy import interpolate
from numpy import array
import sys,os
import math
import requests
from StringIO import StringIO
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import colorbar
from matplotlib import patches
from pylab import get_cmap
from pyproj import Proj, transform
from PIL import Image, PngImagePlugin
import time
import pdb
import pprint
import ConfigParser
from mpl_toolkits.axes_grid1 import make_axes_locatable
#from joblib import Parallel, delayed
#import multiprocessing
from mpl_toolkits.basemap import Basemap
sys.path.append("../python")
import receiver_interface,chan94_algorithm_filtered,chan94_algorithm,grid_based_algorithm    
import dop
import osm_tile_download

matplotlib.rcParams['text.usetex'] = True
matplotlib.rcParams['text.latex.unicode'] = True
matplotlib.rcParams['font.size'] = 15
matplotlib.rcParams['font.family'] = "serif"
matplotlib.rcParams['font.serif'] = "cm10"
       
def get_spaced_colors(n):
    max_value = 16581375 #255**3
    interval = int(max_value / n)
    colors = [hex(I)[2:].zfill(6) for I in range(0, max_value, interval)]
    return [[int(i[:2], 16)/255, int(i[2:4], 16)/255, int(i[4:], 16)/255] for i in colors]
       
       
       
def parse_options():
    """ Options parser. """
    parser = OptionParser(usage="%prog: [options] file")
    #parser.add_option("-a", "--algorithm", type="str", default="chan",
    #                  help="Select algoritm")
    parser.add_option("-g", "--geographical", action="store_true", default=True,
                      help="Geographical Map")
    #parser.add_option("", "--cov", action="store_true", default=False,
    #                  help="calculate positonal covariances")
    parser.add_option("", "--plot-difference", action="store_true", default=False,
                      help="calculate difference between fixed reference and reference selection")
    parser.add_option("", "--plot-reference", action="store_true", default=False,
                      help="plot which reference is selected for min-dop")
    parser.add_option("", "--reference-selection", type = "str", default="Min-DOP",
                      help="reference selection algorithm. Implemented options are: Manual, Min-DOP ")
    parser.add_option("", "--ignore-sensors", type = "str", default="[]",
                      help="list of indices sensors that should be ignored, e.g. [1,5]")
    parser.add_option("", "--input-type", type = "str", default="log",
                      help="select input method: by default, a log of gr-localization is required.\n Option manual enables input of file containing receiver and bounding-box gps coordiantes.\n File structure: [(Rx1x,Rx1y),(Rx2x,...)...]\n\t[lonlowerBound, latLowerBound, lonUpperBound, latUpperBound]")
    parser.add_option("-s", "--save", action="store_true", default=False,
                      help="Save plots to files")
    (options, args) = parser.parse_args()
    if len(args) != 1 and options.input_type == "log":   # 
        parser.error('Filename not given')
    return options,args       


if __name__ == "__main__":
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    options,args = parse_options()
    receivers_steps=[]
    gps_flag = False
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
        bbox = list(acquisition[12])
        """
        print bbox
        bbox=list(bbox)
        bbox[0]=bbox[0]+0.0015
        bbox[2]=bbox[2]-0.001
        print bbox
        """
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
        f = open(args[0],"r")
        receivers_positions = eval(f.readline())
        print receivers_positions
        gps_flag = True
        bbox = eval(f.readline())
        print bbox
        ref_receiver = 0
    filename = "dop_" + time.strftime("%d_%m_%y-%H_%M_%S")
    
    inProj = Proj(init='epsg:4326')
    outProj = Proj(init='epsg:3857')
    x0, y0 = transform(inProj,outProj,bbox[0],bbox[1])
    x1, y1 = transform(inProj,outProj,bbox[2],bbox[3])
    receivers_positions = np.array(receivers_positions)
    x = x1 - x0
    y = y1 - y0
    '''
    x0 = np.amin(receivers_positions,axis=0)[0] -15
    y0 = np.amin(receivers_positions,axis=0)[1] -15
    x1 = np.amax(receivers_positions,axis=0)[0] +15
    y1 = np.amax(receivers_positions,axis=0)[1] +15
    '''

    
    figure_map = plt.figure(figsize=(20,10))

    # evaluate exactly 200 positions on each axis

    x_steps = np.linspace(0,x,400)
    '''
    lons = np.ndarray(shape=(len(x_steps)))
    for i,x in enumerate(x_steps):
        lons[i] = 
    '''
    y_steps = np.linspace(0,y,400)
    scale_x = x/400
    scale_y = y/400

    scale = math.ceil(math.sqrt(abs(x*y/0.3136)))
    ax = figure_map.add_subplot(111, xlim=(x0,x1), ylim=(y0,y1), autoscale_on=False)
    extent = ax.get_window_extent().transformed(figure_map.dpi_scale_trans.inverted())
    extent_width, extent_height = extent.width, extent.height
    extent_width *= figure_map.dpi
    extent_height *= figure_map.dpi
    # first check if OSM is available
    if osm_tile_download.check_osm():
        print "Using online map with bounding box", bbox
        # search for existing map with this bounding box
        if not any(i.find("map_" + "+".join(str(j).replace(".", ",") for j in bbox) + ".png") != -1 for i in
                   os.listdir("../maps/")):
            # download only if no map can be found
            #
            # scraping no longer allowed
            # use tile server instead
            lat_deg = bbox[1]
            lon_deg = bbox[0]
            delta_lat = bbox[3] - bbox[1]
            delta_lon = bbox[2] - bbox[0]
            # print "Tile download params", lat_deg, lon_deg, delta_lat, delta_lon, extent_width, extent_height
            img = osm_tile_download.get_image_cluster(lat_deg, lon_deg, delta_lat, delta_lon, extent_width,
                                                      extent_height)
            if not os.path.exists("../maps"):
                os.makedirs("../maps")
            # add bbox as meta data
            meta = PngImagePlugin.PngInfo()
            meta.add_text("bbox", str(bbox))
            img.save("../maps/map_" + "+".join(str(i).replace(".", ",") for i in bbox) + ".png", "png",
                     pnginfo=meta)
        else:
            # if available, open offline map instead
            img = Image.open("../maps/map_" + "+".join(str(i).replace(".", ",") for i in bbox) + ".png")
            bbox_meta = eval(img.info['bbox'])
            if list(bbox_meta) != bbox:
                print "Error: bounding boxes in meta data and filename do not agree"
                sys.exit()
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
    #int(np.linalg.norm(basemap(bbox[0],bbox[1])-basemap(bbox[2],bbox[1])))
    basemap.imshow(img, interpolation='lanczos', origin='upper')
    #FIXME:HARDCODED PARAMETERS
    barlength = 40
    x_offset_scale = 0.17
    y_offset_scale = 0.15
    basemap.drawmapscale(lon=bbox[0]-x_offset_scale*(bbox[0]-bbox[2]), lat=bbox[1]-y_offset_scale*(bbox[1]-bbox[3]), lon0=bbox[0]-x_offset_scale*(bbox[0]-bbox[2]), lat0=bbox[1]-y_offset_scale*(bbox[1]-bbox[3]), length = barlength,  units='m',barstyle='fancy',fontsize = 18)
    '''
    if options.input_type == "manual":
        for i, receiver_pos in enumerate(receivers_positions):
            receivers_positions[i] = basemap(receiver_pos[0],receiver_pos[1])
    '''

    #ax.axis([25,230,15,165])
    reference_selection = options.reference_selection
    receivers_ignored = eval(options.ignore_sensors)
    for idx in range(len(receivers_ignored)):
        receivers_ignored[idx] -= 1
    #TODO: axis ticks -> bad documentation
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
    skip_cnt = 0
    for receiver_idx in range(len(receivers_positions)):
        if (receiver_idx) not in receivers_ignored:
            receivers[receiver_idx - skip_cnt] = receiver_interface.receiver_interface(None,None,receiver_idx)
            receivers[receiver_idx - skip_cnt].coordinates=receivers_positions[receiver_idx]
            receivers[receiver_idx - skip_cnt].serial = receiver_idx
            print receivers[receiver_idx - skip_cnt].coordinates
        else:
            skip_cnt += 1
    i = 1 
    
    """
    for rx in receivers_positions:
        
        ax.scatter(rx[0], rx[1],linewidths=2, marker='x', c='b', s=200, alpha=0.9)
        # set annotation RXx
        text = "Rx" + str(i)
        # index of logged reference receiver starts at 0 not at 1
        rx = (rx[0]+2,rx[1])
        if i != (ref_receiver+1) or options.reference_selection == "Min-DOP" or options.plot_difference:
            ax.annotate(text, rx,fontweight='bold', fontsize = 18,bbox=dict(facecolor='w', alpha=0.9))
        else:
            ax.annotate(text, rx,fontweight='bold', fontsize = 18,bbox=dict(facecolor='r', alpha=0.9, color="red"))
        i += 1
    """
    figure_map.canvas.draw()

    x_steps,y_steps = np.meshgrid(x_steps,y_steps)
    

    if not options.plot_reference:
        dops_min = np.ndarray(shape=np.shape(x_steps))
        dops_manual = np.ndarray(shape=np.shape(x_steps))
        for (x_it,y_it), value in np.ndenumerate(x_steps):
            if options.reference_selection == "Min-DOP" or options.plot_difference:
                dops_min[x_it,y_it] = (1/0.3)*dop.get_min_dop([x_steps[x_it,y_it],y_steps[x_it,y_it]],receivers)
                figure_map.canvas.set_window_title(filename + "_min-dop")
            if options.reference_selection == "Manual" or options.plot_difference:
                dops_manual[x_it,y_it]= (1/0.3)*dop.calc_dop([x_steps[x_it,y_it],y_steps[x_it,y_it]],receivers, ref_receiver)[0]
                figure_map.canvas.set_window_title(filename + "_manual")
            #references[x_it,y_it] = dop.reference_selection_dop([x_steps[x_it,y_it],y_steps[x_it,y_it]],receivers)
        #dop_diff = dops_manual - dops_min
        if options.reference_selection == "Min-DOP":
            dops = dops_min
        else:
            dops = dops_manual
        xmin,ymin = np.unravel_index(dops.argmin(), dops.shape)
        xmax,ymax = np.unravel_index(dops.argmax(), dops.shape)
        print x_steps[xmin,ymin], y_steps[xmin,ymin],np.amin(dops)
        print x_steps[xmax,ymax], y_steps[xmax,ymax],np.amax(dops)

        dops[dops>5] = 5
        dops[dops<10**-1] = 10**-1
        levels = np.arange(0, 5, 0.5)
        #grid = plt.pcolor(x_steps,y_steps,dops, cmap='coolwarm', alpha=0.5, linewidth=0, edgecolor = 'face', rasterized = True)
        ctr = plt.contour(x_steps, y_steps, dops,levels,colors = "k")
        plt.clabel(ctr, levels, inline=1,fmt='%1.1f',fontsize=12)
        #, extent=[x_steps.min(), x_steps.max(), y_steps.min(), y_steps.max()]
        #plt.imshow(dops, interpolation='nearest', cmap='coolwarm',aspect = "auto", alpha=0.5)
        #grid = ax.pcolor(x_steps,y_steps,references, cmap='prism', alpha=0.7)
        #grid.set_clim(-3,30)
        #divider = make_axes_locatable(ax)
        #cax = divider.append_axes("right", size="2%", pad=0.02)
        #cbar = basemap.colorbar()
        #cbar.ax.tick_params(labelsize = 36) 
        #cbar.make_axes(grid,location="right")
        #cbar.set_label("DOP",fontsize = 28,labelpad = 20)
        #plt.title("DOP")
    else:
        cmap_cont = matplotlib.cm.brg
        #cmap_cont(range(0,len(receivers_positions)))
        '''
        cl = cmap_cont([x *int(np.floor(256/(len(receivers_positions)-1))) for x in range(0, len(receivers_positions)-1)].append(255))
        print [x *int(np.floor(256/(len(receivers_positions)-1))) for x in range(0, len(receivers_positions)-1)].append(255)
        '''
        cl = cmap_cont(range(0,256,int(255/(len(receivers_positions)-1))))
        cmap = matplotlib.colors.ListedColormap(cl)
        references = np.ndarray(shape=np.shape(x_steps))
        figure_map.canvas.set_window_title(filename + "_references")
        for (x_it,y_it), value in np.ndenumerate(x_steps):
            references[x_it,y_it],dop_location,H = dop.reference_selection_dop([x_steps[x_it,y_it],y_steps[x_it,y_it]],receivers)
            if references[x_it,y_it] in receivers_ignored:
                references[x_it,y_it] += 1 + np.where(receivers_ignored == references[x_it,y_it])[0][0]
            
        #rfx = plt.pcolor(x_steps,y_steps,references, cmap=cmap, alpha=0.5, linewidth=0, edgecolor = 'face', rasterized = True)
        rfx = plt.imshow(references, interpolation='nearest', cmap=cmap, origin = 'lower', alpha=0.5, extent=[0,x,0,y])
        #references = transform_scalar(references, lons, lats, nx, ny)
        #rfx = basemap.imshow(references, cmap=cmap, alpha=0.5)
        
        cbar = basemap.colorbar(rfx)
        cbar.ax.get_yaxis().set_ticks([])
        for j in range(len(receivers_positions)):
            cbar.ax.text(.5, (10.1/len(receivers_positions) * (j+0.5)) / 10.0, "$%i$"%(j+1), ha='center', va='center',fontsize=28)
        cbar.set_label("Selected reference sensor",fontsize = 28,labelpad = 20)
        
    # For nice looking but hardcoded plots in paper
    #colorlist = ['#7F7FFF','#FE7F7F','#80FD7F']
        cmap_cont = matplotlib.cm.brg
        cl = cmap_cont(range(0,256,int(255/(len(receivers_positions)-1))))
        colorlist = cl
    for idx in range(len(receivers_ignored)):
        receivers_ignored[idx] += 1
    for rx in receivers_positions:
        if i not in receivers_ignored:
            if options.plot_reference:
                color = colorlist[i-1]
            else:
                color = "b"
            ax.scatter(rx[0], rx[1],linewidths=2, marker='^', c=color, s=400, alpha=1)
            # set annotation RXx
            text = "Rx" + str(i)
            # index of logged reference receiver starts at 0 not at 1
            rx = (rx[0]+2,rx[1]+2.5)
            
            if i != (ref_receiver+1) or options.reference_selection == "Min-DOP" or options.plot_difference:
                ax.annotate(text, rx,fontweight='bold', fontsize = 16,bbox=dict(facecolor='w', alpha=0.9))
            else:
                ax.annotate(text, rx,fontweight='bold', fontsize = 16,bbox=dict(facecolor='r', alpha=0.9, color="red"))
        i += 1
    if options.save:
        plt.savefig(filename + ".pdf", dpi=100, edgecolor='none')

        




        
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

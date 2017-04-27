#!/usr/bin/env python

from optparse import OptionParser
import numpy as np
from scipy import interpolate, signal
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import patches
import numpy as np
import sys, warnings, os
from pyproj import Proj, transform
from PIL import Image
import math
import requests
from StringIO import StringIO
import time
import pprint
import ConfigParser
import pdb
from mpl_toolkits.basemap import Basemap
sys.path.append("../python")
import receiver_interface, chan94_algorithm, grid_based_algorithm
from interpolation import corr_spline_interpolation
from ConfigSectionMap import ConfigSectionMap
from chan94_algorithm import estimate_delay_interpolated
from procrustes import procrustes
import mds_self_tdoa
matplotlib.rcParams['text.usetex'] = True
matplotlib.rcParams['text.latex.unicode'] = True
c = 299700000

def parse_options():
    """ Options parser. """
    parser = OptionParser(usage="%prog: [options] file")
    parser.add_option("-g", "--geographical", action="store_true", default=True,
                      help="Geographical Map")
    parser.add_option("-m", "--map", action="store_true", default=False,
                      help="plot results of self localization on map")
    parser.add_option("", "--history", action="store_true", default=False,
                      help="plot delay and transmitter history for whole experiment")
    parser.add_option("-r", "--replay", action="store_true", default=False,
                      help="replay whole experiment with sample log")
    parser.add_option("-c", "--config", type="str", default="",
                      help="Configuration file for self localization algorithm")
    parser.add_option("", "--crop-ict", action="store_true", default= False, 
                      help="hack to crop the default gui map to the region in front of the receiers")
    parser.add_option("-s", "--save", action="store_true", default=False,
                      help="Save plots to files")
    (options, args) = parser.parse_args()
    if len(args) == 0 or (len(args) == 1 and options.replay) or len(args) >2:
        parser.error('Invalid number of inputs!')
    return options,args

if __name__ == "__main__":
    options,args = parse_options()
    receivers_steps=[]
    # args[0] ^= selfloc log   
    f_results=open(args[0],"r")
    '''
    line=f_results.readline()
    line=f_results.readline()
    line=f_results.readline()
    line=f_results.readline()
    '''
    #header
    line = f_results.readline()
    acquisition = eval(line)
    sampling_rate = acquisition[0]
    frequency = acquisition[1]
    frequency_calibration = acquisition[2]
    calibration_position = acquisition[3]
    interpolation = acquisition[4]
    bandwidth = acquisition[5]
    samples_to_receive = acquisition[6]
    lo_offset = acquisition[7]
    bbox = acquisition[8]
    receivers_positions = acquisition[9]
    selected_positions = np.array(acquisition[10])
    receivers_gps = acquisition[11]
    receivers_antenna = acquisition[12]
    receivers_gain = acquisition[13]
    selfloc_average_length = acquisition[14]
    num_anchors = acquisition[15]
    anchor_average = acquisition[16]
    ref_receiver = acquisition[17]

    #selfloc results
    transmitter_history = eval(f_results.readline())
    timestamp_history = eval(f_results.readline())
    delay_tensor = np.array(eval(f_results.readline()))
    D = np.array(eval(f_results.readline()))
    anchor_loop_delay_history = eval(f_results.readline())
    anchor_positions = np.array(eval(f_results.readline()))
    anchor_positions_procrustes = np.array(eval(f_results.readline()))
    anchor_gt_positions = np.array(eval(f_results.readline()))
    pos_selfloc = np.array(eval(f_results.readline()))
    pos_selfloc_procrustes = np.array(eval(f_results.readline()))
    #tform = dict(zip(eval(f_results.readline()),eval(f_results.readline())))
    #print tform

    print "+".join(str(j).replace(".",",") for j in bbox)
    if not any(i.find("+".join(str(j).replace(".",",") for j in bbox))!= -1 for i in os.listdir("../maps/") ):
        # request only if no map can be found
        r = requests.get("http://render.openstreetmap.org/cgi-bin/export?bbox=" + str(bbox)[1:-1] + "&scale=" + str(scale) + "&format=png", stream=True)
        if r.status_code == 200:
            img = Image.open(StringIO(r.content))
            if not os.path.exists("../maps"):
                    os.makedirs("../maps")
            img.save("../maps/map"+"+".join(str(i).replace(".",",") for i in bbox)+".png")
        else:
            print "OSM error"
    else:
        # if available, open offline map instead
        img = Image.open("../maps/map"+"+".join(str(i).replace(".",",") for i in bbox)+".png")
    #img = Image.open("../maps/ict_cubes.png")

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
    if options.geographical:
        basemap = Basemap(llcrnrlon=bbox[0], llcrnrlat=bbox[1],
                              urcrnrlon=bbox[2], urcrnrlat=bbox[3],
                              projection='tmerc', lon_0=lon_0, lat_0=lat_0)
    else:
        basemap = Basemap(width=bbox[2], height=bbox[3],
                          lon_0=0,lat_0=0,
                          projection='tmerc')
    
    # adjust when logging is tested
    inProj = Proj(init='epsg:4326')
    outProj = Proj(init='epsg:3857')
    x0, y0 = transform(inProj,outProj,bbox[0],bbox[1])
    x1, y1 = transform(inProj,outProj,bbox[2],bbox[3])
    receivers_positions = np.array(receivers_positions)
    x = x1 - x0
    y = y1 - y0
    f_results.close()
    
    scale = math.ceil(math.sqrt(abs(x*y/0.3136)))
    
    if options.map:
        figure_map = plt.figure(figsize=(20,10))
        ax = figure_map.add_subplot(111, xlim=(x0,x1), ylim=(y0,y1), autoscale_on=False)
        basemap.imshow(img, interpolation='lanczos', origin='upper')
        i = 1
        if options.crop_ict:
            ax.axis([88,172,57,114])
            basemap.drawmapscale(lon=6.06201, lat=50.77874, lon0=6.06201, lat0=50.77874, length=20,  units='m',barstyle='fancy',fontsize = 30, yoffset=1.2)
        else:
            basemap.drawmapscale(lon=bbox[0]-0.1*(bbox[0]-bbox[2]), lat=bbox[1]-0.07*(bbox[1]-bbox[3]), lon0=bbox[0]-0.1*(bbox[0]-bbox[2]), lat0=bbox[1]-0.07*(bbox[1]-bbox[3]), length=int(0.1 * np.linalg.norm(np.array(basemap(bbox[0],bbox[1])) - np.array(basemap(bbox[2],bbox[1])))),  units='m',barstyle='fancy',fontsize = 18)            
        """
        ax.set_xticks(np.linspace(80,180,int(10)))
        ax.set_yticks(np.linspace(30,130,int(10)))
        """
        x_diff_anchors = anchor_positions_procrustes[:,0] - anchor_gt_positions[:,0]
        y_diff_anchors = anchor_positions_procrustes[:,1] - anchor_gt_positions[:,1]
        rmse_anchors = np.sqrt(np.mean(np.square(x_diff_anchors) + np.square(y_diff_anchors)))

        x_diff_positions = pos_selfloc_procrustes[:,0] - receivers_positions[:,0]
        y_diff_positions = pos_selfloc_procrustes[:,1] - receivers_positions[:,1]
        rmse_positions = np.sqrt(np.mean(np.square(x_diff_positions) + np.square(y_diff_positions)))
        
        for rx in receivers_positions:
            if i == 1:
                ax.scatter(rx[0], rx[1],linewidths=2, marker='^', c='w', s=400, alpha=0.9, label = "Receiver ground-truth position")
            else:
                ax.scatter(rx[0], rx[1],linewidths=2, marker='^', c='w', s=400, alpha=0.9)
            # set annotation RXx
            text = "Rx" + str(i)
            # index of logged reference receiver starts at 0 not at 1
            rx = (rx[0] + 5, rx[1] - 1)
            #if i != (ref_receiver+1): #and options.reference_selection == "Manual":
            ax.annotate(text, rx,fontweight='bold', fontsize = 30,bbox=dict(facecolor='w', alpha=0.9))
            i += 1
        i = 1
        for rx in pos_selfloc_procrustes:
            if i == 1:
                ax.scatter(rx[0], rx[1],linewidths=2, marker='^', c="r", s=400, alpha=0.9, label = "Final result receivers, RMSE: "+ "${0:.3f}".format(rmse_positions)+"m$")
            else:
                ax.scatter(rx[0], rx[1],linewidths=2, marker='^', c="r", s=400, alpha=0.9)
            # set annotation RXx
            text = "Rx" + str(i)
            # index of logged reference receiver starts at 0 not at 1
            rx = (rx[0] + 5, rx[1] - 1)
            #if i != (ref_receiver+1): #and options.reference_selection == "Manual":
            ax.annotate(text, rx,fontweight='bold', fontsize = 30,bbox=dict(facecolor='r', alpha=0.9))
            i += 1
        i = 1
        for rx in anchor_gt_positions:
            if i == 1:
                ax.scatter(rx[0], rx[1],linewidths=2, marker='^', c="g", s=400, alpha=0.9, label = "Anchor ground-truth position")
            else:
                ax.scatter(rx[0], rx[1],linewidths=2, marker='^', c="g", s=400, alpha=0.9)
            # set annotation RXx
            text = "ANC" + str(i)
            # index of logged reference receiver starts at 0 not at 1
            rx = (rx[0] + 5, rx[1] - 1)
            i += 1
        i = 1
        for rx in anchor_positions_procrustes:
            if i == 1:
                ax.scatter(rx[0], rx[1],linewidths=2, marker='^', c="b", s=400, alpha=0.9, label = "Result for anchors, RMSE: "+ "${0:.3f}".format(rmse_anchors)+"m$")
            else:
                ax.scatter(rx[0], rx[1],linewidths=2, marker='^', c="b", s=400, alpha=0.9)
            # set annotation RXx
            text = "ANC" + str(i)
            # index of logged reference receiver starts at 0 not at 1
            rx = (rx[0] + 5, rx[1] - 1)
            i += 1
        
        ax.legend()

        if options.save:
            #p.figure_map.tight_layout()
            plt.savefig(args[0].split("/")[-1].split(".")[0]+options.mapplot_mode + "_map.pdf", dpi=150)

    if options.history:
        #make delay plots here
        #maybe useful for debugging, but too much effort?
        figure_history = plt.figure()
        axis_history = figure_history.add_subplot(111)
        axis_history.plot(transmitter_history)
        print delay_history
        print anchor_loop_delay_history
        # concatenate D and anchor_delay_history! for delays
        pass

    plt.show()
    
    
    if options.replay:
        # first set of samples: delays for algorithm. length: average_length*num_sensors
        # second set: delays for anchoring. length: average_length* num_anchors
        # mabe build in check?
        f_samples = open(args[1],"r")
        for line_number, line in enumerate(f_samples.readlines()):  
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
                receivers[receiver_idx].correlation_interpolation = True
                receivers[receiver_idx].measurement_noise = 10
                receivers[receiver_idx].selected_position = "selfloc"
            for receiver in receivers.values():
                x = np.linspace(0,len(receiver.samples),len(receiver.samples))
                f = interpolate.interp1d(x, receiver.samples)
                x_interpolated = np.linspace(0,len(receiver.samples),len(receiver.samples) * interpolation)
                #receiver.samples = f(x_interpolated)
            for receiver in receivers.values():
                if receiver != ref_receiver:
                    correlation = np.absolute(np.correlate(receiver.samples, receivers[ref_receiver].samples, "full"))
                
            receivers_steps.append(receivers)

        f_samples.close()
        receivers_selfloc = receivers_steps[:len(receivers_positions)*selfloc_average_length]
        receivers_anchoring = receivers_steps[len(receivers_positions)*selfloc_average_length:]
        if len(receivers_anchoring) != anchor_average * num_anchors:
            print "Alert: samples missing!"
            pdb.set_trace()
        for cnt_j in range(len(receivers_selfloc)/selfloc_average_length):
            for cnt_average in range(selfloc_average_length):
                curr_rx_set = receivers_selfloc[cnt_j*selfloc_average_length + cnt_average]
                for cnt_l, rx_l in enumerate(curr_rx_set):
                    for cnt_k, rx_k in enumerate(curr_rx_set):
                        if cnt_j != cnt_l and cnt_j != cnt_k and cnt_l != cnt_k:
                            window_size = 13
                            #by now ugly hack, rethink later
                            delay_tensor[cnt_j,cnt_l,cnt_k,cnt_average] = corr_spline_interpolation(curr_rx_set.values()[cnt_l].samples, curr_rx_set.values()[cnt_k].samples, window_size)[1] 
                            #print(correlate({receivers.keys()[cnt_l]:receivers.values()[cnt_k],receivers.keys()[cnt_l]:receivers.values()[cnt_k]})[1])
                        else:
                            delay_tensor[cnt_j,cnt_l,cnt_k,cnt_average] = 0.0
        D = np.ndarray(shape=(len(receivers_positions),len(receivers_positions),len(receivers_positions)))
        sum_square_tdoa = 0
        for j in range(len(receivers_positions)):
            for l in range(len(receivers_positions)):
                for k in range(len(receivers_positions)):
                    # average distance differences
                    tdoa = sum(delay_tensor[j,l,k]) / selfloc_average_length / sampling_rate * 299700000.0
                    sum_square_tdoa += tdoa**2
                    D[j,l,k] = tdoa

        pos_selfloc = None
        stress = 10
        pos_selfloc, stress = mds_self_tdoa.selfloc(D,basemap(bbox[2],bbox[3]), sum_square_tdoa, pos_selfloc, 500, 1.0, stress)
        anchor_loop_delays = []
        anchor_loop_delay_history = []
        anchor_positions = []
        for receivers in receivers_anchoring:
            for receiver in receivers:
                receivers.values()[receiver].coordinates_selfloc = pos_selfloc[receiver]
                window_size = 13
                #TODO: include averaging
                if not ref_receiver == receiver:
                    delay = corr_spline_interpolation(receivers.values()[receiver].samples, receivers.values()[ref_receiver].samples, window_size)[1]
            anchor_loop_delays.append(delay)
            # no sensor is transmitting
            transmitter_history.append(-1)
            if len(anchor_loop_delays) == anchor_average:
                delay_mean = np.array(anchor_loop_delays).mean(0)
                anchor_positions.append(chan94_algorithm.localize(receivers, ref_receiver, np.round(basemap(bbox[2],bbox[3])), delay_mean)["coordinates"])
                anchor_loop_delay_history.append(anchor_loop_delays)
                anchor_loop_delays = []

        anchor_gt_positions = np.array(anchor_gt_positions)
        anchor_positions = np.array(anchor_positions)
        d, coordinates_procrustes, tform = procrustes(anchor_gt_positions, anchor_positions, scaling = False)
        print(coordinates_procrustes)
        reflection = np.linalg.det(tform["rotation"])
        pos_selfloc_procrustes =  np.dot(pos_selfloc,tform["rotation"]) + tform["translation"]
        pdb.set_trace()
        
        results_file="../log/results_post_selfloc" + time.strftime("%d_%m_%y-%H:%M:%S") + ".txt"
        if any(receivers):
            f=open(results_file,"w")
            pprint.pprint("##########################################################################################################################################################################################", f)
            pprint.pprint("time,delays(1-2,1-3,1-X...),delays_calibration(1-2,1-3,1-X...),delays_auto_calibration(1-2,1-3,1-X...),sampling_rate,frequency,frequency_calibration,calibration_position,interpolation,bandwidth,samples_to_receive,lo_offset,bbox,receivers_positions,selected_positions,receivers_gps,receivers_antenna,receivers_gain,estimated_positions,index_ref_receiver,auto_calibrate,acquisition_time", f)
            pprint.pprint("##########################################################################################################################################################################################", f)
        else:
            sys.exit("log import failed")   
    
        print "selfloc test results written to: \n" +results_file
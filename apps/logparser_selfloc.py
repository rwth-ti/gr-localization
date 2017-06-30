#!/usr/bin/env python

from optparse import OptionParser
import numpy as np
from scipy import interpolate, signal
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
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
import dmds_self_tdoa
import helpers
import ransac_1d

#mpl
matplotlib.rcParams['text.usetex'] = True
matplotlib.rcParams['text.latex.unicode'] = True
matplotlib.rcParams['font.size'] = 15
matplotlib.rcParams['font.family'] = "serif"
matplotlib.rcParams['font.serif'] = "cm10"


c = 299700000.0

def parse_options():
    """ Options parser. """
    parser = OptionParser(usage="%prog: [options] results_file [samples_file]")
    parser.add_option("-g", "--geographical", action="store_true", default=True,
                      help="Geographical Map")
    parser.add_option("-m", "--map", action="store_true", default=False,
                      help="plot results of self localization on map")
    parser.add_option("", "--history", action="store_true", default=False,
                      help="plot delay and transmitter history for DMDS")
    parser.add_option("-r", "--replay", action="store_true", default=False,
                      help="replay whole experiment. Sample log required as second argument")
    parser.add_option("-c", "--config", type="str", default="",
                      help="Configuration file for self localization algorithm")
    parser.add_option("", "--crop-ict", action="store_true", default= False, 
                      help="hack to crop the default gui map to the region in front of the receiers")
    parser.add_option("-s", "--save", action="store_true", default=False,
                      help="Save plots to files")
    parser.add_option("", "--stress", action="store_true", default=False,
                      help="Plot stress function")
    parser.add_option("", "--check-anchoring", action="store_true", default=False,
                      help="debug")
    (options, args) = parser.parse_args()
    if len(args) == 0 or len(args) >2:
        parser.error('Invalid number of inputs!')
    return options, args

if __name__ == "__main__":
    options,args = parse_options()
    receivers_steps=[]
    # args[0] ^= selfloc log   
    f_results=open(args[0],"r")
    line=f_results.readline()
    line=f_results.readline()
    line=f_results.readline()
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
    receivers_offset = acquisition[14]
    selfloc_average_length = acquisition[15]
    num_anchors = acquisition[16]
    anchor_average = acquisition[17]
    ref_receiver = acquisition[18]
    alpha = acquisition[19]

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
    stress_list = eval(f_results.readline())

    ransac_tdoa = ransac_1d.ransac_1d(ransac_1d.ConstantLeastSquaresModel(), 0.3, 0.1, 1000, 5)

    delay_means = []
    for entry in anchor_loop_delay_history:
        delay_mean = []
        for i in range(len(receivers_positions)-1):
            delay_mean.append(ransac_tdoa.ransac_fit(np.array(entry)[:,i]))
        delay_means.append(delay_mean)

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
    x = x1 - x0
    y = y1 - y0
    f_results.close()
    
    #receivers_positions = []
    
    #hacked_position_list = [(6.06274836, 50.77902740),(6.06228259, 50.77913433),(6.06223227, 50.77883295),(6.06253711, 50.77878010)]
    #for pos in hacked_position_list:
    #    receivers_positions.append(basemap(pos[0],pos[1]))
    
    #receivers_positions = np.array(receivers_positions)
    #FIXME hack
    D_true = np.ndarray(shape = (len(receivers_positions),len(receivers_positions),len(receivers_positions)))
    #receivers_positions = [(131.2797962158893, 59.88438221876277), (121.4281368314987, 102.70505203219363),(153.97361311259738, 93.98137587687233), (115.85778131733423, 72.92402160679922)]
    receivers_positions = np.array(receivers_positions)
    for cnt_j in range(len(receivers_positions)):
        for cnt_l, rx_l in enumerate(receivers_positions):
            for cnt_k, rx_k in enumerate(receivers_positions):
                if cnt_j != cnt_l and cnt_j != cnt_k and cnt_l != cnt_k:
                    D_true[cnt_j, cnt_l, cnt_k] = (np.linalg.norm(receivers_positions[cnt_l]-receivers_positions[cnt_j])-np.linalg.norm(receivers_positions[cnt_k]-receivers_positions[cnt_j]))
                else:
                    D_true[cnt_j, cnt_l, cnt_k] = 0.0
    difference_D = D - D_true
    scale = math.ceil(math.sqrt(abs(x*y/0.3136)))
    if options.check_anchoring:
        d, pos_selfloc_procrustes, tform = procrustes(receivers_positions, pos_selfloc, scaling=False)
    
    if options.map:
        figure_map = plt.figure(figsize=(20,10))
        ax = figure_map.add_subplot(111, xlim=(x0,x1), ylim=(y0,y1), autoscale_on=False)
        figure_map.canvas.set_window_title(args[0].split("/")[-1].split(".")[0] + "_map")
        basemap.imshow(img, interpolation='lanczos', origin='upper')
        i = 1
        if options.crop_ict:
            ax.axis([85,230,48,140])
            basemap.drawmapscale(lon=6.06201, lat=50.77870, lon0=6.06201, lat0=50.77870, length=20,  units='m',barstyle='fancy',fontsize = 30, yoffset=1.2)
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
                ax.scatter(rx[0], rx[1],linewidths=2, marker='^', c='w', s=400, alpha=0.9, label = "Sensor ground-truth")
            else:
                ax.scatter(rx[0], rx[1],linewidths=2, marker='^', c='w', s=400, alpha=0.9)
            # set annotation RXx
            text = "Rx" + str(i)
            # index of logged reference receiver starts at 0 not at 1
            rx = (rx[0] + 5, rx[1] - 1)
            #if i != (ref_receiver+1): #and options.reference_selection == "Manual":
            ax.annotate(text, rx,fontweight='bold', fontsize = 24,bbox=dict(facecolor='w', alpha=0.9))
            i += 1
        i = 1
        for rx in pos_selfloc_procrustes:
            if i == 1:
                ax.scatter(rx[0], rx[1],linewidths=2, marker='^', c="r", s=400, alpha=0.9, label = "Result sensors, RMSE: "+ "${0:.3f}".format(rmse_positions)+"m$")
            else:
                ax.scatter(rx[0], rx[1],linewidths=2, marker='^', c="r", s=400, alpha=0.9)
            # set annotation RXx
            text = "Rx" + str(i)
            # index of logged reference receiver starts at 0 not at 1
            rx = (rx[0] + 5, rx[1] - 1)
            #if i != (ref_receiver+1): #and options.reference_selection == "Manual":
            ax.annotate(text, rx,fontweight='bold', fontsize = 24,bbox=dict(facecolor='r', alpha=0.9))
            i += 1
        i = 1
        for rx in anchor_gt_positions:
            if i == 1:
                ax.scatter(rx[0], rx[1],linewidths=2, marker='^', c="g", s=400, alpha=0.9, label = "Anchor ground-truth")
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
                ax.scatter(rx[0], rx[1],linewidths=2, marker='^', c="b", s=400, alpha=0.9, label = "Result anchors, RMSE: "+ "${0:.3f}".format(rmse_anchors)+"m$")
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
            plt.savefig(args[0].split("/")[-1].split(".")[0] + "_map.pdf", dpi=150)

    if options.history:
        figure_history = plt.figure()
        figure_history.canvas.set_window_title(args[0].split("/")[-1].split(".")[0] + "_DMDS-history")
        ax1 = figure_history.add_subplot(111)

        delay_12 = []
        delay_13 = []
        delay_14 = []
        delay_23 = []
        delay_24 = []
        delay_34 = []
        for cnt_j in range(len(delay_tensor)):
            delay_12.extend(delay_tensor[cnt_j,0,1,:].tolist())
            delay_13.extend(delay_tensor[cnt_j,0,2,:].tolist())
            delay_14.extend(delay_tensor[cnt_j,0,3,:].tolist())
            delay_23.extend(delay_tensor[cnt_j,1,2,:].tolist())
            delay_24.extend(delay_tensor[cnt_j,1,3,:].tolist())
            delay_34.extend(delay_tensor[cnt_j,2,3,:].tolist())

        plt_12 = ax1.plot(delay_12,label = r'$\tau_{12}$' )
        plt_13 = ax1.plot(delay_13,label = r'$\tau_{13}$' )
        plt_14 = ax1.plot(delay_14,label = r'$\tau_{14}$' )
        plt_23 = ax1.plot(delay_23,label = r'$\tau_{23}$' )
        plt_24 = ax1.plot(delay_24,label = r'$\tau_{24}$' )
        plt_34 = ax1.plot(delay_34,label = r'$\tau_{34}$' )
        ax1.set_xlabel(r'Acquisitions')
        ax1.set_ylabel(r'TDOA [ns]')
        ax1.legend()
        ax2 = ax1.twiny()
        # dummy plot
        ax2.plot(delay_12,visible=False)
        ax2.set_xlabel("Transmitter")
        ax2.set_xticklabels([])
        ax2.locator_params(nbins=4)
        ax2.xaxis.set_minor_locator(ticker.AutoMinorLocator(n=2))
        for tick in ax2.xaxis.get_minor_ticks():
            tick.tick2line.set_markersize(0)
        ax2.xaxis.set_minor_formatter(ticker.FixedFormatter(('Sensor 1','Sensor 2','Sensor 3','Sensor 4')))
        ax2.grid()
        if options.save:
            #p.figure_map.tight_layout()
            plt.savefig(args[0].split("/")[-1].split(".")[0] + "_history.pdf", dpi=150)

    if options.stress:
        figure_stress = plt.figure()
        figure_stress.canvas.set_window_title(args[0].split("/")[-1].split(".")[0] + "_stress")
        axis_stress = figure_stress.add_subplot(111)
        axis_stress.plot(stress_list)
        axis_stress.set_ylabel(r'Stress')
        axis_stress.set_xlabel(r'Iterations')
        axis_stress.grid()
        if options.save:
            #p.figure_map.tight_layout()
            plt.savefig(args[0].split("/")[-1].split(".")[0] + "_stress.pdf", dpi=150)

    plt.show()



    
    if options.replay:
        # first set of samples: delays for algorithm. length: average_length*num_sensors
        # second set: delays for anchoring. length: average_length* num_anchors
        # mabe build in check?
        if len(args) == 2:
            f_samples = open(args[1],"r")
            for line_number, line in enumerate(f_samples.readlines()):
                receivers_samples = eval(eval(line))
                receivers = dict()
                # FIXME
                if line_number < selfloc_average_length * len(receivers_positions):
                    for cnt_tx in range(1, len(receivers_positions) + 1):
                        if selfloc_average_length * (cnt_tx - 1) <= line_number < selfloc_average_length * cnt_tx:
                            receivers_samples.insert(cnt_tx - 1, [])
                for receiver_idx in range(len(receivers_positions)):
                    receivers[receiver_idx] = receiver_interface.receiver_interface(None,None,receiver_idx)
                    receivers[receiver_idx].coordinates = receivers_positions[receiver_idx]
                    receivers[receiver_idx].offset = receivers_offset[receiver_idx]
                    receivers[receiver_idx].serial = receiver_idx
                    receivers[receiver_idx].frequency = frequency
                    receivers[receiver_idx].interpolation = interpolation
                    receivers[receiver_idx].samp_rate = sampling_rate
                    receivers[receiver_idx].samples_to_receive = samples_to_receive
                    receivers[receiver_idx].samples = receivers_samples[receiver_idx]
                    receivers[receiver_idx].correlation_interpolation = True
                    receivers[receiver_idx].measurement_noise = 10
                    receivers[receiver_idx].selected_position = "selfloc"
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
                    for cnt_l, rx_l in enumerate(curr_rx_set.values()):
                        for cnt_k, rx_k in enumerate(curr_rx_set.values()):
                            if cnt_j != cnt_l and cnt_j != cnt_k and cnt_l != cnt_k:
                                window_size = 13
                                #by now ugly hack, rethink later
                                delay_tensor[cnt_j,cnt_l,cnt_k,cnt_average] = corr_spline_interpolation(curr_rx_set.values()[cnt_l].samples, curr_rx_set.values()[cnt_k].samples, window_size)[1] / sampling_rate * 10**9 + rx_l.offset - rx_k.offset
                                #print(correlate({receivers.keys()[cnt_l]:receivers.values()[cnt_k],receivers.keys()[cnt_l]:receivers.values()[cnt_k]})[1])
                            else:
                                delay_tensor[cnt_j,cnt_l,cnt_k,cnt_average] = 0.0
            D = np.ndarray(shape=(len(receivers_positions),len(receivers_positions),len(receivers_positions)))
            sum_square_tdoa = 0
            for j in range(len(receivers_positions)):
                for l in range(len(receivers_positions)):
                    for k in range(len(receivers_positions)):
                        # average distance differences
                        tdoa = ransac_tdoa.ransac_fit(delay_tensor[j,l,k])/ 10**9 * 299700000.0
                        #tdoa = np.mean(delay_tensor[j,l,k])/ 10**9 * 299700000.0
                        sum_square_tdoa += tdoa**2
                        D[j,l,k] = tdoa
            #sum_square_tdoa = 0
            #for j in range(len(receivers_positions)):
            #    for l in range(len(receivers_positions)):
            #        for k in range(len(receivers_positions)):
            #            sum_square_tdoa += D_true[j, l, k]**2

            anchor_loop_delays = []
            anchor_loop_delay_history = []
            delay_means = []
            for receivers in receivers_anchoring:
                #
                # pdb.set_trace()
                # del receivers[0]
                delay = []
                for receiver in receivers:
                    window_size = 13
                    if not ref_receiver == receiver:
                        delay.append(corr_spline_interpolation(receivers.values()[receiver].samples,
                                     receivers.values()[ref_receiver].samples, window_size)[1] / sampling_rate * 10**9 + \
                                     receivers[receiver].offset - receivers[ref_receiver].offset)
                anchor_loop_delays.append(delay)
                # no sensor is transmitting
                transmitter_history.append(-1)
                if len(anchor_loop_delays) == anchor_average:
                    #delay_mean = ransac_tdoa.ransac_fit(np.array(anchor_loop_delays))
                    # delay_mean = np.mean(np.array(anchor_loop_delays))
                    #delay_means.append(delay_mean)
                    anchor_loop_delay_history.append(anchor_loop_delays)
                    anchor_loop_delays = []
        pos_selfloc = None
        stress = [10]
        pos_selfloc, stress = dmds_self_tdoa.selfloc(D, basemap(bbox[2],bbox[3]), sum_square_tdoa, None, 5000, alpha, stress)

        # for chan algorithm:
        receivers_dummy = {}
        for receiver_idx in range(len(receivers_positions)):
            receivers_dummy[receiver_idx] = receiver_interface.receiver_interface(None, None, receiver_idx)
            receivers_dummy[receiver_idx].coordinates = receivers_positions[receiver_idx]
            receivers_dummy[receiver_idx].offset = receivers_offset[receiver_idx]
            receivers_dummy[receiver_idx].serial = receiver_idx
            receivers_dummy[receiver_idx].frequency = frequency
            receivers_dummy[receiver_idx].interpolation = interpolation
            receivers_dummy[receiver_idx].samp_rate = sampling_rate
            receivers_dummy[receiver_idx].samples_to_receive = samples_to_receive
            receivers_dummy[receiver_idx].samples = [0.0]
            receivers_dummy[receiver_idx].correlation_interpolation = True
            receivers_dummy[receiver_idx].measurement_noise = 10
            receivers_dummy[receiver_idx].selected_position = "selfloc"
            receivers_dummy[receiver_idx].coordinates_selfloc = pos_selfloc[receiver_idx]

        delay_means = []
        for entry in anchor_loop_delay_history:
            delay_mean = []
            for i in range(len(receivers_positions) - 1):
                delay_mean.append(ransac_tdoa.ransac_fit(np.array(entry)[:, i]))
            delay_means.append(delay_mean)
        for j, delay_mean in enumerate(delay_means):
            anchor_positions[j] = chan94_algorithm.localize(receivers_dummy, ref_receiver, np.round(basemap(bbox[2], bbox[3])), delay=delay_mean)["coordinates"]
        anchor_gt_positions = np.array(anchor_gt_positions)
        anchor_positions = np.array(anchor_positions)
        d, coordinates_procrustes, tform = procrustes(anchor_gt_positions, anchor_positions, scaling = False)
        print anchor_gt_positions
        print(coordinates_procrustes)
        reflection = np.linalg.det(tform["rotation"])
        pos_selfloc_procrustes =  np.dot(pos_selfloc,tform["rotation"]) + tform["translation"]


        
        header =  "["  + str(sampling_rate) + "," + str(frequency) + "," + str(frequency_calibration) + "," \
                    + str(calibration_position) + "," + str(interpolation) + "," \
                    + str(bandwidth) + "," + str(samples_to_receive) + "," + str(lo_offset) + "," \
                    + str(bbox) + "," + str(receivers_positions.tolist()) + "," + str(selected_positions.tolist()) + "," \
                    + str(receivers_gps) + "," + str(receivers_antenna) + "," + str(receivers_gain) + "," + str(receivers_offset) \
                    + "," + str(selfloc_average_length) + "," + str(num_anchors) + "," + str(anchor_average) + "," \
                    + str(receivers_dummy.keys().index(ref_receiver)) + "," + str(alpha) + "]\n"

        results_file_selfloc = "../log/results_post_selfloc_" + time.strftime("%d_%m_%y-%H_%M_%S") + ".txt"
        fi = open(results_file_selfloc,'w')
        fi.write("##########################################################################################################################################################################################\n")
        fi.write("rx_time,sampling_rate,frequency,frequency_calibration,calibration_position,interpolation,bandwidth,samples_to_receive,lo_offset,bbox,receivers_positions,selected_positions,receivers_gps,receivers_antenna,receivers_gain,sample_average,num_anchors,anchor_average,index_ref_receiver,alpha\n")
        fi.write("##########################################################################################################################################################################################\n")
        fi.write(header)
        fi.write(str(transmitter_history) + "\n")
        fi.write(str(timestamp_history) + "\n")
        fi.write(str(delay_tensor.tolist()) + "\n")
        fi.write(str(D.tolist()) + "\n")
        fi.write(str(anchor_loop_delay_history) + "\n")
        fi.write(str(anchor_positions.tolist()) + "\n")
        fi.write(str(coordinates_procrustes.tolist()) + "\n")
        fi.write(str(anchor_gt_positions.tolist()) + "\n")
        fi.write(str(pos_selfloc.tolist()) + "\n")
        fi.write(str(pos_selfloc_procrustes.tolist()) + "\n")
        #fi.write(str(tform.keys()) + "\n")
        #tform["rotation"] = tform["rotation"].tolist()
        #tform["translation"] = tform["translation"].tolist()
        #fi.write(str(tform.values()) + "\n")
        fi.write(str(stress) + "\n")
        fi.close()
        print "selfloc test results written to: \n" +results_file_selfloc


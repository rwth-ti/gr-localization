#!/usr/bin/env python

from optparse import OptionParser
import numpy as np
from scipy import interpolate, signal
from numpy import array
import sys,os
import math
import requests
from StringIO import StringIO
import time
import pdb
import pprint
import ConfigParser
from mpl_toolkits.basemap import Basemap
sys.path.append("../python")
import receiver_interface,chan94_algorithm,grid_based_algorithm
from  kalman import kalman_filter
from ConfigSectionMap import ConfigSectionMap
import dop 
from chan94_algorithm import estimate_delay_interpolated
from interpolation import corr_spline_interpolation

c = 299700000

def parse_options():
    """ Options parser. """
    parser = OptionParser(usage="%prog: [options] file")
    parser.add_option("-a", "--algorithm", type="str", default="chan",
                      help="Select algoritm")
    parser.add_option("-g", "--geographical", action="store_true", default=True,
                      help="Geographical Map")
    parser.add_option("", "--enable-kalman", action="store_true", default=False,
                      help="Do Kalman filtering. Will ignore real time filtered values.")
    parser.add_option("", "--reference-selection", type = "str", default="Manual",
                      help="reference selection algorithm. Implemented options are: Manual, Max-signal-power, Min-signal-power, Min-DOP ")
    parser.add_option("-c", "--config", type="str", default="",
                      help="Configuration File for Kalman Filter")
    (options, args) = parser.parse_args()
    if len(args)<2:   # 
        parser.error('Filename not given')
    return options,args

if __name__ == "__main__":
    options,args = parse_options()
    receivers_steps=[]
    print options.reference_selection
    
    f_results=open(args[0],"r")
    line=f_results.readline()
    line=f_results.readline()
    line=f_results.readline()
    line=f_results.readline()
    
    acquisition = eval(eval(line))
    timestamps = []
    ref_receivers = []
    xk_1 = []
    init_settings_kalman = {} 
    x_cov=0
    y_cov=0
    timestamps.append(acquisition[0])
    delays = acquisition[1]
    delays_calibration = acquisition[2]
    delays_auto_calibration = acquisition[3]
    sampling_rate = acquisition[4]
    frequency = acquisition[5]
    frequency_calibration = acquisition[6]
    calibration_position = acquisition[7]
    interpolation = 1#acquisition[8]
    bandwidth = acquisition[9]
    samples_to_receive = acquisition[10]
    lo_offset = acquisition[11]
    bbox = acquisition[12]
    receivers_positions = acquisition[13]
    #receivers_positions[1] = np.array(receivers_positions[1])#+array([-1.5,-2])
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
        init_settings_kalman["num_rx"] = len(receivers_positions) 
    except:
        pass
    idx = 1
    for line in f_results.readlines():
        acquisition = eval(eval(line))
        timestamps.append(acquisition[0])
        ref_receivers.append(acquisition[19])
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
    reference_selection = options.reference_selection
    
    f_results.close()
    f_samples = open(args[1],"r")
    delay = []
    
    for line_number, line in enumerate(f_samples.readlines()):  
        if line_number >= len(timestamps):
            break
            
        acquisition_samples = eval(eval(line))
        receivers_samples=acquisition_samples[0]
        receivers=dict()
        for receiver_idx in range(len(receivers_samples)):
            receivers[receiver_idx]=receiver_interface.receiver_interface(None,None,receiver_idx)
            receivers[receiver_idx].coordinates=receivers_positions[receiver_idx]
            receivers[receiver_idx].serial = receiver_idx
            receivers[receiver_idx].frequency = frequency
            receivers[receiver_idx].interpolation = interpolation
            receivers[receiver_idx].samp_rate = sampling_rate
            receivers[receiver_idx].samples_to_receive = samples_to_receive
            receivers[receiver_idx].samples = receivers_samples[receiver_idx] 
            receivers[receiver_idx].correlation_interpolation = True
        if interpolation != 1:
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
    
    results_file="../log/results_post_" + time.strftime("%d_%m_%y-%H_%M_%S") + ".txt"
    if any(receivers):
        f=open(results_file,"w")
        pprint.pprint("##########################################################################################################################################################################################", f)
        pprint.pprint("time,delays(1-2,1-3,1-X...),delays_calibration(1-2,1-3,1-X...),delays_auto_calibration(1-2,1-3,1-X...),sampling_rate,frequency,frequency_calibration,calibration_position,interpolation,bandwidth,samples_to_receive,lo_offset,bbox,receivers_positions,selected_positions,receivers_gps,receivers_antenna,receivers_gain,estimated_positions,index_ref_receiver,auto_calibrate,acquisition_time", f)
        pprint.pprint("##########################################################################################################################################################################################", f)
    else:
        sys.exit("log import failed")   
    if options.enable_kalman: 

        #Kalman filtering the returned data
        if not any(init_settings_kalman) or options.config != "":
            cfg=ConfigParser.ConfigParser()
            invalid=False
            cfg.read(options.config)
            init_settings_kalman=ConfigSectionMap(cfg,"sectionOne")
            #init_settings_kalman["noise_var_x"]=init_settings_kalman["noise_var_x"]/(sampling_rate*interpolation)
        elif not any(init_settings_kalman) and options.config == "":
            sys.exit("no valid configuration found")
            
            
            
        if options.algorithm == "chan" or options.algorithm == "both":
            init_settings_kalman['algorithm']='chan'
            kalman=kalman_filter(init_settings_kalman)
            estimated_positions["chan"]=chan94_algorithm.localize(receivers_steps[0],ref_receiver,np.round(basemap(bbox[2],bbox[3])))
            measurement=estimated_positions["chan"]["coordinates"]#+np.array([-20,-20])
            xk_1 = np.hstack((np.array(list(estimated_positions["chan"]["coordinates"])),np.zeros(kalman.get_state_size()-2))) #init state
            kalman_states = xk_1
            Pk_1=kalman.get_init_cov()
            
            for i in range(len(receivers_steps)):
                start_time=time.time()
                signal_strength=[]
                if options.reference_selection in ["Min-signal-power","Max-signal-power"]:
                    for idx in range(len(receivers_steps[i])):
                        signal_strength.append(np.sum(np.square(np.abs(receiver.samples))))
                        #pdb.set_trace()
                    if options.reference_selection == "Min-signal-power":
                        ref_receiver = np.argmin(signal_strength)
                    if options.reference_selection == "Max-signal-power":
                        ref_receiver = np.argmax(signal_strength)
                
                if not i==0:
                    estimated_positions["chan"]=chan94_algorithm.localize(receivers_steps[i],ref_receiver,np.round(basemap(bbox[2],bbox[3])),kalman.get_a_priori_est(xk_1)[:2])
                    measurement=kalman.pre_filter(estimated_positions["chan"]["coordinates"],xk_1)
                '''
                if dop_location < 1:
                    dop_location = 1
                if dop_location > 10:
                    dop_location = 10
                '''
                if options.reference_selection == "Min-DOP":
                    ref_receiver,dop_location,H = dop.reference_selection_dop(estimated_positions["chan"]["coordinates"], receivers_steps[i])
                else:
                    dop_location,H=dop.calc_dop(estimated_positions["chan"]["coordinates"], receivers_steps[i],ref_receiver)
                kalman.adapt_R(H)
                xk_1,Pk_1=kalman.kalman_fltr(np.array(list(measurement)),Pk_1,xk_1,"chan")
                x_cov = Pk_1[0,0]
                y_cov = Pk_1[1,1]
                if i>0:
                    kalman_states=np.vstack((kalman_states,xk_1))

                estimated_positions["chan"]["kalman_coordinates"]=xk_1[:2].tolist()
                elapsed_time=time.time()-start_time
                print elapsed_time
                delays = []
                '''
                for receiver in receivers_steps[i]:
                    if receiver != ref_receiver:
                        delays.append(float(estimate_delay_interpolated(receivers_steps[i][receiver].samples, receivers_steps[i][ref_receiver].samples)))

                delays = (np.array(delays)+np.true_divide(np.array(delays_calibration),10)).tolist()
                '''
                delay = []
                for idx in range(len(receivers_steps[i])):
                    if not ref_receiver == idx:
                        # For the spline interpolation of the cross correlation, a sufficient support is necessary
                        window_size = 13
                        if receivers_steps[i][idx].correlation_interpolation:
                            correlation_acquisition, delay_acquisition  = corr_spline_interpolation(receivers_steps[i][idx].samples, receivers_steps[i][ref_receiver].samples,window_size)
                            delay.append(delay_acquisition)
                line = "[" + str(timestamps[i]) + "," + str(delay) + "," + str(delays_calibration) + "," + str(delays_auto_calibration) + "," + str(sampling_rate) + "," + str(frequency) + "," + str(frequency_calibration) + "," + str(calibration_position) + "," + str(interpolation) + "," + str(bandwidth) + "," + str(samples_to_receive) + "," + str(lo_offset) + "," + str(bbox) + "," + str(receivers_positions) + "," + str(selected_positions) + "," + str(receivers_gps ) + "," + str(receivers_antenna) + "," + str(receivers_gain) + "," + str(estimated_positions) + "," + str(ref_receiver) + "," + str(auto_calibrate) + "," + str(acquisition_time) + ","+ str(xk_1.tolist()).replace('\n', '') + ","+ str(init_settings_kalman) +","+  "'" + str(reference_selection) +"'"+","+str(x_cov) + ","+str(y_cov) +"]"
                pprint.pprint(line,f,width=9000)

        if options.algorithm=="grid_based" or options.algorithm=="both":        
            init_settings_kalman['algorithm']='grid_based'
            kalman=kalman_filter(init_settings_kalman)
            estimated_positions["grid_based"]=grid_based_algorithm.localize(receivers_steps[0],np.round(basemap(bbox[2],bbox[3])),1,interpolation*samples_to_receive,ref_receiver)
            measurement = estimated_positions["grid_based"]["coordinates"]
            xk_1= np.hstack((np.array(list(estimated_positions["grid_based"]["coordinates"])),np.zeros(kalman.get_state_size()-2))) #init state
            kalman_states= xk_1
            Pk_1=kalman.get_init_cov()
            for i in range(len(receivers_steps)):
                estimated_positions["grid_based"]=grid_based_algorithm.localize(receivers_steps[i],np.round(basemap(bbox[2],bbox[3])),1,interpolation*samples_to_receive,ref_receiver)
                estimated_positions["grid_based"]["coordinates"]=kalman.pre_filter(estimated_positions["grid_based"]["coordinates"],xk_1)
                xk_1,Pk_1=kalman.kalman_fltr(np.array(list(estimated_positions["grid_based"]["coordinates"])),Pk_1,xk_1,"grid_based")
                if i>0:
                    kalman_states=np.vstack((kalman_states,xk_1))

                estimated_positions["grid_based"]["kalman_coordinates"]=xk_1[:2].tolist()
                estimated_positions["grid_based"]["grid"]=0
                line = "[" + str(timestamps[i]) + "," + str(delays) + "," + str(delays_calibration) + "," + str(delays_auto_calibration) + "," + str(sampling_rate) + "," + str(frequency) + "," + str(frequency_calibration) + "," + str(calibration_position) + "," + str(interpolation) + "," + str(bandwidth) + "," + str(samples_to_receive) + "," + str(lo_offset) + "," + str(bbox) + "," + str(receivers_positions) + "," + str(selected_positions) + "," + str(receivers_gps ) + "," + str(receivers_antenna) + "," + str(receivers_gain) + "," + str(estimated_positions) + "," + str(ref_receiver) + "," + str(auto_calibrate) + "," + str(acquisition_time) + ","+ str(xk_1.tolist()).replace('\n', '') + ","+ str(init_settings_kalman) + "'" + str(reference_selection) +"'"+","+str(x_cov) + ","+str(y_cov)  + "]"
                pprint.pprint(line,f,width=9000)
    else:
        for i in range(len(receivers_steps)):
            signal_strength=[]         
            if options.algorithm=="chan"or options.algorithm=="both":
                if options.reference_selection in ["Min-signal-power","Max-signal-power"]:
                    for idx in range(len(receivers_steps[i])):
                        signal_strength.append(np.sum(np.square(np.abs(receivers_steps[i][idx].samples))))
                        #pdb.set_trace()
                        if options.reference_selection == "Min-signal-power":
                            ref_receiver = np.argmin(signal_strength)
                        if options.reference_selection == "Max-signal-power":
                            ref_receiver = np.argmax(signal_strength)

                        
                if options.reference_selection == "Min-DOP":
                    ref_receiver,dop_location, H = dop.reference_selection_dop(estimated_positions["chan"]["coordinates"],receivers_steps[i])
                print ref_receiver
                delay = []
                for idx in range(len(receivers_steps[i])):
                    if not ref_receiver == idx:
                        # For the spline interpolation of the cross correlation, a sufficient support is necessary
                        window_size = 13
                        if receivers_steps[i][idx].correlation_interpolation:
                            correlation_acquisition, delay_acquisition  = corr_spline_interpolation(receivers_steps[i][idx].samples, receivers_steps[i][ref_receiver].samples,window_size)
                            delay.append(delay_acquisition)
                estimated_positions["chan"] = chan94_algorithm.localize(receivers_steps[i],ref_receiver,np.round(basemap(bbox[2],bbox[3])))
            elif options.algorithm=="grid_based" or options.algorithm=="both":
                estimated_positions["grid_based"]=grid_based_algorithm.localize(receivers_steps[i],np.round(basemap(bbox[2],bbox[3])),1,interpolation*samples_to_receive,ref_receiver)
                estimated_positions["grid_based"]["grid"]=0
            for idx in len(receivers_positions):
                receivers_positions[idx] = receivers_positions[idx].tolist() 
            line = "[" + str(timestamps[i]) + "," + str(delay) + "," + str(delays_calibration) + "," + str(delays_auto_calibration) + "," + str(sampling_rate) + "," + str(frequency) + "," + str(frequency_calibration) + "," + str(calibration_position) + "," + str(interpolation) + "," + str(bandwidth)+ "," + str(samples_to_receive) + "," + str(lo_offset) + "," + str(bbox) + "," + str(receivers_positions) + "," + str(selected_positions) + "," + str(receivers_gps )+ "," + str(receivers_antenna) + "," + str(receivers_gain) + "," + str(estimated_positions) + "," + str(ref_receiver) + "," + str(auto_calibrate)+ "," + str(acquisition_time) + "," + str(xk_1).replace('\n', '') + "," + str(init_settings_kalman) + "," + "'" + str(reference_selection) +"'"+","+str(x_cov) + ","+str(y_cov) + "]"
            pprint.pprint(line,f,width=9000)
    f.close()
       
    print "algorithm test results written to: \n" +results_file
    
    
    

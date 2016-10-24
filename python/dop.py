#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv
import pdb

def calc_dop(target_position,receivers,ref_receiver):
    "calculate dilution of precision for given szenario"
    target_position = np.array(list(target_position))
    if receivers[ref_receiver].selected_position == "manual":
        reference_position  = np.array(list(receivers[ref_receiver].coordinates))
    else:
        reference_position  = np.array(list(receivers[ref_receiver].coordinates_gps))
    distance_reference = target_position - reference_position
    H = np.ndarray(shape=[len(receivers)-1,len(target_position)])
    idx = 0
    for key in receivers.keys():
        #print "ref_receiver: " ,ref_receiver,"receiver: ",receivers[key]
        if key != ref_receiver:
            if receivers[key].selected_position == "manual":
                receiver_position  = np.array(list(receivers[key].coordinates))
            else:
                receiver_position  = np.array(list(receivers[key].coordinates_gps))

            distance_target = target_position - receiver_position
            tdoa_gradient = np.divide(((distance_target/np.linalg.norm(distance_target))-(distance_reference/np.linalg.norm(distance_reference))).T,0.3)#divide by c*10^s-9
            H[idx,:] = tdoa_gradient
            idx += 1
    DOP = np.sqrt(np.trace(inv(np.dot(H.T,H))))
    return DOP, H
    
def reference_selection_dop(target_position,receivers):
    """ select reference receiver with lowest dop""" 
    dop_references = []
    H_references = []
    for idx in range(len(receivers)): 
        dop,H = calc_dop(target_position,receivers,receivers.keys()[idx])
        dop_references.append(dop)
        H_references.append(H)
    reference_idx = np.argmin(dop_references)
    return receivers.keys()[reference_idx],np.amin(dop_references),H_references[np.argmin(dop_references)]
    
def get_min_dop(target_position,receivers):
    dop_references = []
    for idx in range(len(receivers)):
        dop,H = calc_dop(target_position,receivers,receivers.keys()[idx]) 
        dop_references.append(dop)
    return np.amin(dop_references)

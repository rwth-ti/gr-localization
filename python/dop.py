#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv
import pdb

def calc_dop(target_position,receivers,ref_receiver):
    "calculate dilution of precision for given szenario"
    target_position = np.array(list(target_position))
    reference_position  = np.array(list(receivers[ref_receiver].coordinates))
    distance_reference = target_position - reference_position
    #print target_position
    H = np.ndarray(shape=[len(target_position),len(target_position)])
    idx = 0
    for key in receivers.keys():
        #print "ref_receiver: " ,ref_receiver,"receiver: ",receivers[key]
        if key != ref_receiver:
            
            distance_target = target_position - np.array(receivers[key].coordinates)
            tdoa_gradient = ((distance_target/np.linalg.norm(distance_target))-(distance_reference/np.linalg.norm(distance_reference))).T
            H[idx,:] = tdoa_gradient
            idx += 1
    DOP = np.sqrt(np.trace(inv(np.dot(H.T,H)))**-1)
    return DOP
    
def reference_selection_dop(target_position,receivers):
    """ select reference receiver with lowest dop""" 
    dop_references = []
    for idx in range(len(receivers)): 
        dop_references.append(calc_dop(target_position,receivers,receivers.keys()[idx]))
    print dop_references
    reference_idx = np.argmin(dop_references)
    return receivers.keys()[reference_idx]

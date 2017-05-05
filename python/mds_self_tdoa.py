#!/usr/bin/env python

# TDOA localization algorithm inspired by Shepard-Kruskal Algorithmus
# this version with known beacons and sensors can not transmit

import numpy as np
import chan94_algorithm
from procrustes import procrustes
# interface:
# delta_jkl: tensor of delays tau: dict(1,2,3) of tdoas collected for beacon localization  coords_gt: 3x2 matrix
def selfloc(D, roi, sum_square_tdoa, prev_coordinates, max_it, alpha, stress_last):
    
    # pos_conf_init(1,:) = pos_sensors(1,:)
    # pos_conf_init(2,:) = pos_sensors(2,:)
    # prev_coordinates & max_it for debug purposes
    # calculate delta hat? 
    # find the configuration
    num_sensors = len(D[0,0,:].tolist())
    if prev_coordinates == None:
        pos_conf = np.max(roi) * np.random.uniform(low=0.0,high=1.0,size=(num_sensors,2))
    else: 
        pos_conf = prev_coordinates
    
    stress = stress_last[-1]
    sigma = 0
    fail_count = 0
    while True:
        fail_count += 1
        stress_ratio = 1.5
        it = 0 
        # check if converged
        while abs(stress_ratio) > 0.001 and stress > 2*sigma + 0.001 and it < max_it:
            # calculate configuration TDOAs and derive new locations
            pos_it_conf = pos_conf
            sum_tdoa_diff = 0
            for idx_node_j in range(num_sensors):
                for idx_node_l in range(num_sensors):
                    for idx_node_k in range(num_sensors):
                        if idx_node_l != idx_node_j and idx_node_l != idx_node_k and idx_node_j != idx_node_k:
                            # determine TDOA between target, node and
                            # another node for configuration and determine error
                            tdoa_conf = geometric_tdoa(pos_it_conf[idx_node_j], pos_it_conf[idx_node_k],pos_it_conf[idx_node_l])
                            tdoa_diff = D[idx_node_j, idx_node_k, idx_node_l] - tdoa_conf
                            # for STRESS
                            sum_tdoa_diff = sum_tdoa_diff + tdoa_diff**2
                            # move in direction of k
                            mov_vec = pos_it_conf[idx_node_j] - pos_it_conf[idx_node_k]
                            # normalize
                            mov_vec = np.true_divide(mov_vec, np.linalg.norm(mov_vec))
                            pos_conf[idx_node_j] = pos_conf[idx_node_j] + np.dot(alpha/((num_sensors-1)*(num_sensors-2))*tdoa_diff,mov_vec)
                            #pdb.set_trace()
            it += 1
            # calculate STRESS criterion
            if max_it != 1:
                stress_last.append(stress)
            stress = np.sqrt(sum_tdoa_diff/sum_square_tdoa)
            stress_ratio = stress_last[-1] - stress
            # center points
            center_conf = np.mean(pos_conf, axis = 0)
            pos_conf = pos_conf - center_conf
            print "stress", stress
            print "stress ratio", stress_ratio
           
        if stress < (2*sigma + 1.5) or (max_it == 1 and abs(stress_ratio) > 0.001) or stress > 20:
            break
        else:
            pos_conf = np.max(roi) * np.random.uniform(low=0.0,high=1.0,size=(num_sensors,2))
            # fix anchors
    #             pos_conf = bsxfun(@minus,pos_conf,pos_conf(1,:)-pos_sensors(1,:))
        #     pos_conf(1,:) = pos_sensors(1,:)
        #     pos_conf(2,:) = pos_sensors(2,:)
    stress_last.append(stress)
    return pos_conf, stress_last

#receivers list will need to be passed later
def anchor(pos_sensors, pos_beacons, D_beacons):
    num_sensors = len(pos_sensors)
    for idx_tag in range(len(pos_beacons)):
        for idx_node in range(1,num_sensors):
            tdoas_beacon[idx_node-1] = geometric_tdoa(pos_sensors[1],pos_sensors[idx_node],pos_beacons[idx_tag]) + sigma*randn
        Q_tdoas = 0.5*(np.ones(num_sensors-1)+np.identity(num_sensors-1))
        Q_sensors = 0.5*(np.ones(2*num_sensors)+np.identity(2*num_sensors))
        '''
        pos_beacons_chan_true[idx_tag] = TDOALoc(pos_sensors.T,tdoas_beacon.T,Q_tdoas)
        pos_beacons_chan_false[idx_tag] = TDOALoc(pos_conf.T,tdoas_beacon.T,Q_tdoas)
        '''
    # determine the procrustes transform based on the beacons ground truth
    a,pos_beacons_proc, T= procrustes(pos_beacons,pos_beacons_chan_false)
    reflection = np.linalg.det(T.T)

    # transform sensor configuration accordingly
    pos_conf_proc = pos_conf*T.T + T.c[1]

def geometric_tdoa(a,b,c):
    d_ab = np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)
    d_ac = np.sqrt((a[0]-c[0])**2+(a[1]-c[1])**2)
    tdoa = d_ab - d_ac
    return tdoa

if __name__ == "__main__":
    pos_sensors = np.array([[120,20],[30,80],[200,80],[120,150]])
    pos_beacons = np.array([[120,90],[120,70],[95,80]])
    sum_tdoa = 0
    D = np.ndarray(shape=(len(pos_sensors),len(pos_sensors),len(pos_sensors)))
    for idx_node_l in range(len(pos_sensors)):
        for idx_node_j in range(len(pos_sensors)):
            for idx_node_k in range(len(pos_sensors)):
                if idx_node_l != idx_node_j and idx_node_l != idx_node_k and idx_node_j != idx_node_k:
                    # determine TDOA between target, node and another node,
                    # add error
                    tdoa = geometric_tdoa(pos_sensors[idx_node_l], pos_sensors[idx_node_k],pos_sensors[idx_node_j])
                    # for STRESS
                    sum_tdoa = sum_tdoa + tdoa**2
                    # store tensor with tdoa information
                    D[idx_node_l,idx_node_k,idx_node_j] = tdoa
    pos_curr = None
    stress = 10
    #for idx in range(30):
    pos_curr, stress = selfloc(D, pos_sensors[2:], sum_tdoa, pos_curr, 500, 1.0, stress)
    print procrustes(pos_sensors, pos_curr)[1]
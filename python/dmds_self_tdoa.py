#!/usr/bin/env python

# TDOA localization algorithm inspired by Shepard-Kruskal Algorithmus
# this version with known beacons and sensors can not transmit

import numpy as np
import pdb
# interface:
# delta_jkl: tensor of delays tau: dict(1,2,3) of tdoas collected for beacon localization  coords_gt: 3x2 matrix
def selfloc(D, roi, sum_square_tdoa, prev_coordinates, max_it, sigma = 0.1):
    
    # pos_conf_init(1,:) = pos_sensors(1,:)
    # pos_conf_init(2,:) = pos_sensors(2,:)
    # prev_coordinates & max_it for debug purposes
    # calculate delta hat? 
    # find the configuration
    num_sensors = len(D[0,0,:].tolist())
    if prev_coordinates == None:
        pos_conf = np.max(roi) * np.random.uniform(low=0.0,high=1.0,size=(num_sensors,2))
        #pos_conf = np.array([(0, 0), (200, 0), (0, 200), (200, 200)])
    else: 
        pos_conf = prev_coordinates
    it = 0
    alpha = 0.3
    stress_threshold = sigma + 0.001
    stress_diff_threshold = 0.0001
    pos_it_conf = pos_conf
    best_result = pos_conf
    stress = 10
    stress_min = 10
    stress_last = []
    while stress > stress_threshold and it < max_it:
        stress_diff = 1.5
        # check if converged
        while abs(stress_diff) > stress_diff_threshold and it < max_it and stress < 20:
            # calculate configuration TDOAs and derive new locations
            pos_it_conf = pos_conf.copy()
            sum_tdoa_diff = 0
            for idx_node_j in range(num_sensors):
                for idx_node_l in range(num_sensors):
                    for idx_node_k in range(num_sensors):
                        if idx_node_l != idx_node_j and idx_node_l != idx_node_k and idx_node_j != idx_node_k:
                            # determine TDOA between target, node and
                            # another node for configuration and determine error
                            #pdb.set_trace()
                            tdoa_conf = geometric_tdoa(pos_it_conf[idx_node_k], pos_it_conf[idx_node_j], pos_it_conf[idx_node_l])
                            tdoa_diff = D[idx_node_k, idx_node_j, idx_node_l] - tdoa_conf
                            # for STRESS
                            sum_tdoa_diff = sum_tdoa_diff + tdoa_diff**2
                            # move in direction of k
                            mov_vec = pos_it_conf[idx_node_j] - pos_it_conf[idx_node_k]
                            # normalize
                            mov_vec = np.true_divide(mov_vec, np.linalg.norm(mov_vec))
                            pos_conf[idx_node_j] = pos_conf[idx_node_j] + np.dot((1.0*alpha)/((num_sensors-1)*(num_sensors-2)) * tdoa_diff, mov_vec)
            it += 1
            # calculate STRESS criterion
            if max_it != 1:
                stress_last.append(stress)
            stress = np.sqrt(sum_tdoa_diff/sum_square_tdoa)
            stress_diff = stress_last[-1] - stress
            # center points
            center_conf = np.mean(pos_conf, axis = 0)
            pos_conf = pos_conf - center_conf
            print "stress", stress
            print "stress ratio", stress_diff
            print it
            if stress < stress_min:
                stress_min = stress
                best_result = pos_conf
        if stress >= 20 or stress > stress_threshold:
            pos_conf = np.max(roi) * np.random.uniform(low=0.0, high=1.0, size=(num_sensors, 2))
            sum_tdoa_diff = 0
            for idx_node_j in range(num_sensors):
                for idx_node_l in range(num_sensors):
                    for idx_node_k in range(num_sensors):
                        if idx_node_l != idx_node_j and idx_node_l != idx_node_k and idx_node_j != idx_node_k:
                            # determine TDOA between target, node and
                            # another node for configuration and determine error
                            # pdb.set_trace()
                            tdoa_conf = geometric_tdoa(pos_it_conf[idx_node_k], pos_it_conf[idx_node_j],
                                                       pos_it_conf[idx_node_l])
                            tdoa_diff = D[idx_node_k, idx_node_j, idx_node_l] - tdoa_conf
                            # for STRESS
                            sum_tdoa_diff = sum_tdoa_diff + tdoa_diff ** 2
            stress = np.sqrt(sum_tdoa_diff / sum_square_tdoa)
    stress_last.append(stress)
    print stress_min
    return best_result, stress_last


def geometric_tdoa(a,b,c):
    d_ab = np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)
    d_ac = np.sqrt((a[0]-c[0])**2+(a[1]-c[1])**2)
    tdoa = d_ab - d_ac
    return tdoa
    

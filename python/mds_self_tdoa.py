# TDOA localization algorithm inspired by Shepard-Kruskal Algorithmus
# this version with known beacons and sensors can not transmit

import numpy as np
import chan94_algorithm



# interface:
# delta_jkl: tensor of delays; tau: dict(1,2,3) of tdoas collected for beacon localization ; coords_gt: 3x2 matrix
def selfloc(D, roi, alpha):
    
    # pos_conf_init(1,:) = pos_sensors(1,:)
    # pos_conf_init(2,:) = pos_sensors(2,:)

    # calculate delta hat? 
    # find the configuration
    num_sensors = len(D[0,0,:].tolist())
    pos_conf = max(roi) * np.random.uniform(low=0.0,high=1.0,size=(num_sensors,2))
    stress = 1
    sigma = 0
    while stress > (2*sigma + 0.001):
        stress_ratio = 1.5
        # check if converged
        while abs(1-stress_ratio) > 0.01 and stress > 2*sigma + 0.001:
            stress_last = stress
            # calculate configuration TDOAs and derive new locations
            pos_it_conf = pos_conf
            sum_tdoa_diff = 0
            for idx_node_i in range(num_sensors):
                for idx_node_j in range(num_sensors):
                    for idx_node_k in range(num_sensors):
                        if idx_node_i != idx_node_j and idx_node_i != idx_node_k and idx_node_j != idx_node_k:
                            # determine TDOA between target, node and
                            # another node for configuration and determine error
                            tdoa_conf = geometric_tdoa(pos_it_conf[idx_node_k],pos_it_conf[idx_node_i],pos_it_conf[idx_node_j])
                            tdoa_diff = D[idx_node_i,idx_node_j,idx_node_k] - tdoa_conf
                            # for STRESS
                            sum_tdoa_diff = sum_tdoa_diff + tdoa_diff**2
                            # move in direction of k
                            mov_vec = pos_it_conf[idx_node_i] - pos_it_conf[idx_node_k]
                            # normalize
                            mov_vec = np.true_divide(mov_vec, np.linalg.norm(mov_vec))
                            pos_conf[idx_node_i] = pos_conf[idx_node_i] + np.dot(alpha*1/((num_sensors-1)*(num_sensors-2))*tdoa_diff,mov_vec) 
            # calculate STRESS criterion
            stress = np.sqrt(sum_tdoa_diff)/((num_sensors-1)*(num_sensors-2))
            stress_ratio = stress_last/stress
            # center points
            center_conf = np.mean(pos_conf)
            pos_conf = pos_conf - center_conf
            # fix anchors
    #             pos_conf = bsxfun(@minus,pos_conf,pos_conf(1,:)-pos_sensors(1,:))
        #     pos_conf(1,:) = pos_sensors(1,:)
        #     pos_conf(2,:) = pos_sensors(2,:)
    return pos_conf

'''
def anchor(pos_sensors, pos_beacons, D_beacons):
    num_sensors = len(pos_sensors)
    for idx_tag = range(len(pos_beacons)):
        for idx_node = 2:num_sensors:
            tdoas_beacon(idx_node-1) = geometric_tdoa(pos_beacons(idx_tag,:),pos_sensors(1,:),pos_sensors(idx_node,:)) + sigma*randn
        Q_tdoas = 0.5*(ones(num_sensors-1)+eye(num_sensors-1))
        Q_sensors = 0.5*(ones(2*num_sensors)+eye(2*num_sensors))
        pos_beacons_chan_true(idx_tag,:) = TDOALoc(pos_sensors.',tdoas_beacon.',Q_tdoas)
        pos_beacons_chan_false(idx_tag,:) = TDOALoc(pos_conf.',tdoas_beacon.',Q_tdoas)
    end
    
    # determine the procrustes transform based on the beacons ground truth
    [~,pos_beacons_proc, T] = procrustes(pos_beacons,pos_beacons_chan_false,'Scaling',false)
    reflection = det(T.T)

    # transform sensor configuration accordingly
    pos_conf_proc = bsxfun(@plus,pos_conf*T.T,T.c(1,:))
'''
def geometric_tdoa(a,b,c):
    d_ab = np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)
    d_ac = np.sqrt((a[0]-c[0])**2+(a[1]-c[1])**2)
    tdoa = d_ab - d_ac
    return tdoa
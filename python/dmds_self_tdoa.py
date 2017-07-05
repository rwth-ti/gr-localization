#!/usr/bin/env python

# TDOA localization algorithm inspired by Shepard-Kruskal Algorithmus
# this version with known beacons and sensors can not transmit

import numpy as np
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
    stress_diff_threshold = 0.000001
    pos_it_conf = pos_conf
    best_result = pos_conf
    stress = 10
    stress_min = 10
    stress_last = []
    while stress > stress_threshold and it < max_it:
        pos_conf = np.max(roi) * np.random.uniform(low=0.0, high=1.0, size=(num_sensors, 2))
        # calculate initial STRESS
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
            stress_last.append(stress)
            # calculate new STRESS
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
            it += 1
    stress_last.append(stress)
    print stress_min
    return best_result, stress_last

def dmds_grad_descend_anchoring(D, tdoas, roi, pos_anchors, sum_square_tdoa, ref_receiver, max_it, sigma = 11):
    D_anchor = tdoa_mat_to_tensor(tdoas.copy(), ref_receiver)
    sensor_pos = np.array([[131.27979622,  59.88438222],
                           [121.42813683, 102.70505203],
                           [153.97361311,  93.98137588],
                           [115.85778132,  72.92402161]])
    x_b = pos_anchors[:, 0]
    y_b = pos_anchors[:, 1]
    D_anchor_true = ddoa_matrix_beacons(sensor_pos[:,0], sensor_pos[:,1], x_b, y_b)
    D_anchor_diff = D_anchor - D_anchor_true

    n = len(D[0, 0, :].tolist())
    m = len(D_anchor[0, 0, :].tolist())
    do_sum = sum_square_tdoa
    dob_sum = np.sum(np.sum(np.sum(np.square(D))))
    it = 0
    alpha = 0.1
    beta = 0.9
    stress_list = []
    stress = 100
    threshold_stress_diff = 0.0001
    while it < max_it:
        pos_conf = np.max(roi) * np.random.uniform(low=0.0,high=1.0,size=(n,2))
        xo = pos_conf[:, 0]
        yo = pos_conf[:, 1] # Candidate
        while it < max_it:
            D_est = ddoa_matrix(xo, yo)
            D_anchor_est = ddoa_matrix_beacons(xo, yo, x_b, y_b)
            stress_last = stress
            stress = np.sqrt((1 - beta) * np.sum(np.sum(np.sum(np.square(D - D_est)))) + beta * np.sum(np.sum(np.sum(np.square(D_anchor - D_anchor_est)))))
            stress_list.append(stress)
            stress_diff = -stress + stress_last
            grad = stress_gradient_anchoring(xo,yo,x_b,y_b,D,D_anchor,beta)
            #pdb.set_trace()
            xo = xo - alpha * 1.0 / ((n - 1) * (n - 2)) * grad[:, 0]
            yo = yo - alpha * 1.0 / ((n - 1) * (n - 2)) * grad[:, 1]
            # check stopping criteria
            if abs(stress_diff) < threshold_stress_diff and stress > sigma:
                break
            it += 1
            print "stress: ", stress
            print "stress_diff: ", stress_diff
            if stress < sigma:
                beta = 0.4
        # if solution is good stop iterating
    print it
    return np.array([xo,yo]).T, stress_list


def tdoa_mat_to_tensor(tdoas, ref_receiver):
    #kind of ugly
    #TODO check if ns or distances in robertos algorithm!
    tdoa_tensor = np.ndarray(shape =(len(tdoas), len(tdoas[:,1])+1,len(tdoas[:,1])+1))
    conversion_factor = 0.2997
    tdoas = np.insert(tdoas,ref_receiver,0,axis=1)
    for cnt_j in range(len(tdoas)):
        for cnt_l in range(len(tdoas[:, 0])+1):
            for cnt_k in range(len(tdoas[:, 0])+1):
                if cnt_l == cnt_k:
                    tdoa_tensor[cnt_j, cnt_l, cnt_k] = 0.0
                else:
                    tdoa_tensor[cnt_j, cnt_l, cnt_k] = (tdoas[cnt_j, cnt_l] - tdoas[cnt_j, cnt_k]) * conversion_factor
    return tdoa_tensor

def ddoa_matrix_beacons(xo, yo, x_b, y_b):
    receivers_positions = np.array([xo, yo]).T
    anchor_positions = np.array([x_b, y_b]).T
    D_true = np.ndarray(shape=(len(anchor_positions), len(receivers_positions), len(receivers_positions)))
    for cnt_j in range(len(anchor_positions)):
        for cnt_l, rx_l in enumerate(receivers_positions):
            for cnt_k, rx_k in enumerate(receivers_positions):
                if cnt_l != cnt_k:
                    D_true[cnt_j, cnt_l, cnt_k] = (
                    np.linalg.norm(receivers_positions[cnt_l] - anchor_positions[cnt_j]) - np.linalg.norm(
                        receivers_positions[cnt_k] - anchor_positions[cnt_j]))
                else:
                    D_true[cnt_j, cnt_l, cnt_k] = 0.0

    return D_true

def ddoa_matrix(xo,yo):
    receivers_positions = np.array([xo,yo]).T
    D_true = np.ndarray(shape=(len(receivers_positions), len(receivers_positions), len(receivers_positions)))
    for cnt_j in range(len(receivers_positions)):
        for cnt_l, rx_l in enumerate(receivers_positions):
            for cnt_k, rx_k in enumerate(receivers_positions):
                if cnt_j != cnt_l and cnt_j != cnt_k and cnt_l != cnt_k:
                    D_true[cnt_j, cnt_l, cnt_k] = (np.linalg.norm(receivers_positions[cnt_l]-receivers_positions[cnt_j]) - np.linalg.norm(receivers_positions[cnt_k]-receivers_positions[cnt_j]))
                else:
                    D_true[cnt_j, cnt_l, cnt_k] = 0.0
    return D_true


def stress_gradient_anchoring(x, y, x_b, y_b, do, dbo, beta):
    h = 0.001
    n = len(x)
    grad = np.ndarray(shape=(n, 2))
    for i in range(n):
        component = np.zeros(n)
        component[i] = 1
        x_1 = x - component * h
        d_1 = ddoa_matrix(x_1, y)
        d_b = ddoa_matrix_beacons(x_1, y, x_b, y_b)
        S_1 = np.sum(np.sum(np.sum(np.square(do - d_1))))
        S_b = np.sum(np.sum(np.sum(np.square(dbo - d_b))))
        S_ = (1 - beta) * S_1 + beta * S_b
        x1 = x + component * h
        d1 = ddoa_matrix(x1, y)
        db = ddoa_matrix_beacons(x1, y, x_b, y_b)
        S1 = np.sum(np.sum(np.sum(np.square(do - d1))))
        Sb = np.sum(np.sum(np.sum(np.square(dbo - db))))
        S = (1 - beta) * S1 + beta * Sb
        grad[i, 0] = (S - S_) / (2 * h)

        y_1 = y - component * h
        d_1 = ddoa_matrix(x, y_1)
        d_b = ddoa_matrix_beacons(x, y_1, x_b, y_b)
        S_1 = np.sum(np.sum(np.sum(np.square(do - d_1))))
        Sb = np.sum(np.sum(np.sum(np.square(dbo - d_b))))
        S_ = (1 - beta) * S_1 + beta * S_b
        y1 = y + component * h
        d1 = ddoa_matrix(x, y1)
        db = ddoa_matrix_beacons(x, y1, x_b, y_b)
        S1 = np.sum(np.sum(np.sum(np.square(do - d1))))
        Sb = np.sum(np.sum(np.sum(np.square(dbo - db))))
        S = (1 - beta) * S1 + beta * Sb
        grad[i, 1] = (S - S_) / (2 * h)
    return grad

def geometric_tdoa(a,b,c):
    d_ab = np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)
    d_ac = np.sqrt((a[0]-c[0])**2+(a[1]-c[1])**2)
    tdoa = d_ab - d_ac
    return tdoa

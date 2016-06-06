import numpy as np
import time


def estimate_delay(y1, y2):
    correlation = np.absolute(np.correlate(y1, y2, "full", False)).tolist()
    delay = (np.argmax(correlation) - len(y1) + 1).tolist()
    return delay


def localize(receivers, ref_receiver, bbox):

    t = time.time()
    sample_rate = receivers.values()[0].samp_rate * receivers.values()[0].interpolation
    y = []
    pos_rx = []
    for key in receivers:
        receiver = receivers[key]
        if key == ref_receiver:
            y.insert(0, receiver.samples)
        else:
            y.append(receiver.samples)
        if receiver.selected_position == "manual":
            if key == ref_receiver:
                pos_rx.insert(0, receiver.coordinates)
            else:
                pos_rx.append(receiver.coordinates)
        else:
            if key == ref_receiver:
                pos_rx.insert(0, receiver.coordinates_gps)
            else:
                pos_rx.append(receiver.coordinates_gps)

    c = 299700000

    #cross correlations
    d = []
    for receiver in receivers:
        if receiver != ref_receiver:
            d.append(float(estimate_delay(receivers[receiver].samples, receivers[ref_receiver].samples))/sample_rate)
    d12 = d[0]
    d13 = d[1]

    # Set receivers position.
    x1 = pos_rx[0][0]
    y1 = pos_rx[0][1]
    x2 = pos_rx[1][0]
    y2 = pos_rx[1][1]
    x3 = pos_rx[2][0]
    y3 = pos_rx[2][1]

    try:
        # Estimated distance difference from receivers to transmiter
        r21 = c * d12
        r31 = c * d13

        # Parameters of equation (5) from the reference paper
        K1 = pow(x1,2) + pow(y1,2)
        K2 = pow(x2,2) + pow(y2,2)
        K3 = pow(x3,2) + pow(y3,2)

        # Define position differences[0]
        x21 = x2 - x1
        x31 = x3 - x1
        y21 = y2 - y1
        y31 = y3 - y1
        rmax21= np.sqrt(x21**2+y21**2)
        rmax31= np.sqrt(x31**2+y31**2)
        #If a TDOA value that leads to distances greater than the real distance between 2 receivers occurs, clip it to maximum value.
        if r21>rmax21:
            r21=rmax21
            print "Warning: invalid TDOA between r2 & r1. Set to greatest value possible.
        if r31>rmax31:
            r31=rmax31    
            print "Warning: invalid TDOA between r3 & r1. Set to greatest value possible.
        # Solve equations system
        A = (y21*r31-y31*r21)/(y31*x21-y21*x31)
        B = (y21*(0.5*(pow(r31,2)-K3+K1))-y31*(0.5*(pow(r21,2)-K2+K1)))/(y31*x21-y21*x31)
        C = (x31*r21-x21*r31)/(y31*x21-y21*x31)
        D = (x31*(0.5*(pow(r21,2)-K2+K1))-x21*(0.5*(pow(r31,2)-K3+K1)))/(y31*x21-y21*x31)

        alpha = pow(A,2)+pow(C,2)-1
        beta = 2*(A*B+C*D-A*x1-C*y1)
        gamma = pow(x1,2)+pow(y1,2)-2*x1*B-2*y1*D+pow(B,2)+pow(D,2)

        r1 = [(-beta+np.sqrt(pow(beta,2)-4*alpha*gamma+0j))/(2*alpha),(-beta-np.sqrt(pow(beta,2)-4*alpha*gamma+0j))/(2*alpha)]
        valid_roots = []
        for root in r1:
            if root > 0:
                valid_roots.append(root)
        valid_roots = np.array(valid_roots)
        xy = np.real((valid_roots*A+B,valid_roots*C+D)).T
        # Calculate position with the solution in the roi
        if len(xy) > 1:
            if np.linalg.norm(xy[0]-np.array(bbox)/2) < np.linalg.norm(xy[1]-np.array(bbox)/2):
                print "1"
                xy = xy[0]#changed:different solution gets chosen(closer to the center!)
            else:
                print "2"
                xy = xy[1]
        else:
            print "3"
            xy = xy[0]
        xy = (xy[0],xy[1])
    except:
        print "Singular matrix, setting location to center of roi."
        xy = (bbox[0]/2,bbox[1]/2)
    t_used = time.time()-t
    print "Chan results: ",xy," time: ", t_used
    return {"coordinates": xy,"t_used":t_used}

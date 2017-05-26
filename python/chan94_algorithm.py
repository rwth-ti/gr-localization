import numpy as np
import time
from interpolation import *

c = 299700000.0

def chan_3rx(pos_rx, d, xk_prio):
    t = time.time()
    d12 = d[0]
    d13 = d[1]
    #print "d21:"+str(d12)+"d31:"+str(d13)

    # Set receivers position.
    x1 = pos_rx[0][0]
    y1 = pos_rx[0][1]
    x2 = pos_rx[1][0]
    y2 = pos_rx[1][1]
    x3 = pos_rx[2][0]
    y3 = pos_rx[2][1]
    
    center_triangle = ((x1+x2+x3)/3,(y1+y2+y3)/3)
    r21 = c * d12
    r31 = c * d13
    #print "r21:"+str(r21)+"r31:"+str(r31)
    try:
        # Estimated distance difference from receivers to transmiter
        
        # Parameters of equation (5) from the reference paper
        K1 = pow(x1,2) + pow(y1,2)
        K2 = pow(x2,2) + pow(y2,2)
        K3 = pow(x3,2) + pow(y3,2)

        # Define position differences[0]
        x21 = x2 - x1
        x31 = x3 - x1
        y21 = y2 - y1
        y31 = y3 - y1
        #print "r21:"+str(r21)+"r31:"+str(r31)
        rmax21= np.sqrt(x21**2+y21**2)
        rmax31= np.sqrt(x31**2+y31**2)
        #If a TDOA value that leads to distances greater than the real distance between 2 receivers occurs, clip it to maximum value.
        if r21>rmax21:
            r21=rmax21
            print "Warning: invalid TDOA between r2 & r1. Set to greatest value possible."
        if r21<-rmax21:
            r21=-(rmax21)
            print "Warning: invalid TDOA between r2 & r1. Set to greatest value possible."
        if r31>rmax31:
            r31=rmax31 
            print "Warning: invalid TDOA between r3 & r1. Set to greatest value possible."
        if r31<-rmax31:
            r31=-(rmax31 ) 
            print "Warning: invalid TDOA between r3 & r1. Set to greatest value possible."
            
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
            if  xk_prio.any():
                if np.linalg.norm(xy[0]-xk_prio) < np.linalg.norm(xy[1]-xk_prio):
                    print "1" 
                    xy = xy[0]#changed:different solution gets chosen(closer to prediction!)
                else:
                    print "2"
                    xy = xy[1]
            else:
                if np.linalg.norm(xy[0]-center_triangle) < np.linalg.norm(xy[1]-center_triangle):
                    print "1"
                    xy = xy[0]#changed:different solution gets chosen(closer to prediction!)
                else:
                    print "2"
                    xy = xy[1]
        else:
            print "3"
            xy = xy[0]
        xy = (xy[0],xy[1])
    except:
        print "Singular matrix, setting location to center of receivers."
        xy = center_triangle
    t_used = time.time()-t
    print "Chan results: ",xy," time: ", t_used
    return {"coordinates": xy,"t_used":t_used}

# Solution for 4 or more receivers with source in near field
def chan_tdoa(pos, d, Q):
    pos = np.array(pos)
    d= np.array(d)
    t = time.time()
    M,D = pos.shape # number of receiver stations M and dimensions D
    K = np.zeros((M,1))

    for i in range(M):
        K[i] = pos[i,0]**2 + pos[i,1]**2
    r_i1 = np.zeros((M-1,1)) # range difference in reference to receiver 1
    x_i1 = np.zeros((M-1,1))
    y_i1 = np.zeros((M-1,1))
    for i in range(M-1):
        r_i1[i] = c * d[i] 
        x_i1[i] = pos[i+1,0] - pos[0,0]
        y_i1[i] = pos[i+1,1] - pos[0,1]
        
        r_i1_max = np.sqrt(x_i1[i]**2+y_i1[i]**2)
        if r_i1[i] > r_i1_max:
            r_i1[i] = r_i1_max
        elif r_i1[i] < -r_i1_max:
            r_i1[i] = - r_i1_max
    
    
    #try:
    # ############################ First step (14b) ############################
    # For near-field source use (14b) first to give an approximation of B
    h = 0.5*(r_i1**2 - K[1:] + K[0]) # (11)
    G = -np.concatenate((x_i1, y_i1, r_i1),1) # (11)
    z1 = chan14b(h, G, Q) # first position estimate
    #print 'z1:', z1t = time.time()
    # ########################## Second step (14a) ############################
    cov_z, z2 = chan14a(h, G, Q, pos, z1)
    #print 'z2:', z2
    # ########################### Third step (22a) ############################
    z3 = chan22a(cov_z, pos, z2)
    #print 'z3:', z3try:
#    except:
#        print 'Chan TDoA Error'
#        z1 = np.reshape([0]*3, (D+1,1))
#        z3 = np.reshape([0]*2, (D,1))
    xy = (z3[0].item(0,0),z3[1].item(0,0))
    t_used = time.time()-t
    print "Chan results: ",xy," time: ", t_used
    return {"coordinates": xy,"t_used":t_used}


def chan14b(h, G, Q):
    '''
    Be adviced!: For use of chan14b() the number of TDoA-receivers should be at
                 least D+2, with D the number of t = time.time()dimensions to be estimated for
                 the TX position (else risk error due to singular matrix).
    '''
    h = np.asmatrix(h)
    G = np.asmatrix(G)
    Q_inv = np.asmatrix(np.linalg.pinv(Q))
    return np.linalg.pinv(np.transpose(G)*Q_inv*G)*np.transpose(G)*Q_inv*h # (14b)


def chan14a(h, G, Q, pos, est):
    '''
    pos: RX positions
    est:  an estimate of the TX position
    Q:   covariance matrix of TDoA or DoA measurements
    Returns also cov_z from (17) which is needed in (21)
    '''
    M,D = pos.shape
    h = np.asmatrix(h)
    G = np.asmatrix(G)
    Q = np.asmatrix(Q)
    r_i0 = np.zeros((M-1,1)) # 'true' distances
    for i in range(M-1):
        r_i0[i] = np.sqrt( (pos[i+1,0]-est[0])**2 + (pos[i+1,1]-est[1])**2 )
    B = np.asmatrix( np.diag( np.transpose(r_i0)[0] ) ) # (12)
    Psi_inv = np.linalg.pinv( pow(c,2)*B*Q*B )
    cov_z = np.linalg.pinv(np.transpose(G)*Psi_inv*G) # (17)
    new_est = cov_z*np.transpose(G)*Psi_inv*h # (14a)
    # gives same position result as (14b) always for TDoA-only case
    return cov_z, new_est
    

def chan22a(cov_z, pos, est):
    '''
    s. chan14a()
    cov_z needed for (21)
    '''

    M,D = pos.shape
    xy_1 = np.reshape( np.concatenate((pos[0], np.asarray([0]))), (D+1,1) )
    zd = est - np.asmatrix( xy_1 ) # (19)
    h_hat = np.asmatrix( np.square(zd) ) # (19)
    G_hat = np.asmatrix( np.vstack( (np.eye(2), np.ones(2)) ) ) # (19)
    B_hat = np.asmatrix(np.diag( np.transpose(np.asarray(zd))[0] )) # (21)
    Psi_hat_inv = np.linalg.pinv( 4*B_hat*cov_z*B_hat ) # (21)
    z_hat = np.linalg.pinv(np.transpose(G_hat)*Psi_hat_inv*G_hat)*np.transpose(G_hat)*Psi_hat_inv*h_hat # (22a)
    # for the following sign correction the first estimate has to be good enough!
    new_est = np.multiply( np.sqrt( np.abs(z_hat)), np.sign(zd[:2])) + np.reshape(pos[0], (D,1)) # (24)
    return new_est


def estimate_delay(y1, y2):
    correlation = np.absolute(np.correlate(y1, y2, "full")).tolist()
    delay = (np.argmax(correlation) - len(y1) + 1).tolist()
    return delay

def estimate_delay_interpolated(y1, y2):
    correlation, delay = corr_spline_interpolation(y1, y2, 11)
    return delay.tolist()

def localize(receivers, ref_receiver, bbox, xk_prio=np.array([]), delay = []):
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
        elif receiver.selected_position == "selfloc":
            if key == ref_receiver:
                pos_rx.insert(0, receiver.coordinates_selfloc)
            else:
                pos_rx.append(receiver.coordinates_selfloc)
        else:
            if key == ref_receiver:
                pos_rx.insert(0, receiver.coordinates_gps)
            else:
                pos_rx.append(receiver.coordinates_gps)
    #cross correlations
    d = []
    if len(delay) == 0:
        for receiver in receivers:
            if receiver != ref_receiver:
                if receivers[receiver].correlation_interpolation:
                    d.append(float(estimate_delay_interpolated(receivers[receiver].samples, receivers[ref_receiver].samples))/sample_rate)
                else:
                    d.append(float(estimate_delay(receivers[receiver].samples, receivers[ref_receiver].samples))/sample_rate)
    else:
        for j, receiver in enumerate(receivers):
            if receiver != ref_receiver:
                d.append(delay[j]/sample_rate)

    if len(pos_rx) == 3:
        return chan_3rx(pos_rx, d, xk_prio)
    else:
        # see chan, ho: A simple and efficient estimator for hyperbolic location
        # first construct covariance matrix
        Q_shape = 0.5 *np.ones(shape=(len(pos_rx)-1,len(pos_rx)-1)) + 0.5 * np.eye(len(pos_rx)-1)
        # scale with noise power
        P_noise = receivers.values()[0].measurement_noise
        Q = P_noise*Q_shape
        return chan_tdoa(pos_rx, d, Q)

    

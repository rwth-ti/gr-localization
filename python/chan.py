# -*- coding: utf-8 -*-
'''
Equation solving for TDoA Localization technique, as described in

Y. T. Chan, "A simple and efficient estimator for hyperbolic location"
IEEE Transactions on Signal Processing Vol.42, No.8, August 1994

pos: contains positions of the M receivers in [MxD]-form with D the dimensions
d:   TDoA delays with respect to first receiver in [M-1x1]-form
Q:   Covariance matrix of the delay vector d
'''
import numpy as np

c = 299700000.0

# Solution for 4 or more receivers with source in near field
def chan_tdoa(pos, d, Q):
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
    
    #try:
    # ############################ First step (14b) ############################
    # For near-field source use (14b) first to give an approximation of B
    h = 0.5*(r_i1**2 - K[1:] + K[0]) # (11)
    G = -np.concatenate((x_i1, y_i1, r_i1),1) # (11)
    z1 = chan14b(h, G, Q) # first position estimate
    #print 'z1:', z1
    # ########################## Second step (14a) ############################
    cov_z, z2 = chan14a(h, G, Q, pos, z1)
    #print 'z2:', z2
    # ########################### Third step (22a) ############################
    z3 = chan22a(cov_z, pos, z2)
    #print 'z3:', z3
#    except:
#        print 'Chan TDoA Error'
#        z1 = np.reshape([0]*3, (D+1,1))
#        z3 = np.reshape([0]*2, (D,1))
    xy = (z3[0],z3[1])
    print "TDoA Chan results: ",xy
    return {"coordinates": xy}


def chan14b(h, G, Q):
    '''
    Be adviced!: For use of chan14b() the number of TDoA-receivers should be at
                 least D+2, with D the number of dimensions to be estimated for
                 the TX position (else risk error due to singular matrix).
    '''
    h = np.asmatrix(h)
    G = np.asmatrix(G)
    Q_inv = np.asmatrix(np.linalg.inv(Q))
    return np.linalg.inv(np.transpose(G)*Q_inv*G)*np.transpose(G)*Q_inv*h # (14b)


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
    Psi_inv = np.linalg.inv( pow(c,2)*B*Q*B )
    cov_z = np.linalg.inv(np.transpose(G)*Psi_inv*G) # (17)
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
    Psi_hat_inv = np.linalg.inv( 4*B_hat*cov_z*B_hat ) # (21)
    z_hat = np.linalg.inv(np.transpose(G_hat)*Psi_hat_inv*G_hat)*np.transpose(G_hat)*Psi_hat_inv*h_hat # (22a)
    # for the following sign correction the first estimate has to be good enough!
    new_est = np.multiply( np.sqrt( np.abs(z_hat)), np.sign(zd[:2])) + np.reshape(pos[0], (D,1)) # (24)
    return new_est



'''
Hybrid TDoA/AoA Estimation as described in:
    Hybrid TDoA/ AoA Mobile user Location for Wideband CDMA Cellular Systems

Only one DoA-Measurement at RX1 (reference RX) can be incorporated.

theta: Noisy DoA-Measurement from reference receiver. Orientation of the angle 
        is reversed compared ot our MUSIC convention!
'''
def hybrid_chan(rx_pos, d, theta, Q):
    M,D = rx_pos.shape # number of receiver stations M and dimensions D
    pos1 = np.copy(rx_pos[0]) # save position of first RX for later
    pos = rx_pos - pos1 # Set Reference RX 1 in origin -> recalculate coordinates
    pos1 = np.reshape(pos1,(D,1))
    K = np.zeros((M-1,1))
    r_i1 = np.zeros((M-1,1)) # range difference in reference to receiver 1
    for i in range(M-1):
        r_i1[i] = c * d[i]
        K[i] = pos[i+1,0]**2 + pos[i+1,1]**2
    
    #try:
    # ############################ First step (14b ############################
    # For near-field source use (14b) first to give an approximation of B
    h = 0.5*(np.concatenate((r_i1**2 - K, np.atleast_2d(0)),axis=0)) # (9)
    G = np.concatenate( (pos[1:], r_i1), axis=1)
    G = -np.concatenate((G, np.atleast_2d([-np.sin(theta), np.cos(theta), 0])), axis=0) # (9)
    # If more than 3 RX are present, (14b) can be used to give a first estimate
    if M > 3:
        z1 = chan14b(h, G, Q) # first position estimate
    else:
        z1 = np.asarray([500,500]) # random point usually works
    z1_ext = z1 + np.concatenate((pos1,np.atleast_2d(0)), axis=0)
    print 'z1:', z1_ext # Outside of funtion coodinates not referenced to RX1
    # ########################## Second step (14a) ############################
    cov_z, z2 = hybrid_chan14a(h, G, Q, pos, z1)
    z2_ext = z2 + np.concatenate((pos1,np.atleast_2d(0)), axis=0)
    print 'z2:', z2_ext
    # ########################### Third step (22a) ############################
    z3 = chan22a(cov_z, pos, z2)
#    if theta > np.pi/2:
#        z3[0] = z3[0]*(-1)
    z3_ext = z3 + pos1
    print 'z3:', z3_ext
#    except:
#        print 'Chan Hybrid Error'
#        z2_ext = np.reshape([0]*3, (D+1,1))
#        z3_ext = np.reshape([0]*2, (D,1))
        
    return z2_ext[:2], z3_ext
        
    
def hybrid_chan14a(h, G, Q, pos, est):
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
    r_i0 = np.zeros((M,1)) # 'true' distances
    for i in range(M):
        r_i0[i] = np.sqrt( (pos[i,0]-est[0])**2 + (pos[i,1]-est[1])**2 )
    B = np.asmatrix( np.diag( np.concatenate((np.transpose(r_i0[1:])[0], r_i0[0]/c)) ) ) # (12)
    Psi_inv = np.linalg.inv( pow(c,2)*B*Q*B )
    cov_z = np.linalg.inv(np.transpose(G)*Psi_inv*G) # (17)
    new_est = cov_z*np.transpose(G)*Psi_inv*h # (14a)
    return cov_z, new_est



# Solution for 3 receivers (equation (10) in paper) (Manuel)
def chan10(pos_rx, d):
    M,D = pos_rx.shape
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
    
        # Solve equations system
        A = (y21*r31-y31*r21)/(y31*x21-y21*x31)
        B = (y21*(0.5*(pow(r31,2)-K3+K1))-y31*(0.5*(pow(r21,2)-K2+K1)))/(y31*x21-y21*x31)
        C = (x31*r21-x21*r31)/(y31*x21-y21*x31)
        D = (x31*(0.5*(pow(r21,2)-K2+K1))-x21*(0.5*(pow(r31,2)-K3+K1)))/(y31*x21-y21*x31)
    
        alpha = pow(A,2)+pow(C,2)-1
        beta = 2*(A*B+C*D-A*x1-C*y1)
        gamma = pow(x1,2)+pow(y1,2)-2*x1*B-2*y1*D+pow(B,2)+pow(D,2)
    
        r1 = [(-beta+np.sqrt(pow(beta,2)-4*alpha*gamma+0j))/(2*alpha),(-beta-np.sqrt(pow(beta,2)-4*alpha*gamma+0j))/(2*alpha)]
        # Calculate position with the solution in the roi
        xy = np.real((max(r1)*A+B,max(r1)*C+D))
    except:
        xy = np.reshape([0]*2,(D,1))
    print 'TDoA Estimate:', xy
    return xy
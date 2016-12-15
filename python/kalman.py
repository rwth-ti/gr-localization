import numpy as np
from collections import deque
from numpy import dot,array,identity
from numpy.linalg import inv
import sys, math
import pdb

class kalman_filter():

    def __init__(self,init_settings):
        self.window_size = 7
        self.correction_sequences = deque([])
        self.xk_1=[]
        self.model=init_settings["model"]
        self.delta_t=init_settings["delta_t"]
        self.noise_factor=init_settings["noise_factor"] 
        self.filter_receivers=init_settings["filter_receivers"] #integrate if necessary 
        self.noise_var_x=init_settings["noise_var_x"] 
        self.noise_var_y=init_settings["noise_var_y"]
        self.cnt_valid_locations=0
        self.cnt_invalid_locations=0
        self.max_acceleration=init_settings["max_acceleration"]
        self.R_chan =   array( [[  1,  0.5],
                     [0.5,  1]])*init_settings["noise_var_x"]
        self.R_cs =    array( [[  1,  -0.14],
                     [-0.14,  1]])*init_settings["noise_var_x"]
        
        self.R_dop=np.array([])
             
        self.B =0 
        self.state_noise=(self.delta_t**2)*self.noise_factor  #if performance decreases -> back to delta_t
        if self.model == "simple":
            self.state_size=4
            self.init_cov = array([[self.noise_var_x, 0, 0, 0],
                    [0, self.noise_var_y, 0, 0],
                    [0, 0, self.noise_var_x/self.delta_t, 0],
                    [0, 0, 0, self.noise_var_y/self.delta_t ]])
            self.phi = array([[1, 0, self.delta_t, 0,],
                           [0, 1, 0, self.delta_t],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1 ]])
            self.M = array([[1, 0, 0, 0],
                   [0, 1, 0, 0 ]] )
            self.Q = array([[0, 0, 0, 0],
                   [0, 0, 0, 0] ,
                   [0, 0, 1, 0 ],
                   [0, 0, 0, 1 ]])*self.state_noise

                    

            self.full_state_size=self.state_size 
        if self.model == "maneuvering":
            self.state_size=6 
            #correlation coefficient
            self.rho=0.15
            

            self.init_cov =array( [[self.noise_var_x, 0, self.noise_var_x/self.delta_t, 0, 0, 0] ,
                       [ 0, self.noise_var_y, 0, self.noise_var_y/self.delta_t, 0, 0 ],
                       [self.noise_var_x/self.delta_t, 0, self.noise_var_x/self.delta_t**2+self.state_noise, 0, 0, 0 ] ,
                       [ 0, self.noise_var_y/self.delta_t, 0, self.noise_var_y/self.delta_t**2+self.state_noise,  0, 0 ] ,
                       [ 0, 0, 0, 0, self.state_noise, 0 ],
                       [ 0, 0, 0, 0, 0, self.state_noise ]] )
            self.phi =array([[1, 0, self.delta_t, 0, 0,  0],
                      [ 0, 1, 0, self.delta_t, 0,  0 ],
                      [ 0, 0, 1, 0, 1,    0 ],
                      [ 0, 0, 0, 1, 0,    1 ],
                      [ 0, 0, 0, 0, self.rho,  0 ],
                      [ 0, 0, 0, 0, 0,  self.rho]] )
            self.M = array([[1, 0, 0, 0, 0, 0],
                       [0, 1, 0, 0, 0, 0]]) 
            self.Q = array([[0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0] ,
                       [0, 0, 0, 0, 1, 0] ,
                       [0, 0, 0, 0, 0, 1 ]])*self.state_noise 

            self.full_state_size=self.state_size 
#    def set_delta_t(self,delta_t):
#        self.delta_t=delta_t
    def update_matrices_post(self,delta_t):
        self.delta_t=delta_t
        self.Q=self.Q/self.state_noise
        
        self.state_noise=(self.delta_t**2)*self.noise_factor
        self.Q=self.Q*self.state_noise
        if self.model =="simple":
            self.phi = array([[1, 0, self.delta_t, 0,],
                           [0, 1, 0, self.delta_t],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1 ]])
            
        if self.model=="maneuvering":
            self.phi =array([[1, 0, self.delta_t, 0, 0,  0],
                      [ 0, 1, 0, self.delta_t, 0,  0 ],
                      [ 0, 0, 1, 0, 1,    0 ],
                      [ 0, 0, 0, 1, 0,    1 ],
                      [ 0, 0, 0, 0, self.rho,  0 ],
                      [ 0, 0, 0, 0, 0,  self.rho]] )
                      
    def get_state_size(self):
        return self.state_size
    def get_init_cov(self):
        return self.init_cov            
    def time_update(self, Pk_1, xk_1):
        xk_prio= dot(self.phi,xk_1.transpose())
        Pk_prio= dot(dot(self.phi,Pk_1),self.phi.transpose())+self.Q 
        return xk_prio, Pk_prio
    def get_a_priori_est(self,xk_1):
        return dot(self.phi,xk_1.transpose())
    def pre_filter(self,measurement,xk_1):
        if np.sqrt((np.linalg.norm((measurement[0]-xk_1[0])/self.delta_t-xk_1[2]))**2+(np.linalg.norm((measurement[1]-xk_1[1])/self.delta_t-xk_1[3]))**2)/(2*self.delta_t)>self.max_acceleration and self.cnt_valid_locations>5 and self.cnt_invalid_locations<3:
            measurement=xk_1[:2]+np.random.normal()
            
            self.cnt_invalid_locations=self.cnt_invalid_locations+1
            print "acceleration exeeds the greatest value allowed. The Kalman-Filter prediction will be used."
            if self.cnt_invalid_locations==3:
                self.cnt_valid_locations=0
        else:
            self.cnt_invalid_locations=0
            self.cnt_valid_locations=self.cnt_valid_locations+1
        return measurement
        
        
    def scale_measurement_noise(self,dop):
        # scale kalman filter measurement matrix depending on dilution of precision
        self.R_dop = self.R_chan*dop

    def adapt_R(self,H):
        # scale kalman filter measurement matrix depending on dilution of precision
        
        P_inv = inv(dot(H.T,H))
        self.R_dop = dot(dot(dot(dot(P_inv,H.T),self.R_chan),H),P_inv)
        
        idxs=np.where(self.R_dop>20)
        #print idxs
        if idxs[0].any() or idxs[1].any():
            self.R_dop[idxs]=20
        idxs=np.where(self.R_dop<-20)
        print idxs
        if idxs[0].any() or idxs[1].any():
            self.R_dop[idxs]=-20
        #print self.R_dop
        
    '''
    def adapt_Q(self,Pk,Pk_1):
        if len(self.correction_sequences) == self.window_size:
            
            self.Q = np.divide(np.sum(np.array(self.correction_sequences),0),self.window_size)#-dot(dot(self.phi,Pk_1),self.phi.transpose())+Pk
            
        print self.Q
    
    '''
    def kalman_fltr(self, measurement, Pk_1, xk_1, algorithm):
        # measurement:vector,Pk_1:mxm matrix,xk_1: size m-vector, self:containing state propagation matrices, delta_t:time distance between measurements
        self
        if not self.R_dop.any():
            if algorithm == "chan":
                R=self.R_chan
            elif algorithm == "grid_based":
                R=self.R_cs
            else:
                sys.exit('algorithm does not exist')
        else:
            R = self.R_dop

        #a priori estimations
        #Time Update
        xk_prio, Pk_prio=self.time_update(Pk_1, xk_1)

        #Kalman Gain
        Kk=dot(dot(Pk_prio,self.M.transpose()),inv(dot(dot(self.M,Pk_prio),self.M.transpose())+R))
    
        #Measurement Update
        Pk=dot((identity(self.full_state_size)-dot(Kk,self.M)),Pk_prio)
        xk= (xk_prio+dot(Kk,(measurement.transpose()-dot(self.M,xk_prio)))).transpose()
        '''
        if any(self.xk_1):
            self.correction_sequences.append(dot(np.array([dot(Kk,(measurement.transpose()-dot(self.M,xk_prio)))]).T,np.array([dot(Kk,(measurement.transpose()-dot(self.M,xk_prio)))]))
)       
        if len(self.correction_sequences) > self.window_size:
            self.correction_sequences.popleft()
            '''
        self.xk_1 = xk
        #self.adapt_Q(Pk,Pk_1)
        return xk ,Pk
        
        


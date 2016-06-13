import numpy as np
from numpy import dot,array,identity
from numpy.linalg import inv
import sys, math

class kalman_filter():

    def __init__(self,init_settings):
        self.model=init_settings["model"]
        self.delta_t=init_settings["delta_t"]
        self.noise_factor=init_settings["noise_factor"] 
        self.filter_receivers=init_settings["filter_receivers"] #integrate if necessary 
        
        self.noise_var_x=init_settings["noise_var_x"] 
        self.noise_var_y=init_settings["noise_var_y"]
        self.max_acceleration=init_settings["max_acceleration"]
        self.R_chan =   array( [[  1,  -0.135],
                     [-0.135,  1]])*init_settings["measurement_noise_chan"]
        self.R_cs =    array( [[  1,  -0.14],
                     [-0.14,  1]])*init_settings["measurement_noise_grid"]
             
        self.B =0 
        self.state_noise=(self.delta_t**2)*self.noise_factor  #if performance decreases -> back to delta_t
        if self.model == "hellebrandt":
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
            self.rho=0.1 
            

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
        if self.model =="hellebrandt":
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
        if np.sqrt((np.linalg.norm((measurement[0]-xk_1[0])/self.delta_t-xk_1[2]))**2+(np.linalg.norm((measurement[1]-xk_1[1])/self.delta_t-xk_1[3]))**2)/(2*self.delta_t)>self.max_acceleration:
            measurement=xk_1[:2]+np.random.normal()
            print "acceleration exeeds the greatest value allowed. The Kalman-Filter prediction will be used."
        return measurement
    def kalman_fltr(self, measurement, Pk_1, xk_1, algorithm,invalid=False):
    #measurement:vector,Pk_1:mxm matrix,xk_1: size m-vector, self:containing state propagation matrices, delta_t:time distance between measurements
        if algorithm == "chan":
            R=self.R_chan
        elif algorithm == "grid_based":#replace with grid based(?)
            R=self.R_cs
        else:
            sys.exit('algorithm does not exist')
    
        #a priori estimations
        #Time Update
        xk_prio, Pk_prio=self.time_update(Pk_1, xk_1)
        #if not invalid:
        #Kalman Gain
        Kk=dot(dot(Pk_prio,self.M.transpose()),inv(dot(dot(self.M,Pk_prio),self.M.transpose())+R))
    
        #Measurement Update
        Pk=dot((identity(self.full_state_size)-dot(Kk,self.M)),Pk_prio)
        xk= (xk_prio+dot(Kk,(measurement.transpose()-dot(self.M,xk_prio)))).transpose()
        #pdb.set_trace()
        return xk ,Pk


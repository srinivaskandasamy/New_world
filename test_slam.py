from math import tan, atan, sqrt, cos, sin, pi
from numpy import eye, diag, array, exp, ceil, floor, dot
from numpy.linalg import inv, det
from random import gauss, randint, uniform
import copy
import sys

class Particle:
    def __init__(self,pose):
        self.pose = pose
        self.trajectory = []
        self.landmark_poses = []
        self.landmark_covariances = []
        self.hist_landmark = ['map',['0'],['pi/2'],['pi'],['3pi/2']]
        
    def number_of_landmarks(self):
        return len(self.landmark_poses)
    
    @staticmethod
    def motion(pose, control, WB, dt):  
        angle = (control[1]+pose[2])%(2*pi)
        return ([x+y for x,y in zip(pose,[control[0]*dt*cos(angle),
                                          control[0]*dt*sin(angle),
                                          control[0]*dt*sin(control[1])/WB])])
    
    def move(self,control,WB,dt,pose):
        self.pose = self.motion(pose,control,WB,dt)
        self.trajectory.append(self.pose)
        
    def expected_measurement(self,pose,landmark):     
        return array([landmark[0],landmark[1],tan(landmark[2]-pose[2])])      
    
    def measurement_correspondence(self,pose,measurement,number_of_landmarks,Qt_measurement_covariance):
        likelihoods = []
        for i in range(number_of_landmarks):
            likelihoods.append(self.landmark_correspondence(measurement,i,Qt_measurement_covariance))
        return likelihoods
    
    @staticmethod
    def dh_dlandmark(pose,landmark):
        return array([[1,0,0],[0,1,0],[0,0,-1/(cos(pose[2]-landmark[2])**2)]]) # H - Direct odometry measurement and collision
    
    def H_and_Ql_from_measurement(self,landmark_number,Qt_measurement_covariance):
        
        H = self.dh_dlandmark(self.pose,self.landmark_poses[landmark_number])
        Ql = dot(dot(H,self.landmark_covariances[landmark_number]),H.T) + Qt_measurement_covariance
        
        return (H, Ql)
        
    def landmark_correspondence_likelihood(self,measurement,landmark_number,Qt_measurement_covariance):
        # For a given measurment and a landmark number, it returns a suitable likelihood value of the correspondence

        landmark = self.landmark_poses[landmark_number]
        phi = landmark[2]
        theta = self.pose[2]
        
        # Prune the landmark which are not in the robot's frame of collision
        if phi == pi/2 * floor(theta/(pi/2)) or phi == pi/2 * ceil(theta/(pi/2)): 
            zhat = self.expected_measurement(self.pose,landmark)
            H, Ql = self.H_and_Ql_from_measurement(landmark_number,Qt_measurement_covariance)
            dz = measurement - zhat
            # Compute likelihood
            sqrtdetQl = sqrt(det(Ql))
            normal = 1 / (2*pi*sqrtdetQl)
            l = normal * exp(-0.5*dot(dot(dz.T,inv(Ql)),dz))            
        else: 
            l = 0
        return l
    
    def initialize_new_landmark(self,measurement,Qt_measurement_covariance):

        self.landmark_poses.append([self.pose[0],self.pose[1],self.pose[2]+atan(measurement[2])]) # Orientation update as FSM
        # We have to modify the orientation of the landmark to multiples of 90 degrees
        Hinv = array([[1,0,0],[0,1,0],[0,0,1/(1+measurement[2]**2)]])
        Sigma = dot(dot(Hinv,Qt_measurement_covariance),Hinv.T)
        # Remember to add the covariance of the particle at that moment
        
        self.landmark_covariances.append(Sigma)
        
    def update_landmark(self,measurement,landmark_number,Qt_measurement_covariance):
        
        mu = self.landmark_poses[landmark_number]
        Sigma = self.landmark_covariances[landmark_number]
        H, Ql = self.H_and_Ql_from_measurement(landmark_number,Qt_measurement_covariance)
        K = dot(dot(Sigma,H.T),inv(Ql))
        
        mu = mu + dot(K,(measurement - self.expected_measurement(self.pose,mu)))
        Sigma = dot((eye(3) - K*H),Sigma)
        
        return (mu,Sigma)
    
    def update_particle(self,measurement,number_of_landmarks,minimum_correspondence_likelihood,Qt_measurement_covariance):
        likelihoods = []
        for i in range(number_of_landmarks):
            likelihoods.append(self.landmark_correspondence_likelihood(measurement,i,Qt_measurement_covariance))

        if not likelihoods or max(likelihoods) < minimum_correspondence_likelihood:
            self.initialize_new_landmark(measurement,Qt_measurement_covariance)
            return minimum_correspondence_likelihood
        
        else:
            lmax = max(likelihoods)
            lmax_index = likelihoods.index(lmax)
            mu,Sigma = self.update_landmark(measurement,lmax_index,Qt_measurement_covariance)
            self.landmark_poses[lmax_index] = mu
            self.landmark_covariances[lmax_index] = Sigma
            
            return lmax

class FastSLAM:
    def __init__(self,initial_particles,robot_width,minimum_correspondence_likelihood,measurement_stddev,
                 x_stddev,y_stddev,control_speed_factor,control_head_factor, sample_time):
        # Particles
        self.particles = initial_particles
        
        # Constants
        self.robot_width = robot_width
        self.minimum_correspondence_likelihood = minimum_correspondence_likelihood
        self.xstddev = x_stddev
        self.ystddev = y_stddev
        self.measurement_stddev = measurement_stddev
        self.control_speed_factor = control_speed_factor
        self.control_head_factor = control_head_factor
        self.dt = sample_time
        self.WB = robot_width
        
    def predict(self,pose,control):
        # Prediction step of FastSLAM

        speed, head = control
        speed_std = self.control_speed_factor * sqrt(speed) # To be modified
        head_std  = self.control_head_factor * sqrt(359 - head) # To be modified; mirror image of the speed deviation
        
        for p in self.particles:
            speed = gauss(speed,speed_std)
            head = gauss(head,head_std)
            p.move([speed,head],self.WB,self.dt,pose)
    
    def update_and_compute_weights(self,measurement):

        Qt_measurement_covariance = diag([self.xstddev ** 2, self.ystddev ** 2,self.measurement_stddev ** 2]) 
        weights = []
        for p in self.particles:
            number_of_landmarks = p.number_of_landmarks()
            weight = p.update_particle(measurement,number_of_landmarks,self.minimum_correspondence_likelihood,
                                            Qt_measurement_covariance)
            weights.append(weight)
            
        return weights
    
    def resample(self,weights):
        
        new_particles = []
        max_weight = max(weights)
        index = randint(0, len(self.particles)-1)
        offset = 0.0
        for i in xrange(len(self.particles)):
            offset += uniform(0, 2.0 * max_weight)
            while offset > weights[index]:
                offset -= weights[index]
                index = (index + 1) % len(weights)
            new_particles.append(copy.deepcopy(self.particles[index]))
            
        return new_particles
    
    def correct(self,x,y,theta,wall_type):
        if wall_type == 1:
            phi=pi
        elif wall_type == 3:
            phi = 0
        elif wall_type == 2:
            phi = 3*pi/2
        elif wall_type == 4:
            phi = pi/2
            
        measurement = array([x,y,tan(phi-theta)])
        
        weights = self.update_and_compute_weights(measurement)
        self.particles = self.resample(weights)
        
        return weights
    
    def prune(self,w):
        prune_traj = []
        for i in range(2):
            
        
if __name__ == '__main__':
    
    robot_width = 2
    sample_time = 0.01
    
    minimum_correspondence_likelihood = 1e-3
    xstddev = 0.001
    ystddev = 0.001
    measurement_stddev = 0.001
    
    control_speed_factor = 0.01
    control_head_factor = 0.01
    number_of_particles = 10
    
    start_state = poses[0][0:3]
    initial_particles = [copy.copy(Particle(start_state))
                         for _ in xrange(number_of_particles)]

    fs = FastSLAM(initial_particles,robot_width,minimum_correspondence_likelihood,measurement_stddev,xstddev,ystddev,
                 control_speed_factor,control_head_factor, sample_time)
    
    for i in xrange(lposes-1):
        # Correction step
        print "Iteration",i
        '''Use of entire trajectory for the correction step (Rao-Blackwellization)''' 
        w = fs.correct(array([poses[i][0],poses[i][1],tan(poses[i][3]-poses[i][2])])) # Odometry and collision values
        #Prediction
        fs.predict(poses[i+1][0:3],[100,poses[i+1][2]])
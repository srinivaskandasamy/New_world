from __future__ import division
import sphero_driver
import sys
import numpy as np
import math
import time
import threading
# import scipy.io     # import MATLAB file for the reference data
# import matplotlib.pyplot as plt     # plotting the reference data for reference

# from scipy.linalg import block_diag as bd
# from scipy.signal import lsim2, lti
# #    from numpy import linspace
# import numpy.matlib as nm

class SpheroNode(threading.Thread):
    
        x = 0        
        y = 0
       
        def __init__(self, thread_ID, name,color):
                self.update_rate = 50
                self.sampling_divisor = int(400 / self.update_rate)
                self.is_connected = False
                self.robot = sphero_driver.Sphero()
                self.cmd_heading = 0
                self.cmd_speed = 0
                self.power_state_msg = "No Battery Info"
                self.power_state = 0
                threading.Thread.__init__(self)
                self.thread_ID  =   thread_ID
                self.name   =   name
                self.star_work()
                self.set_color([color[0], color[1], color[2]])
                
        def run(self):
#                 threadLock = threading.Lock()
#                 threadLock.acquire()
#                 D   =   scipy.io.loadmat('ren3.mat')
#                 D_x =   D.get('X')
#                 D_y =   D.get('Y')  
                if self.name == "alpha":
                        print "alpha is up"
#                         for i in range(10):
#                             print D_x[4*i,0], D_y[4*i,0]
                        self.set_control(0,0)
                        print "alpha has completed"
                if self.name == "bravo":
                        print "bravo is up"
#                         for i in range(10):
#                             print D_x[4*i,1], D_y[4*i,1]
                        self.set_control(0,0)
                        print "bravo has completed"
                if self.name == "charlie":
                        print "charlie is up"
#                         for i in range(10):
#                             print D_x[4*i,1], D_y[4*i,1]
                        self.set_control(0,0)
                        print "charlie has completed"
#                 threadLock.release()                    

        def star_work(self):
                try:
                        self.is_connected = self.robot.connect()
                        self.raddress   =   self.is_connected
                        self.is_connected = True
                except:
                        print "Failed to connect"
                        sys.exit(1)    
                # setup streaming    
                self.robot.set_filtered_data_strm(self.sampling_divisor, 1 , 0, True)
                self.robot.add_async_callback(sphero_driver.IDCODE['DATA_STRM'], self.parse_data_strm)  
                self.robot.start()
                   
        def set_color(self, msg):
                if self.is_connected:
                        self.robot.set_rgb_led(int(msg[0]), int(msg[1]), int(msg[2]), 0, False)
                        
        def parse_data_strm(self, data):
                if self.is_connected:
                    self.x = data["ODOM_X"]
                    self.y = data["ODOM_Y"]
#                     q0 = int(data["QUATERNION_Q0"])
#                     q1 = int(data["QUATERNION_Q1"])
#                     q2 = int(data["QUATERNION_Q2"])
#                     q3 = int(data["QUATERNION_Q3"])
#                     a = +2 * (q0 * q3 + q1 * q2)
#                     b = +1 - 2 * (q2 * q2 + q3 * q3)
#                     d = (a * a + b * b) ** (1 / 2)
#                     hd = np.arctan((a / d) / (b / d)) * 180 / np.pi                                        
                    #print '[x:{0}],[y:{1}]'.format(self.x, self.y)
        
        def set_heading(self, angle):
                if self.is_connected:
                    heading_deg = int(self.normalize_angle_positive(angle) * 180.0 / math.pi)
                    self.robot.set_heading(heading_deg, False)
                    
        def normalize_angle_positive(self, angle):
                return math.fmod(math.fmod(angle, 2.0 * math.pi) + 2.0 * math.pi, 2.0 * math.pi);
                    
        def set_angular_velocity(self, vel):
                if self.is_connected:
                    rate = int((vel * 180 / math.pi) / 0.784)
                    self.robot.set_rotation_rate(rate, False)
                    
        def set_control(self,xd,yd):
            
                time.sleep(5)
                
                print "Setting heading"                
                hd = self.calculate_heading(xd,yd)         
                ed = self.calculate_error(xd,yd)

                print 'Starting control loop at [x:{0}],[y:{1}]'.format(self.x, self.y)
                
                xint = 0
                yint = 0
                
                while abs(ed) >= 1: 
                    
                    time.sleep(0.02)

                    speed = int(max(0,min(30,0.3 * ed+ 0.2 * abs(xint) + 0.2 * abs(yint))))
                    xint = min(100,xint + (self.x-xd))
                    yint = min(100,yint + (self.y-yd))
                    
                    self.robot.roll(speed,hd,1,False)
                    
                    ed = self.calculate_error(xd,yd)
                    hd = self.calculate_heading(xd,yd)
                    
                    print '[x:{0}],[y:{1}],[h:{2}],[e:{3}],[s:{4}],name:{5}'.format(self.x, self.y, hd, ed,speed,self.name)
                    
                print "Done"
                self.robot.roll(0,0,1,False)
                            
        def calculate_heading(self, xd, yd):
            
            ye = yd - self.y
            xe = xd - self.x
            
            if ye == 0 and xe >= 0:
                
                return 90
            
            elif ye == 0 and xe < 0:
                
                return 270
            
            else:
                
                angle = int(math.atan(abs(xe / ye)) / math.pi * 180)                       
                if ye > 0 and xe >= 0:
                    return angle
                if ye < 0 and xe >= 0:
                    return 90 + angle
                if ye < 0 and xe < 0:
                    return 180 + angle
                if ye > 0 and xe < 0:
                    return 270 + angle           
        
        def calculate_error(self,xd,yd):
            return np.sqrt((xd - self.x) ** 2 + (yd - self.y) ** 2)
        
        def stop(self):
                self.robot.roll(int(0), int(0), 1, False)
                self.robot.shutdown = True
                self.is_connected = self.robot.disconnect()
                self.robot.join()



if __name__ == '__main__':
    
    # Default empty thread
    threads  =   []
    
    # Thread creation 
    alpha = SpheroNode(1,"alpha",[255, 0, 0])
    bravo = SpheroNode(2,"bravo",[0, 255, 0])
    charlie = SpheroNode(3,"charlie",[0, 0, 255])
    
    # Running synchronized threads
    alpha.start()
    bravo.start()
    charlie.start()
    alpha.join()
    bravo.join()
    charlie.join()
    
    # Future reference data to substitute xd and yd in a loop fashion
#     D   =   scipy.io.loadmat('ren3.mat')
#     D_x =   D.get('X')
#     D_y =   D.get('Y')
#     
    # Reference plots for the formation 
#     plt.plot(D_x,D_y)
#     plt.show()
    
    # Spheros to sleep mode
#     time.sleep(100)
#     alpha.stop()
#     bravo.stop()
    
#     charlie = SpheroNode()
#     charlie.start()
#     charlie.set_color([0, 0, 255])
#     delta = SpheroNode()
#     delta.start()
#     delta.set_color([255, 255, 0])
#     golf = SpheroNode()
#     golf.start()
#     golf.set_color([255, 255, 255])
#     india = SpheroNode()
#     india.start()
#     india.set_color([255, 0, 255]) 
    
    
    

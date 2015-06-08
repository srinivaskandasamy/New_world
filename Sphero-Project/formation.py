from __future__ import division
import sphero_driver
import sys
import numpy as np
import math
import time

# from scipy.linalg import block_diag as bd
# from scipy.signal import lsim2, lti
# #    from numpy import linspace
# import numpy.matlib as nm

class SpheroNode(object):
    
        x = 0        
        y = 0
        
        def __init__(self, default_update_rate=50.0):
                self.update_rate = default_update_rate
                self.sampling_divisor = int(400 / self.update_rate)
                self.is_connected = False
                self.robot = sphero_driver.Sphero()
                self.cmd_heading = 0
                self.cmd_speed = 0
                self.power_state_msg = "No Battery Info"
                self.power_state = 0

        def start(self):
                try:
                        self.is_connected = self.robot.connect()
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

                    speed = int(max(0,min(125,0.02 * ed + 0.01 * abs(xint) + 0.01 * abs(yint))))
                    xint = min(100,xint + (self.x-xd))
                    yint = min(100,yint + (self.y-yd))
                    
                    self.robot.roll(speed,hd,1,False)
                    
                    ed = self.calculate_error(xd,yd)
                    hd = self.calculate_heading(xd,yd)
                    
                    print '[x:{0}],[y:{1}],[h:{2}],[e:{3}],[s:{4}]'.format(self.x, self.y, hd, ed,speed)
                    
                print "Done"
                self.robot.roll(0,0,1,False)
                
        def test(self):
            
            time.sleep(5)
            self.robot.roll(50,0,1,False)
            time.sleep(5)
            self.robot.roll(50,90,1,False)
            time.sleep(5)    
            self.robot.roll(50,180,1,False)
            time.sleep(5)
            self.robot.roll(50,270,1,False)
            time.sleep(5)
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
                self.is_connected   =   True
                self.is_connected = self.robot.disconnect()
                self.robot.join()

  def connect(self):
    sys.stdout.write("Searching for devices....")
    sys.stdout.flush() = self.robot.disconnect()
                self.robot.join()



if __name__ == '__main__':
    
    alpha = SpheroNode()
    alpha.start()
    alpha.set_color([255, 0, 0])
    alpha.set_control(80, 160)
    # alpha.test()
#    bravo = SpheroNode()
#    bravo.start()
#    bravo.set_color([0, 255, 0])
#    bravo.set_control(-80, 160)
    
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
    
    
    

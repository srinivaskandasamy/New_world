from __future__ import division
import sphero_driver
import sys
import numpy as np
import math
import time
import threading

REQ=dict(CMD_ROLL = [0x02, 0x30])

class SpheroNode(threading.Thread):
    
        x = 0        
        y = 0        
        
        def __init__(self, default_update_rate=50.0):
                self.update_rate = 50
                self.sampling_divisor = int(400 / self.update_rate)
                self.is_connected = False
                self.robot = sphero_driver.Sphero()
                self.cmd_heading = 0
                self.cmd_speed = 0
                self.power_state_msg = "No Battery Info"
                self.power_state = 0       
                threading.Thread.__init__(self)
            
        def run(self):
            
                threadLock = threading.Lock()
                threadLock.acquire()
                
                time.sleep(5)

                if self.name == "alpha":
                        print self.name
                        self.start_work()
                        self.set_color([255, 0, 0])
                        print "alpha is up"
                        self.set_control(-50, -50)
                if self.name == "bravo":
                        print self.name
                        self.start_work()
                        self.set_color([0,255,0])
                        print "bravo is up"
                        self.set_control(50, 50)  
                        
                threadLock.release()           
                   
        def set_color(self, msg):
                if self.is_connected:
                        self.robot.set_rgb_led(int(msg[0]), int(msg[1]), int(msg[2]), 0, False)
                        
        def parse_data_strm(self, data):
                if self.is_connected:
                    self.x = data["ODOM_X"]
                    self.y = data["ODOM_Y"]
        
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
                    
        def set_control(self, xd, yd):

                print "Setting heading"                
                hd = self.calculate_heading(xd, yd)         
                ed = self.calculate_error(xd, yd)
 
                print 'Starting control loop at [x:{0}],[y:{1}]'.format(self.x, self.y)
                 
                xint = 0
                yint = 0
                 
                while abs(ed) >= 1: 
                      
                    time.sleep(0.1)
  
                    speed = int(max(0, min(125, 0.015 * ed + 0.01 * abs(xint) + 0.01 * abs(yint))))
                    xint = min(100, xint + (self.x - xd))
                    yint = min(100, yint + (self.y - yd))
                      
                    self.roll(speed, hd, 1, False)
                      
                    ed = self.calculate_error(xd, yd)
                    hd = self.calculate_heading(xd, yd)
                      
                    print '[x:{0}],[y:{1}],[h:{2}],[e:{3}],[s:{4}],name:{5}'.format(self.x, self.y, hd, ed, speed, self)
                      
                print "Done"
                self.roll(0, 0, 1, False)
                              
        def roll(self, speed, heading, state, response):
            self.send(self.pack_cmd(REQ['CMD_ROLL'], [self.clamp(speed, 0, 255), (heading >> 8), (heading & 0xff), state]), response)
            
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
        
        def calculate_error(self, xd, yd):
            return np.sqrt((xd - self.x) ** 2 + (yd - self.y) ** 2)
        
        def stop(self):
                self.robot.roll(int(0), int(0), 1, False)
                self.robot.shutdown = True
                self.is_connected = self.robot.disconnect()
                self.robot.join()



if __name__ == '__main__':

    # Object creation 
    alpha = SpheroNode()
    bravo = SpheroNode()   
    alpha.start()
    bravo.start()

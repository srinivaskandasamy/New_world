from __future__ import division
import sphero_driver
import math
import numpy as np
from time import sleep
from math import atan, pi

REQ=dict(CMD_ROLL = [0x02, 0x30])

class SpheroNode(object):
    
    def __init__(self, default_update_rate=50.0):

        self.update_rate = default_update_rate
        self.sampling_divisor = int(400 / self.update_rate)
        self.sampling_divisor = 25
        self.is_connected = False
        self.robot = sphero_driver.Sphero()
        self.collipy = np.matrix([0, 0, 0, 0, 0, 0, 0, 0])
        self.odometry = np.matrix([0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.corner = np.matrix([0, 0, 0, 0, 0, 0, 0])
        self.t = np.matrix([0])
        self.head = 0
        self.wall_orientation = []
        self.timer = 0
        self.timer1=0
        
    def start(self):
        try:
            self.is_connected = self.robot.connect()
        except:
            print "Failed to connect Sphero"

        self.robot.set_filtered_data_strm(self.sampling_divisor, 1 , 0, True)
        self.robot.add_async_callback(sphero_driver.IDCODE['DATA_STRM'], self.parse_data_strm)
        self.robot.config_collision_detect(1, 45, 110, 45, 110, 100, False)
        self.robot.add_async_callback(sphero_driver.IDCODE['COLLISION'], self.parse_collision)

        self.robot.start()
        
    def parse_data_strm(self, data):
        if self.is_connected:
            self.xl = data["ODOM_X"]
            self.yl = data["ODOM_Y"]
            self.vx = data["VELOCITY_X"]
            self.vy = data["VELOCITY_Y"]
            self.ox = data["QUATERNION_Q0"]
            self.oy = data["QUATERNION_Q1"]
            self.oz = data["QUATERNION_Q2"]
            self.ow = data["QUATERNION_Q3"]
            self.lx = data["ACCEL_X_FILTERED"]/4096.0*9.8
            self.ly = data["ACCEL_Y_FILTERED"]/4096.0*9.8
            self.lz = data["ACCEL_Z_FILTERED"]/4096.0*9.8
            self.ax = data["GYRO_X_FILTERED"]*10*math.pi/180
            self.ay = data["GYRO_Y_FILTERED"]*10*math.pi/180
            self.az = data["GYRO_Z_FILTERED"]*10*math.pi/180
            
            self.timer += 1 # Odometry timer
            
            if self.timer >= 200 and self.timer1 >=80 and math.fabs(self.vx) <=10 and math.fabs(self.vy) <=10:
                self.head = self.head + 120
                self.head = self.angle_wrap(self.head)
                print "Corner",self.head
                sleep(2)
                self.timer1 = 0
                
            self.robot.roll(200, int(self.head), 1, False) #10X - 80
            self.timer1 += 1 # Timer for  corner collision
        
    def parse_collision(self, data):
        if self.is_connected:
            self.cx = data["X"]
            self.cy = data["Y"]
            self.cz = data["Z"]
            self.caxis = int(data["Axis"])
            self.cx_magnitude = data["xMagnitude"]
            self.cy_magnitude = data["yMagnitude"]
            self.cspeed = data["Speed"]
            self.ctimestamp = data["Timestamp"]
            
            theta = self.head # Heading of the robot
            collision_angle = atan(self.cx/self.cy)*180/pi # arctan of collision values 
            phi, correction = self.wall_correct(self.angle_wrap(theta + collision_angle)) # Orientation of the wall
            
            self.head = self.new_head(collision_angle,self.cx,self.cy,correction)       # New heading for the robot        
                     
#             print '[cx:{0}], [cy:{1}], [ax:{2}], [hd:{3}]'.format(self.cx, self.cy, phi, self.head, self)
    
    def set_color(self, msg):
        if self.is_connected:
            self.robot.set_rgb_led(int(msg[0]), int(msg[1]), int(msg[2]), 0, False)
            
    def wall_correct(self,phi):
        
        if self.wall_orientation != []:
            ref = self.wall_orientation[0]
            phi = phi - ref # Coorection to angle wrap
            if phi >= 45 and phi < 135:
                correction = phi - 90
                phi = 90
                print "90 degree wall detected"
            elif phi >= 135 and phi < 225:
                correction = phi - 180
                phi = 180
                print "180 degree wall detected"
            elif phi >= 225 and phi < 315:
                correction = phi - 270
                phi = 270
                print "270 degree wall detected"
            else:
                print "0 degree wall detected"
                if phi > 0:
                    correction = phi
                else:
                    correction = phi - 360
                phi = 0
            
        else:
            self.wall_orientation.append(phi)
            self.wall_orientation.append(self.angle_wrap(phi+90))
            self.wall_orientation.append(self.angle_wrap(phi+180))
            self.wall_orientation.append(self.angle_wrap(phi+270))
            correction = 0
            print "Wall set",self.wall_orientation
            
        return (phi,correction)
    
    def new_head(self,collision_angle,cx,cy,correction):
        reflection = 45 # Extended to 30 and 60 degrees for higher planning
        if cx >= 0 and cy > 0:
#             self.head = phi - 90 - reflection
#             self.head = self.head - (reflection + 90 - collision_angle)
            self.head = self.head - (reflection + 90 - collision_angle)# - correction - able o detect 90 degree wall perfectly without correction (WYP)
        elif cx < 0 and cy > 0:
#             self.head = phi + 90 + reflection
            self.head = self.head + (reflection + 90 - collision_angle)# - correction
            
        return self.angle_wrap(self.head) 
    
    def roll(self, speed, heading, state, response):
            self.send(self.pack_cmd(REQ['CMD_ROLL'], [self.clamp(speed, 0, 255), (heading >> 8), (heading & 0xff), state]), response)
            
    def angle_wrap(self, head):
        
        while head < 0 or head > 359:
            
            if head > 359:
                    head = head - 360
            elif head < 0:
                    head = head + 360         
                
        return head        
    
if __name__ == '__main__':
    print "Starting..."
    s = SpheroNode()
    s.start()
#     s.robot.roll(0,0,1,False)
    print "Junga"
    print "Over"
    
from __future__ import division
import sphero_driver
import math
import numpy as np
from time import sleep
data1 = open('collipydata.dat','a')
data2 = open('odomdata.dat','a')
data3 = open('corner.dat','a')

REQ=dict(CMD_ROLL = [0x02, 0x30],CMD_SET_HEADING = [0x02, 0x01])

class SpheroNode(object):
    
    def __init__(self, default_update_rate=50.0):

        self.update_rate = default_update_rate
        self.sampling_divisor = int(400 / self.update_rate)

        self.is_connected = False
        self.robot = sphero_driver.Sphero()
        self.collipy = np.matrix([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.odometry = np.matrix([0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.corner = np.matrix([0, 0, 0, 0, 0, 0, 0])
        self.t = np.matrix([0])
        self.head = 45
        self.det = 1000
        self.timer = 0
        self.timer1=0
        self.timer2=1000
        self.speed = 60
        
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
            
#             print '[x:{0}],[y:{1}],[vx:{2}],[vy:{3}]'.format(self.xl, self.yl, self.vx, self.vy)
            self.odometry = np.vstack([self.odometry, np.matrix([self.xl, self.yl, self.vx, self.vy, self.lx, self.ly, self.ax, self.ay, self.timer])])
            np.savetxt('odomdata.dat',self.odometry)
            self.timer += 1
            self.timer2 += 1
            
            if self.timer >= 200 and self.timer1 >=80 and math.fabs(self.vx) <=10 and math.fabs(self.vy) <=10:
                self.head = self.head + 150
                self.head = self.angle_wrap(self.head)
                print "Corner",self.head
                self.corner = np.vstack([self.corner,np.matrix([self.xl, self.yl, self.vx, self.vy, self.lx, self.ly, self.head])])
                np.savetxt('corner.dat',self.corner)
                sleep(2)
                self.timer1=0
            if self.timer2 > 250:
                self.speed=60    
                s.robot.set_stablization(1, False)
            s.robot.roll(self.speed, int(self.head), 1, False)
#             self.timer1 += 1
        
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
            self.det = 1000

            myaxis = self.head + math.atan(self.cx/self.cy)*(180/math.pi)
            myaxis = self.angle_wrap(myaxis)
            if myaxis >=45 and myaxis <= 135:
                self.det = 90
                dif = 90 - myaxis
                self.head = self.head + dif
                
            elif myaxis >= 225 and myaxis <= 315:
                self.det = 270
                dif = 270 - myaxis
                self.head = self.head - dif
                
            self.head = self.angle_wrap(self.head)
                
             
            if self.cx > 0 and self.cy > 0: # if X/Y or Y/X is small (head-on collision), change the orientation in a different direction

                self.head = self.head   -  (180 - 1.2 * (180/math.pi) * math.atan(math.fabs(self.cx)/math.fabs(self.cy))) 
                self.speed=0
                self.timer2=0
                s.robot.set_stablization(0,False)
#                 s.robot.roll(0,180,1,False)
#                 sleep(5)
            elif self.cx < 0 and self.cy > 0:
#                 print "Left"
                self.head = self.head   +  (180 - 1.2 * (180/math.pi) * math.atan(math.fabs(self.cx)/math.fabs(self.cy)))
                self.speed=0
                self.timer2=0
                s.robot.set_stablization(0,False)
#                 s.robot.roll(0,180,1,False)
#                 sleep(5)
#             print "After Collision",self.head
            if self.head > 359:
                self.head = self.head - 360
            elif self.head < 0:
                self.head = self.head + 360            
             
#             s.robot.roll(0,int(self.head),1,False)
#             sleep(2)
            print '[cx:{0}], [cy:{1}], [ax:{2}], [hd:{3}], [max:{4}]'.format(self.cx, self.cy, self.caxis, self.head, myaxis, self)
   
            self.collipy = np.vstack([self.collipy, np.matrix([self.cx, self.cy, self.det, myaxis, self.cx_magnitude, self.cy_magnitude, self.caxis, self.cspeed, self.head, self.timer])])
            
            np.savetxt('collipydata.dat',self.collipy)
    
    def set_color(self, msg):
        if self.is_connected:
            self.robot.set_rgb_led(int(msg[0]), int(msg[1]), int(msg[2]), 0, False)
    
    def roll(self, speed, heading, state, response):
            self.send(self.pack_cmd(REQ['CMD_ROLL'], [self.clamp(speed, 0, 255), (heading >> 8), (heading & 0xff), state]), response)
            
    def sphero_calibrate(self,offset): # offset in degrees
        s.robot.set_back_led(255, False)
        s.robot.set_stablization(0, False)
        sleep(5)
        print "Rolling start"
        s.robot.roll(0,int(offset),1,False)
        print "Rolling Done"
        sleep(5)
        s.robot.set_heading(0, False)
        s.robot.set_back_led(0, False)
        s.robot.set_stablization(1, False)
            
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
#     hd = 0
    print "Roll"
#     s.robot.roll(100,hd,1,False)
#     sleep(5)
#         print "Head",hd
#         s.robot.set_back_led(255, False)
#         print "LED set"
    print "Turn"
#     s.robot.roll(0,180,1,False)
#         print "Switch off stabilization"
#         s.robot.set_stablization(0, False)
#         print "Dingy"
#     sleep(5)
#         print "Turn"
    print "Set"
#     s.robot.set_heading(0,False)
#         s.robot.set_raw_motor_values(1, 255, 2, 255, False)
#         s.robot.roll(0,180,1,False)
#         s.robot.set_heading(0, False)
#         s.robot.set_stablization(1, False)
#         s.robot.roll(0,hd,1,False)
#         sleep(10)
#         s.robot.set_back_led(0, False)
#         sleep(10)
#         hd = (hd+90)%360
#         print hd
#     print "Done"
#     print "Lets calibrate"
#     s.sphero_calibrate(45)
#     print "Calibratiuon Done"
#     sleep(2)
#     print "Second"
#     s.robot.roll(80,0,1,False) # Right
#     sleep(5)
#     print "Recalibrate"
#     s.sphero_calibrate(45)
#     print "Calibration Done"
#     sleep(2)
#     print "Third"
#     s.robot.roll(80,0,1,False) # Down
#     sleep(5)
#     print "Recalibrate"
#     s.sphero_calibrate(45)
#     print "Calibration Done"
#     sleep(2)
#     print "Fourth"
#     s.robot.roll(80,0,1,False) # Left
#     sleep(5)
#     print "Back to normal"
#     s.sphero_calibrate(45)
#     s.robot.roll(80,45,1,False)
#     print "Over"
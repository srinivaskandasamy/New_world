from __future__ import division
import sphero_driver
from math import pi, atan, sqrt
import threading
import time

class SpheroNode(threading.Thread):
    
    x=0
    y=0
    xg=0
    yg=0
    
    def __init__(self, default_update_rate=25.0):

        self.update_rate = default_update_rate
        self.sampling_divisor = int(400 / self.update_rate)
        self.sampling_divisor = 25
        self.is_connected = False
        self.robot = sphero_driver.Sphero()
        threading.Thread.__init__(self)
         
    def set_goal(self, x, y):
        self.xg = x
        self.yg = y
        print 'Going to [x:{0}],[y:{1}]'.format(x,y)

    def run(self):
        threadLock = threading.Lock()
        threadLock.acquire()
        self.set_control()
        threadLock.release()
        
    def set_control(self): 
  
        xint = 0
        yint = 0

        while 1:
            
            time.sleep(0.05)
            
            ed = self.calculate_error()
            hd = self.calculate_heading()
                
            if abs(ed) >= 10:             
    
                speed = int(max(0, min(100, 0.2 * ed + 0.02 * abs(xint) + 0.02 * abs(yint))))
                #speed = int(max(0, min(100, 0.2 * ed)))
                
                xint = max(-10, min(10, xint + (self.x - self.xg)))
                yint = max(-10, min(10, yint + (self.y - self.yg)))
                  
                self.robot.roll(speed, hd, 1, False)    
                
                print '{0} -> [x:{1}],[y:{2}],[e:{3}]'.format(self.name, self.x, self.y, ed)            

            else:
                
                self.robot.roll(0, 0, 1, False)   
           

    def start_robot(self, bdaddr=''):
        
        while not self.is_connected:                
            try:
                self.is_connected = self.robot.connect(bdaddr)
                self.name = bdaddr
            except:
                print "Failed to connect"                  
                     
        self.robot.set_filtered_data_strm(self.sampling_divisor, 1 , 0, True)
        self.robot.add_async_callback(sphero_driver.IDCODE['DATA_STRM'], self.parse_data_strm)
        self.robot.config_collision_detect(1, 45, 110, 45, 110, 100, False)
        self.robot.add_async_callback(sphero_driver.IDCODE['COLLISION'], self.parse_collision)
 
        self.robot.start()
        self.start()
        
    def parse_data_strm(self, data):
        if self.is_connected:
            self.x = data["ODOM_X"]
            self.y = data["ODOM_Y"]
        
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
    
    def set_color(self, msg):
        if self.is_connected:
            self.robot.set_rgb_led(int(msg[0]), int(msg[1]), int(msg[2]), 0, False)
            
    def calculate_heading(self):
            
        ye = self.yg - self.y
        xe = self.xg - self.x
        
        if ye == 0 and xe >= 0:
            
            return 90
        
        elif ye == 0 and xe < 0:
            
            return 270
        
        else:
            
            angle = int(atan(abs(xe / ye)) / pi * 180)                       
            if ye > 0 and xe >= 0:
                return angle
            if ye < 0 and xe >= 0:
                return 90 + angle
            if ye < 0 and xe < 0:
                return 180 + angle
            if ye > 0 and xe < 0:
                return 270 + angle           
        
    def calculate_error(self):
        return sqrt((self.xg - self.x) ** 2 + (self.yg - self.y) ** 2)
   
if __name__ == '__main__':
    
    print "Starting..."
    
    A = SpheroNode()
    B = SpheroNode()
    C = SpheroNode()
    
    A.start_robot("68:86:E7:00:DB:54") #ORB
    B.start_robot("68:86:E7:01:2C:FB") #PGY
    #C.start_robot("68:86:E7:00:56:7D") #WYP
    
    print "Running!"
    
    A.set_color([255,0,0])
    B.set_color([0,255,0])
    #C.set_color([255,0,0])
    
    time.sleep(5)
    
    A.set_goal(300,0)
    B.set_goal(300,0)  
    #C.set_goal(150,0)
    
    time.sleep(5)
    
    A.set_goal(-300,0)
    B.set_goal(-300,0)  
    #C.set_goal(150,0)
    
    time.sleep(5)
    
    A.set_goal(300,0)
    B.set_goal(300,0)  
    #C.set_goal(150,0)
    
    time.sleep(5)
    
    A.set_goal(-300,0)
    B.set_goal(-300,0)  
    #C.set_goal(150,0)
    
    time.sleep(5)
    
    A.set_goal(300,0)
    B.set_goal(300,0)  
    #C.set_goal(150,0)
    
    time.sleep(5)
    
    A.set_goal(-300,0)
    B.set_goal(-300,0)  
    #C.set_goal(150,0)
    
    time.sleep(5)
    
    A.set_goal(0,0)
    B.set_goal(0,0)
    #C.set_goal(0,0)
    
    
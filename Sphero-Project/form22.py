from __future__ import division
import sphero_driver
from math import pi, atan, sqrt
from multiprocessing import Pool
import time

class SpheroNode(object):
    
    x=0
    y=0
    
    def __init__(self, default_update_rate=50.0):

        self.update_rate = default_update_rate
        self.sampling_divisor = int(400 / self.update_rate)
        self.sampling_divisor = 25
        self.is_connected = False
        self.robot = sphero_driver.Sphero()
        
    def set_control(self, xd, yd): 
                
        time.sleep(2)

        print "Setting heading"                
        hd = self.calculate_heading(xd, yd)         
        ed = self.calculate_error(xd, yd)
        
        print 'Starting control loop at [x:{0}],[y:{1}]'.format(self.x, self.y)
         
        xint = 0
        yint = 0
         
        while abs(ed) >= 10: 
              
            time.sleep(0.02)
        
#             speed = int(max(0, min(130, 0.05 * ed + 0.02 * abs(xint) + 0.02 * abs(yint))))
            speed = 125
            xint = min(150, xint + (self.x - xd))
            yint = min(150, yint + (self.y - yd))
              
            self.robot.roll(speed, hd, 1, False)
              
            ed = self.calculate_error(xd, yd)
            hd = self.calculate_heading(xd, yd)
              
            print '[x:{0}],[y:{1}],[h:{2}],[e:{3}],[s:{4}]'.format(self.x, self.y, hd, ed, speed)
      
        print "Done"
        self.robot.roll(0, 0, 1, False)
        
    def start(self, bdaddr=''):
        
        while not self.is_connected:                
            try:
                self.is_connected = self.robot.connect(bdaddr)
            except:
                print "Failed to connect"                  
                     
        self.robot.set_filtered_data_strm(self.sampling_divisor, 1 , 0, True)
        self.robot.add_async_callback(sphero_driver.IDCODE['DATA_STRM'], self.parse_data_strm)
        self.robot.config_collision_detect(1, 45, 110, 45, 110, 100, False)
        self.robot.add_async_callback(sphero_driver.IDCODE['COLLISION'], self.parse_collision)
 
        self.robot.start()
        
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
            
    def calculate_heading(self, xd, yd):
            
        ye = yd - self.y
        xe = xd - self.x
        
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
        
    def calculate_error(self, xd, yd):
        return sqrt((xd - self.x) ** 2 + (yd - self.y) ** 2)  
    
if __name__ == '__main__':
    
    print "Starting..."
    
    A = SpheroNode()
    B = SpheroNode()
    
    A.start("68:86:E7:00:2C:EB")
    B.start("68:86:E7:00:57:06")
      
    A.set_color([255,0,0])
    B.set_color([0,255,0])
    
    p = Pool(processes=2)
    p.apply_async(A.set_control, args=(150,150))
    p.apply_async(B.set_control, args=(150,150))
    p.close()
    p.join()
    #runInParallel(A.set_control(50,50),B.set_control(50,50))
    #runInParallel(A.set_control(0,0),B.set_control(0,0))
  
    print "Over"
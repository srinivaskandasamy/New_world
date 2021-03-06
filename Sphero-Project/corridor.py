# Collision detection and data streaming
# Recalibration of Sphero after collision
# Recording collision data and abstracting correlations

import sphero_driver  # Main class file
import math
import numpy as np
# import matplotlib.pyplot as plt
# import csv

REQ=dict(CMD_ROLL = [0x02, 0x30])

class SpheroNode(object):
    
    def __init__(self, default_update_rate=50.0):

        self.update_rate = default_update_rate
        self.sampling_divisor = int(400 / self.update_rate)

        self.is_connected = False
        self.robot = sphero_driver.Sphero()
        self.cmd_heading = 0
        self.cmd_speed = 0
        self.power_state_msg = "No Battery Info"
        self.power_state = 0
        self.collipy = np.matrix([0, 0, 0, 0, 0])
        self.t = np.matrix([0])
        self.head = 0
        self.hd = 0
        
    def start(self):
        try:
            self.is_connected = self.robot.connect()
        except:
            print "Failed to connect Sphero"

        self.robot.set_filtered_data_strm(self.sampling_divisor, 1 , 0, True)
        self.robot.config_collision_detect(1, 45, 110, 45, 110, 100, False)
        self.robot.add_async_callback(sphero_driver.IDCODE['COLLISION'], self.parse_collision)

        self.robot.start()
        
    def parse_data_strm(self, data):
        if self.is_connected:
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
            
            print '[x:{0}],[y:{1}]'.format(self.ox, self.oy)
           
    def stop(self):    
        # tell the ball to stop moving before quiting
        self.robot.roll(int(0), int(0), 1, False)
        self.robot.shutdown = True
        self.is_connected = self.robot.disconnect()
        self.robot.join()
        
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
            
            
            if self.cx > 0 and self.cy > 0:
                print "Right"
                if self.caxis == 2: # this is y axis ; visualize by considering head on collision with the wall (max y) and sliding along wall (max x)
                    print "Y"
                    self.head = self.head - 130                #   self.head = self.head   -  (180 - 2 * math.atan(math.fabs(self.cx)/math.fabs(self.cy)))
                else:               # this is xaxis
                    print "X"
                    self.head = self.head - 80

            elif self.cx < 0 and self.cy > 0:
                print "Left"
                if self.caxis == 2: # this is y axis ; visualize by considering head on collision with the wall (max y) and sliding along wall (max x)
                    print "Y"
                    self.head = self.head + 130                    
                else:               # this is xaxis
                    print "X"
                    self.head = self.head + 80

            else: 
                print "Wrong side"
                
            
            if self.head > 359:
                self.head = self.head - 360
            elif self.head < 0:
                self.head = self.head + 360            
                    
            print '[cx:{0}], [cy:{1}], [cxm:{2}], [cym:{3}], [ax:{4}], [hd:{5}]'.format(self.cx, self.cy, self.cx_magnitude, self.cy_magnitude, self.caxis, self.head, self)
                
            self.collipy = np.vstack([self.collipy, np.matrix([self.cx, self.cy, self.caxis * 100, self.head, self.cspeed])])
                
            self.t = np.vstack([self.t,self.t[-1]+1])           

            self.robot.roll(150, self.head, 1, False)
            
    def cmd_vel(self, msg):
        if self.is_connected:
            self.cmd_heading = self.normalize_angle_positive(math.atan2(msg[0], msg[1])) * 180 / math.pi
            self.cmd_speed = math.sqrt(math.pow(msg[0], 2) + math.pow(msg[1], 2))
            self.robot.roll(int(self.cmd_speed), int(self.cmd_heading), 1, False)
    
    def set_color(self, msg):
        if self.is_connected:
            self.robot.set_rgb_led(int(msg[0]), int(msg[1]), int(msg[2]), 0, False)
    
    def roll(self, speed, heading, state, response):
            self.send(self.pack_cmd(REQ['CMD_ROLL'], [self.clamp(speed, 0, 255), (heading >> 8), (heading & 0xff), state]), response)
            
    def set_stabilization(self, msg):
        if self.is_connected:
            if not msg.data:
                self.robot.set_stablization(1, False)
            else:
                self.robot.set_stablization(0, False)
                
    def set_angular_velocity(self, msg):
        if self.is_connected:
            rate = int((msg.data * 180 / math.pi) / 0.784)
            self.robot.set_rotation_rate(rate, False)

    def configure_collision_detect(self, msg):
        pass

    def reconfigure(self, config, level):
        if self.is_connected:
            self.robot.set_rgb_led(int(config['red'] * 255), int(config['green'] * 255), int(config['blue'] * 255), 0, False)
        return config
    
# Main function
    
if __name__ == '__main__':
#     plt.ion()
#     fig = plt.figure()
#     ax = fig.add_subplot(111)
    print "Starting..."
    s = SpheroNode()
    s.start()
    print "Junga"
    s.robot.roll(150,0,1,False)
    print "Ready"    
    a=1
#     line, = ax.plot([],[],'r-')
#     line1,= ax.plot([],[],'bs')
#     ax.set_xlim([0,75])
#     ax.set_ylim([-200,200])
    
#     while a==1:
        # Plotting collision data
#         s.robot.roll(150, s.collipy[-1,3], 1, False)
#         line.set_xdata(s.t)
#         line.set_ydata(s.collipy[:,3])
#         line1.set_xdata(s.t)
#         line1.set_ydata(s.collipy[:,2])
#         fig.canvas.draw()
    print "Over"
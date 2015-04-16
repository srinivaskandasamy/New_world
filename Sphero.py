from __future__ import division
from math import cos, sin, pi, fabs
from numpy import arange
from Tkinter import *

WB = 2                            # RHP zero, wheel base
r = 10                            # Radius of the sphero robot

class Sphero(object):
    """ Creating a Sphero object for the sample maze construction. The steering angle is assumed to be small to make the 
assumption in calculations. The numerical calculations, as a result, are stable. In future, two other models like point mass 
and SO(3) will also be added if needed.
1. In the first model, we assumed a segway model which has the same inputs as the actual robot but the measurements are not 
available as position or orientation
2. In the second model, we assume a point mass model where the acceleration is the input to the robot as well as the measurements
in the form of change in acceleration.
3. The third model is a mix of both the above models, that represents an input-output model of the robot. The input to the 
system is the heading and velocity (segway) and the output is the collision in the form of change in acceleration. The segway 
model is simulated and the states are differentiatied to obtain V and A. With added noise in A, we integrate back to get the 
position of the robot. If collision is detected, we add this to the integrated data.

Created by Srinivas K."""
    def __init__(self,canvas):
        self.cursor_objects = []
        self.canvas = canvas
    
    def motion(self, pose, control, dt):  # Unicycle mkotion model
        control[1]=control[1]%360
        angle = ((control[1]+pose[2]*180/pi)%360)*pi/180
        return ([x+y for x,y in zip(pose,[control[0]*dt*cos(angle), 
                                          control[0]*dt*sin(angle), 
                                          control[0]*dt*sin(control[1]*pi/180)/WB])])
    
    def draw(self, pose):     # Draw the Sphero robot in the simulated scene
        if self.cursor_objects:
            map(self.canvas.delete, self.cursor_objects)
            self.cursor_objects = []
        
        xsp, ysp =[], []
        xsp.append(pose[0])
        ysp.append(pose[1])
        for i in arange(0,2*pi,0.01):      # List of spherical points
            angle = (i + pose[2])%(2*pi)
            xsp.append(pose[0] + r * cos(angle))
            ysp.append(pose[1] + r * sin(angle))
        
        f = (xsp,ysp)
        
        for i in range(len(f[0])):
            self.cursor_objects.append(self.canvas.create_line(f[0][(i)%len(f[0])],f[1][(i)%len(f[0])],
                                                         f[0][(i+1)%len(f[0])],f[1][(i+1)%len(f[0])]))
    # Constraint set in a GUI window
    def sphero_constraint(self,origin,ww,ws,wl):
        a = origin[0]
        b = origin[1]
        
        # Creating boundary set (inclusing the wall width and the radius of the ball)
        bc = []  # In the form of a^Tx<=b where x=[X Y] and bc = [X Y b]
        bc.append([1, 0, a+ww+r]) # Boundary 1
        bc.append([0, 1, b+ww+r]) # Boundary 2
        bc.append([-1, 0, (a-ww-r+ws[0])*-1]) # Boundary 3
        bc.append([0, -1, (b-ww-r+ws[1])*-1]) # Boundary 4
        
        # Creating constraint of wall 1
        wc1=[]
        wc1.append([-1, 0, (a-ww/2-r+ws[0]/4)*-1])
        wc1.append([0, 1, b+ww+ws[1]*3/4+r])
        wc1.append([1, 0, a+ww/2+r+ws[0]/4])
        
        # Creating constraint of wall 2
        wc2=[]
        wc2.append([-1, 0, (a-ww/2-r+ws[0]/2)*-1])
        wc2.append([0, -1, (b+ww-r+ws[1]/4)*-1])
        wc2.append([1, 0, a+ww/2+r+ws[0]/2])
        
        # Creating constraint of wall 3
        wc3=[]
        wc3.append([-1, 0, (a-ww/2-r+ws[0]*3/4)*-1])
        wc3.append([0, 1, b+ww+r+ws[1]*3/4])
        wc3.append([1, 0, a+ww/2+r+ws[0]*3/4])
        
        return (bc,wc1,wc2,wc3)
    
    def check_collision(self,constraints,pose):
        print pose
        for i in range(len(constraints)):
            count = 0
            for j in range(len(constraints[i])): # bc needs count=4
                print "Now",constraints[i][j]
                ##################
                # Checking Boundary collision
                if i==0: # It is a boundary
                    if constraints[i][j][0]!=0: # Constraints with x-axis conditions
                        if constraints[i][j][0]*pose[0] <= constraints[i][j][2]: # Violation of constraints
                            print "xPose",pose[0]
                            print "Constraint",constraints[i][j][2]
                            print "Boundary Collision"
                            if constraints[i][j][0] >= 0: # Constraint type 1 (180 degrees)
                                print "First"
                                print constraints[i][j]
                                pose[2] = (pose[2] - 2*(pi-pose[2]))%(2*pi)
                            else:                         # Constraint type 2 (0 degrees)
                                print "Second"
                                print constraints[i][j]
                                pose[2] = (pose[2] + 2*(pose[2]))%(2*pi)
                    
                    if constraints[i][j][1]!=0: # Constraints with y-axis conditions
                        if constraints[i][j][1]*pose[1] <= constraints[i][j][2]: # Violation of constraints
                            print "yPose",pose[1]
                            print "Constraint",constraints[i][j][2]
                            print "Boundary Collision"
                            if constraints[i][j][1] >= 0: # Constraint type 1 (270 degrees)
                                print "Third"
                                print constraints[i][j]
                                pose[2] = (pose[2] - 2*(3*pi/2-pose[2]))%(2*pi)
                            else:
                                print "Fourth"
                                print constraints[i][j]
                                pose[2] = (pose[2] - 2*(pi/2-pose[2]))%(2*pi)
                
                ###################
                # Checking inner wall collision
                else:
                    if constraints[i][j][0]!=0: # Constraints with x-axis conditions
                        if constraints[i][j][0]*pose[0] <= constraints[i][j][2]: # Violation of constraints
                            count += 1
                            print "Count",count
                            print "Constraint",constraints[i][j]
                            if fabs(fabs(pose[0]) - fabs(constraints[i][j][2])) < 10:
                                bingo = j
                                print "Bingo",bingo
                
                    if constraints[i][j][1]!=0: # Constraints with y-axis conditions
                        if constraints[i][j][1]*pose[1] <= constraints[i][j][2]: # Violation of constraints
                            count += 1
                            print count
                            print "Constraint",constraints[i][j]
                            if fabs(fabs(pose[1]) - fabs(constraints[i][j][2])) < 10:
                                bingo = -1*j
                                print "Bingo",bingo
                                
                    if count == len(constraints[i]):
                        print "Inner wall collision detected"
                        print "Constraints",constraints[i]
                        if bingo > 0:
                            print "Bingap",constraints[i][bingo]
                            if constraints[i][bingo][0] <= 0: # Constraint type 1 (180 degrees)
                                print "Pose",pose[2]
                                pose[2] = (pose[2] - 2*(pi-pose[2]))%(2*pi)
                            else:                         # Constraint type 2 (0 degrees)
                                print "POsee",pose[2]
                                pose[2] = (pose[2] + 2*(pose[2]))%(2*pi)
                        else:
                            print "Bingan",constraints[i][-1*bingo]
                            if constraints[i][-1*bingo][1] <= 0: # Constraint type 1 (270 degrees)
                                print "Posy",pose[2]
                                pose[2] = (pose[2] - 2*(3*pi/2-pose[2]))%(2*pi)
                            else:
                                print "Posr",pose[2]
                                pose[2] = (pose[2] - 2*(pi/2-pose[2]))%(2*pi)
        return pose
        
if __name__=="__main__":
    s = Sphero()
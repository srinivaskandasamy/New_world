from Tkinter import *
from math import sin, cos, pi

world_size = (225,225)

# Class Sphero - creating a robot object
# Class Sphero_trajectory - creatng the trajectory of the robot
# Class world - contains the size of the world - walls, entry and exit points, and robot as a child object
# Class slider_moved - creating the slider in the artifiical world
# Class 1D_Gaussian - import random.random()

master = Tk()

w = Canvas(master,width=200,height=100)

w.pack()

w.create_line(0,0,200,100)
w.create_line(0,100,200,0,fill="red",dash=(10,10))

w.create_rectangle(50,25,150,75, fill="blue")

mainloop()
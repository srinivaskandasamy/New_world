#!/usr/bin/python
# Default test function
# mobile_ws-->src-->sphero_ros-->sphero_driver-->scripts
# mobile_ws-->src-->sphero_ros-->sphero_node--nodes for sphero ros example file
import time
import sphero_driver
import sys

sphero = Sphero()

sphero.start()
time.sleep(0.5)
print "Red color"
sphero.set_rgb_led(255,0,0,0,False)
time.sleep(0.5)
print "Green color"
sphero.set_rgb_led(0,255,0,0,False)
time.sleep(0.5)
print "Blue color"
sphero.set_rgb_led(0,0,255,0,False)
print "ok"
# time.sleep(1)
sphero.roll(100,0,1,False)
# time.sleep(1)
# sphero.roll(80,90,1,False)
# time.sleep(1)
# sphero.roll(80,180,1,False)
# time.sleep(1)
# sphero.roll(80,270,1,False)
# time.sleep(1)
print "ok."
sphero.disconnect()
print "ok..."
sys.exit(1)
#!/usr/bin/python

"""
This python file just runs all of the terminal commands needed to run the project. It just saves time not having to manually type in these commands every time you want to run the project.

At the moment it only works for the example project, as the project later develops this script might be updated if the other people in the team decide to use this.

This is a first version, next I might work on getting a seperate terminal open to run each robot in order for it to be easy to see the positions of each robot. At the moment, since only 1 terminal is used, all of the output is put in it (which of course makes it messy)

To run the script, simply open up a terminal and type: python automateprojectscript.py

Author: ttho618
"""

import os
from subprocess import Popen, PIPE, signal
from os.path import join


findRoscorePro = Popen("pgrep roscore", stdout=PIPE, shell=True)
killroscorePro = Popen("kill "+findRoscorePro.communicate()[0], shell=True)
 
# The world file to look for
lookfor = "myworld.world"
# I assume that the project on your computer is located within the home directory 
for root, dirs, files in os.walk('/home', topdown=True):
    #print "searching", root
    if '.local' in dirs:
  	dirs.remove('.local')
    if 'catkin_ws' in dirs: # If the project is within this directory, then you need to change this to rosbuild_ws
	dirs.remove('catkin_ws')
    if lookfor in files:
        print "found: %s" % join(root, lookfor)
        worldfile = join(root, lookfor)
        print worldfile


# This would need to be changed if your project is named something different
rosmakePro= Popen('rosmake se306Project',shell=True)
rosmakePro.communicate() # Waits until rosmake has finished

core = Popen('roscore',shell=True) 
stagePro = Popen('rosrun stage stageros %s' %worldfile,shell=True)

# These below lines would need to be changed to fit what you are wanting to run.
runNode= Popen('rosrun se306Project R0',shell=True)
runNode= Popen('rosrun se306Project R1',shell=True)

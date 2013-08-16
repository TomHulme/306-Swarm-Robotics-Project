#!/usr/bin/python

"""
This python file just runs all of the terminal commands needed to run the project. It just saves time not having to manually type in these commands every time you want to run the project.

At the moment it only works for the example project, as the project later develops this script might be updated if the other people in the team decide to use this.

Currently a new terminal is opened for each node calling rosrun, this makes it easier to see which messages belong to which nodes. 

To run the script, simply open up a terminal and type: python automateprojectscript.py

Author: ttho618
"""

import os
import time
from subprocess import Popen, PIPE, signal
from os.path import join
import shlex

filename= "world/myworld.world"
num_sheep= 2
num_fields = 1
field_X= 4
field_Y= 4
num_grass = num_fields*(field_X-1)*(field_Y-1)

worldGenPro = Popen("python world/worldGenerator.py "+filename + " " + str(num_sheep) + " " + str(num_fields) + " " + str(field_X) + " " + str(field_Y),shell=True)
worldGenPro.communicate()

cleanupCMakeFile= Popen("sed -i /rosbuild_add_executable/d se306Project/CMakeLists.txt",shell=True)

# Range goes from 2 to sheep+2 because nodes 0,1 are farmer,sheepdog.
'''
for i in range(2, (num_sheep+2)):
	copyr0Pro = Popen("cp se306Project/src/R0.cpp se306Project/src/R"+str(i)+".cpp", stdout=PIPE, shell=True)
	copyr0Pro.communicate();
	modifyRPro= Popen("find . -name R"+str(i)+".cpp -exec sed -i \"s/RobotNode0/RobotNode"+str(i)+"/g\" {} \;",shell=True)
	modifyRPro.communicate();
	modifyRPro= Popen("find . -name R"+str(i)+".cpp -exec sed -i \"s/robot_0/robot_"+str(i)+"/g\" {} \;",shell=True)
	modifyRPro.communicate();
	modifyRPro= Popen("find . -name R"+str(i)+".cpp -exec sed -i \"s/Robot 0/Robot "+str(i)+"/g\" {} \;",shell=True)
	modifyRPro.communicate();
	addToCMakeFile= Popen("echo \"rosbuild_add_executable(R"+str(i)+" src/R"+str(i)+".cpp)\" >> se306Project/CMakeLists.txt",shell=True)
'''
# Counting grass
for i in range(0, num_grass):
	'''
	copyr0Pro = Popen("cp se306Project/src/Grass0.cpp se306Project/src/Grass"+str(i)+".cpp", stdout=PIPE, shell=True)
	copyr0Pro.communicate();
	modifyRPro= Popen("find . -name Grass"+str(i)+".cpp -exec sed -i \"s/Grass0/Grass"+str(i)+"/g\" {} \;",shell=True)
	modifyRPro.communicate();
	modifyRPro= Popen("find . -name Grass"+str(i)+".cpp -exec sed -i \"s/robot_0/robot_"+str(i+(num_sheep+2))+"/g\" {} \;",shell=True)
	modifyRPro.communicate();
	'''
	addToCMakeFile= Popen("echo \"rosbuild_add_executable(Grass"+str(i)+" src/Grass"+str(i)+".cpp)\" >> se306Project/CMakeLists.txt",shell=True)

#Add farmer, sheepdog, listener
addToCMakeFile= Popen("echo \"rosbuild_add_executable(farmer src/farmer.cpp)\" >> se306Project/CMakeLists.txt",shell=True)
addToCMakeFile= Popen("echo \"rosbuild_add_executable(sheepdog src/sheepdog.cpp)\" >> se306Project/CMakeLists.txt",shell=True)

addToCMakeFile= Popen("echo \"rosbuild_add_executable(SheepNode src/SheepNode.cpp)\" >> se306Project/CMakeLists.txt",shell=True)
addToCMakeFile= Popen("echo \"rosbuild_add_executable(SheepMove src/SheepMove.cpp)\" >> se306Project/CMakeLists.txt",shell=True)
#addToCMakeFile= Popen("echo \"rosbuild_add_executable(field src/field.cpp)\" >> se306Project/CMakeLists.txt",shell=True)
#addToCMakeFile= Popen("echo \"rosbuild_add_executable(grass src/grass.cpp)\" >> se306Project/CMakeLists.txt",shell=True)

addToCMakeFile= Popen("echo \"rosbuild_add_executable(listener src/listener.cpp)\" >> se306Project/CMakeLists.txt",shell=True)

	
# This checks if there is a running roscore process and if there is, it gets killed
findRoscorePro = Popen("pgrep roscore", stdout=PIPE, shell=True)
killroscorePro = Popen("kill "+findRoscorePro.communicate()[0], shell=True)
 
# The world file to look for
lookfor = "myworld.world"
# I assume that the project on your computer is located within the home directory 
for root, dirs, files in os.walk('./', topdown=True):
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

# Reset original grass and sheep back to 0
modifyRPro= Popen("find . -name R0.cpp -exec sed -i \"s/robot_"+str(i)+"/robot_0/g\" {} \;",shell=True)
modifyRPro= Popen("find . -name Grass0.cpp -exec sed -i \"s/robot_"+str(i+(num_sheep+2))+"/robot_0/g\" {} \;",shell=True)

core = Popen('roscore',shell=True) 
time.sleep(3)
stagePro = Popen('rosrun stage stageros %s' %worldfile,shell=True)

# These below lines would need to be changed to fit what you are wanting to run.
# Start from 2 because nodes 0 and 1 are for farmer and sheepdog
#resetParams=Popen("rosparam set sheep/number 0",shell=True)
#print "Resetting sheep/number parameter"
#resetParams=Popen("rosparam set sheepMove/number 0",shell=True)
#print "Resetting sheepMove/number parameter"
# Note: no longer needs to start at 2
for i in range(num_sheep):
	###to merge
	'''
	#	#runNode= Popen("rosrun se306Project R"+str(i) + " __name:=sheep"+str(i),shell=True)
	#	runNode= Popen("rosrun se306Project sheep __name:=sheep"+str(i),shell=True)
	##runNode= Popen('rosrun se306Project R1',shell=True)
	##runNode= Popen('rosrun se306Project R2',shell=True)
	#runNode= Popen('rosrun se306Project farmer',shell=True)
	#runNode= Popen('rosrun se306Project sheepdog',shell=True)
	##runNode= Popen('rosrun se306Project field',shell=True)
	##will need to do stuff with this later
	##runNode= Popen('rosrun se306Project grass',shell=True)
	'''
	print "creating sheep",i
	#runNode= Popen("rosrun se306Project SheepNode __name:=sheep"+str(i) + " _sheepNum:="+str(i),shell=True)
	#runNode= Popen("rosrun se306Project SheepMove __name:=sheepMove"+str(i) + " _sheepNum:="+str(i),shell=True)
	runNode= Popen(shlex.split("""x-terminal-emulator -e 'bash -c "rosrun se306Project SheepNode __name:=sheep{0} _sheepNum:={0}"'""".format(str(i))),stdout=PIPE)
	runNode= Popen(shlex.split("""x-terminal-emulator -e 'bash -c "rosrun se306Project SheepMove __name:=sheepMove{0} _sheepNum:={0} _robotNum:={1}"'""".format(str(i), str(i+2))),stdout=PIPE)
	#runNode= Popen("rosrun se306Project R"+str(i),shell=True)
#runNode= Popen('rosrun se306Project R1',shell=True)
#runNode= Popen('rosrun se306Project R2',shell=True)

runNode= Popen(shlex.split("""x-terminal-emulator -e 'bash -c "rosrun se306Project farmer"'"""),stdout=PIPE)
#runNode= Popen('rosrun se306Project farmer',shell=True)

runNode= Popen(shlex.split("""x-terminal-emulator -e 'bash -c "rosrun se306Project sheepdog"'"""),stdout=PIPE)
#runNode= Popen('rosrun se306Project sheepdog',shell=True)

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
num_sheep= 4
num_fields = 4
field_X= 6
field_Y= 4
num_grass_field = (field_X-1)*(field_Y-1)
num_grass = num_fields*num_grass_field

total_sheep = num_sheep * num_fields


# Runs the worldGenerator file with the specified arguments, waits until it has finished before continuing on
worldGenPro = Popen("python world/worldGenerator.py "+filename + " " + str(num_sheep) + " " + str(num_fields) + " " + str(field_X) + " " + str(field_Y),shell=True)
worldGenPro.communicate()

# Removes all lines in the CMakeLists.txt file that were added from previous executions of the script. Clean slate of the file.
cleanupCMakeFile= Popen("sed -i /rosbuild_add_executable/d se306Project/CMakeLists.txt",shell=True)
cleanupCMakeFile.wait()


# Range goes from 3 to sheep+3 because nodes 0,1,2 are farmer,sheepdog,truck.
#for i in range(3, (num_sheep+3)):
'''
	copyr0Pro = Popen("cp se306Project/src/R0.cpp se306Project/src/R"+str(i)+".cpp", stdout=PIPE, shell=True)
	copyr0Pro.communicate();
	# Updates all references in the .cpp file to the new .cpp file and node it represents
	modifyRPro= Popen("find . -name R"+str(i)+".cpp -exec sed -i \"s/RobotNode0/RobotNode"+str(i)+"/g\" {} \;",shell=True)
	modifyRPro.communicate();
	modifyRPro= Popen("find . -name R"+str(i)+".cpp -exec sed -i \"s/robot_0/robot_"+str(i)+"/g\" {} \;",shell=True)
	modifyRPro.communicate();
	modifyRPro= Popen("find . -name R"+str(i)+".cpp -exec sed -i \"s/Robot 0/Robot "+str(i)+"/g\" {} \;",shell=True)
	modifyRPro.communicate();
	# Adds files to CMakeLists.txt that are to be compiled during rosmake. Only the needed files are to be added
	addToCMakeFile= Popen("echo \"rosbuild_add_executable(R"+str(i)+" src/R"+str(i)+".cpp)\" >> se306Project/CMakeLists.txt",shell=True)
'''
# Counting grass
'''
for i in range(0, num_grass):
	
	copyr0Pro = Popen("cp se306Project/src/Grass0.cpp se306Project/src/Grass"+str(i)+".cpp", stdout=PIPE, shell=True)
	copyr0Pro.communicate();
	modifyRPro= Popen("find . -name Grass"+str(i)+".cpp -exec sed -i \"s/Grass0/Grass"+str(i)+"/g\" {} \;",shell=True)
	modifyRPro.communicate();
	modifyRPro= Popen("find . -name Grass"+str(i)+".cpp -exec sed -i \"s/robot_0/robot_"+str(i+(num_sheep+3))+"/g\" {} \;",shell=True)
	modifyRPro.communicate();
	addToCMakeFile= Popen("echo \"rosbuild_add_executable(Grass"+str(i)+" src/Grass"+str(i)+".cpp)\" >> se306Project/CMakeLists.txt",shell=True)
'''
#Add farmer, sheepdog, listener
addToCMakeFile= Popen("echo \"rosbuild_add_executable(farmer src/farmerNode.cpp src/farmer.cpp)\" >> se306Project/CMakeLists.txt",shell=True)
addToCMakeFile.wait()
addToCMakeFile= Popen("echo \"rosbuild_add_executable(sheepdog src/sheepdogNode.cpp src/sheepdog.cpp)\" >> se306Project/CMakeLists.txt",shell=True)
addToCMakeFile.wait()
addToCMakeFile= Popen("echo \"rosbuild_add_executable(truck src/truck.cpp)\" >> se306Project/CMakeLists.txt",shell=True)
addToCMakeFile.wait()
addToCMakeFile= Popen("echo \"rosbuild_add_executable(laserScanToPointCloud src/LaserScanToPointCloud.cpp)\" >> se306Project/CMakeLists.txt",shell=True)
addToCMakeFile.wait()

#Add the field node into CMakeLists
addToCMakeFile= Popen("echo \"rosbuild_add_executable(GrassNode src/GrassNode.cpp)\" >> se306Project/CMakeLists.txt",shell=True)
addToCMakeFile.wait()
addToCMakeFile= Popen("echo \"rosbuild_add_executable(FieldNode src/FieldNode.cpp src/Field.cpp)\" >> se306Project/CMakeLists.txt",shell=True)
addToCMakeFile.wait()

#Add the sheep
addToCMakeFile= Popen("echo \"rosbuild_add_executable(SheepNode src/SheepNode.cpp)\" >> se306Project/CMakeLists.txt",shell=True)
addToCMakeFile.wait()
addToCMakeFile= Popen("echo \"rosbuild_add_executable(SheepMove src/SheepMove.cpp)\" >> se306Project/CMakeLists.txt",shell=True)
addToCMakeFile.wait()

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
        #print "found: %s" % join(root, lookfor)
        worldfile = join(root, lookfor)
        #print worldfile


# This would need to be changed if your project is named something different
rosmakePro= Popen('rosmake se306Project',shell=True)
rosmakePro.communicate() # Waits until rosmake has finished

core = Popen('roscore',shell=True) 

time.sleep(5)

# Runs rosrun stage with the found .world file

stagePro = Popen('rosrun stage stageros %s' %worldfile,shell=True)

# These below lines would need to be changed to fit what you are wanting to run.
# Start from 3 because nodes 0, 1 and 2 are for farmer, sheepdog and truck respectively
#Run Sheep nodes (SheepNode, SheepMove)
commandString = "gnome-terminal "
for i in range(total_sheep):
	print "creating sheep",i
	commandString += """\\--tab -e 'bash -c \"rosrun se306Project SheepNode __name:=sheep{0} _sheepNum:={0} _fieldX:={1} _fieldY:={2} \"' --title='SheepNode {0}' """.format(str(i), str(field_X), str(field_Y))
	commandString += """\\--tab -e 'bash -c \"rosrun se306Project SheepMove __name:=sheepMove{0} _sheepNum:={0} _robotNum:={1}\"' --title='SheepMove {0}' """.format(str(i), str(i+3))
runNode= Popen(shlex.split(commandString),stdout=PIPE)

#Run Grass nodes
###TODO: something like the following code
#for i in range(num_grass):
#	print "creating grass",i
#	runNode= Popen(shlex.split("""gnome-terminal -e 'bash -c "rosrun se306Project Grass __name:=grass{0} _grassNum:={0}"'""".format(str(i))),stdout=PIPE)

#Run Field Node(s)
#runNode= Popen(shlex.split("""gnome-terminal -e 'bash -c "rosrun se306Project field"'"""),stdout=PIPE)
###TODO: something like the following code
#for i in range(num_fields):
#	print "creating field",i
#	runNode= Popen(shlex.split("""gnome-terminal -e 'bash -c "rosrun se306Project Field __name:=field{0} _fieldNum:={0} _xPos:={1} _yPos:={2}"'""".format(str(i),someX, someY)),stdout=PIPE)
#Run Field nodes
commandString = "gnome-terminal "
for i in range(num_fields):
	print "creating field",i
	runNode = Popen(shlex.split("""gnome-terminal -e 'bash -c "rosrun se306Project FieldNode {0} {1} {2}"' --title='Field {0}'""".format(str(i), str(field_X), str(field_Y))), stdout=PIPE)

	commandString += """\\--tab -e 'bash -c \"rosrun se306Project FieldNode {0} {1} {2}\"' --title='Field {0}' """.format(str(i), str(field_X), str(field_Y))
	for j in range(num_grass_field):
		commandString += """\\--tab -e 'bash -c \"rosrun se306Project GrassNode {0} {1} {2}\"' --title='Grass {0}' """.format(str((i*num_grass_field)+j+1), str(((i*num_grass_field)+j+1)+3+total_sheep), str(i))

runNode= Popen(shlex.split(commandString),stdout=PIPE)


#Run Farmer Node
runNode= Popen(shlex.split("""gnome-terminal -e 'bash -c "rosrun se306Project farmer"' --title='Farmer'"""),stdout=PIPE)
#Run Sheepdog Node
runNode= Popen(shlex.split("""gnome-terminal -e 'bash -c "rosrun se306Project sheepdog {0}"' --title='Sheepdog'""".format(str(total_sheep))),stdout=PIPE)
#Run Truck Node
runNode= Popen(shlex.split("""gnome-terminal -e 'bash -c "rosrun se306Project truck"' --title='Truck'"""),stdout=PIPE)

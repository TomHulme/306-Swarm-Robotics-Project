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
 
# This would need to be changed if your project is named something different
rosmakePro= Popen('rosmake se306Project',shell=True)
rosmakePro.communicate() # Waits until rosmake has finished

core = Popen('roscore',shell=True) 

time.sleep(5)

# Runs rosrun stage with the .world file
stagePro = Popen('rosrun stage stageros world/myworld.world',shell=True)

# These below lines would need to be changed to fit what you are wanting to run.
#Run Sheep nodes (SheepNode, SheepMove) in a single terminal window with tabs separating the different sheep
commandString = "gnome-terminal "
for i in range(total_sheep):
	print "creating sheep",i
	commandString += """\\--tab -e 'bash -c \"rosrun se306Project SheepNode __name:=sheep{0} _sheepNum:={0} _fieldX:={1} _fieldY:={2} \"' --title='SheepNode {0}' """.format(str(i), str(field_X), str(field_Y))
	commandString += """\\--tab -e 'bash -c \"rosrun se306Project SheepMove __name:=sheepMove{0} _sheepNum:={0} _robotNum:={1}\"' --title='SheepMove {0}' """.format(str(i), str(i+3))
runNode= Popen(shlex.split(commandString),stdout=PIPE)

commandString = "gnome-terminal "
for i in range(num_fields):
	print "creating field",i
	#runNode = Popen(shlex.split("""gnome-terminal -e 'bash -c "rosrun se306Project FieldNode {0} {1} {2}"' --title='Field {0}'""".format(str(i), str(field_X), str(field_Y))), stdout=PIPE)
	commandString += """\\--tab -e 'bash -c \"rosrun se306Project FieldNode {0} {1} {2}\"' --title='Field {0}' """.format(str(i), str(field_X), str(field_Y))
	for j in range(num_grass_field):
		commandString += """\\--tab -e 'bash -c \"rosrun se306Project GrassNode {0} {1} {2}\"' --title='Grass {0}' """.format(str((i*num_grass_field)+j), str(((i*num_grass_field)+j+1)+3+total_sheep), str(i))
runNode= Popen(shlex.split(commandString),stdout=PIPE)

#Run Farmer Node in a new gnome terminal with a changed title window
runNode= Popen(shlex.split("""gnome-terminal -e 'bash -c "rosrun se306Project farmer"' --title='Farmer'"""),stdout=PIPE)
#Run Sheepdog Node
runNode= Popen(shlex.split("""gnome-terminal -e 'bash -c "rosrun se306Project sheepdog {0}"' --title='Sheepdog'""".format(str(total_sheep))),stdout=PIPE)
#Run Truck Node
runNode= Popen(shlex.split("""gnome-terminal -e 'bash -c "rosrun se306Project truck"' --title='Truck'"""),stdout=PIPE)

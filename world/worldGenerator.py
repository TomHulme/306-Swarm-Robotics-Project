import sys
from random import randrange
#worldGenerator v1.0
#
#This python file generates a world file based on certain input. It requires a file name,
#number of sheep you wish to simulate, the number of fields within the world, the size of
#the field in the X direction and the size of the field in the Y direction.
#
#The file that gets generated will be in the same folder that this file is.
#As of v1.0, it is only capable of generating one field that is full of grass.

#Looking at Theo's auto launch file, it looks rather easy to incorporate this into it. That
#launch file may need to be run with the same inputs as this requires though.

def JoinString(temp_string1, temp_string2, temp_string3, temp_string4):
    return temp_string1 + temp_string2 + temp_string3 + temp_string4

def WriteFile(fileName, numSheep, numFields, fieldX, fieldY):
    fo = open(fileName, "w+")
    
    #Strings to write to world file
    laser_def = """define mylaser ranger
(
    sensor
    (
    range [0.0 30.0]
    fov 180
    samples 180
    )
    color "black"
    size [ 0.05 0.05 0.1]

)
"""

    truck_laser_def = """define mytrucklaser ranger
(
    sensor
    (
    range [100.0 100.0]
    fov 1
    samples 1
    )
    color "black"
    size [ 0.05 0.05 0.1]

)
"""

    robot_sheep_def = """define sheepRobot position
(
  size [0.35 0.35 0.25]
  drive "diff"
  mylaser(pose [ 0.050 0.000 -0.1 0.000 ])
  localization_origin [0 0 0 0]
)
"""

    robot_NOSIZE_def = """define farmRobot position
(
  drive "diff"
  mylaser(pose [ 0.050 0.000 -0.1 90 ])
  localization_origin [0 0 0 0]
)
"""

    robot_truck_def = """define truckRobot position
(
  drive "diff"
  mytrucklaser(pose [ 0.050 0.000 0.050 90 ])
  localization_origin [0 0 0 0]
)
"""

    grass_def = """define grass position
(
    size [0.75 0.75 0.1]
    mylaser(pose [ 0.050 0.000 0 0.000 ])
    localization_origin [0 0 0 0]
)
"""

    floorplan_def = """define floorplan model
(
    color "brown"

    boundary 1

    ranger_return 1
)
"""
    
    myblock_def = """define my_block model
(
    size [0.5 0.5 0.5]
    gui_nose 0
)
"""
    
    window_def = """window
(
    size [ 800.000 600.000 ]
    scale 30
)
"""
    #Write laser definition to file
    fo.write(laser_def)
    #Write truck laser definition to file
    fo.write(truck_laser_def)
    #Write robot position definition to file
    fo.write(robot_sheep_def)
    #Write robot_truck position definition to file
    fo.write(robot_truck_def)
    #Write robot position WITHOUT SIZE definition to file
    fo.write(robot_NOSIZE_def)
    #Write grass definition to file
    fo.write(grass_def)
    #Write floorplan definition to file
    fo.write(floorplan_def)
    #Write myblock definition to file
    fo.write(myblock_def)
    #Write resolution to file
    fo.write("resolution 0.02\n")
    #Write sim_interval to file 
    fo.write("interval_sim 100\n")
    #Write window definition to file
    fo.write(window_def)
    
    #Write floorplans to file
    prevX = 0
    for i in range(int(numFields)):
        currentX = float(prevX) + float(fieldX)/2
        currentY = float(fieldY)/2
        
        ts1 = 'floorplan (name "field' + str(i) +'" '
        ts2 = 'bitmap "swarm_world.pgm" '
        ts3 = 'size [' + str(fieldX) + ' ' + str(fieldY) + ' 0.5] '
        ts4 = 'pose [' + str(currentX) + ' ' + str(currentY) + ' 0 90.000])\n'
        
        fo.write(JoinString(ts1, ts2, ts3, ts4))
        prevX = currentX + float(fieldX)/2   
    
    #Construct farmer definition
    ts1 = 'farmRobot ('
    ts2 = 'pose [5 2 0.125 0] size [0.5 0.5 0.1] '
    ts3 = 'name "farmer" '
    ts4 = 'color "dark blue" bitmap "FARMER.bmp")\n'

        
    fo.write(JoinString(ts1, ts2, ts3, ts4))
     
    #Construct sheepdog definition
    ts1 = 'farmRobot ('
    ts2 = 'pose [5 5 0.125 0] size [1 0.5 0.1] '
    ts3 = 'name "sheepdog" '
    ts4 = 'color "brown" bitmap "SHEEPDOG.bmp")\n'
        
    fo.write(JoinString(ts1, ts2, ts3, ts4))
    
    #Construct truck definition
    ts1 = 'truckRobot ('
    ts2 = 'pose [2 -3 0.125 0] size [3 3 0.1] '
    ts3 = 'name "truck" '
    ts4 = 'color "blue" bitmap "TRUCK.bmp")\n'
        
    fo.write(JoinString(ts1, ts2, ts3, ts4))

    count = 0    
    #Write sheep robots to file
    for i in range((int(numFields))):
            field_offset = i * int(fieldX)
            row = 1
            col = 1 + field_offset
            for j in range(int(numSheep)):
    
                #Construct sheep definition
                ts1 = 'sheepRobot ('
                ts2 = 'pose [' + str(col) + ' ' + str(row)  +  ' 0.125 '+str(randrange(-180,180))+'] '
                ts3 = 'name "sheepMove' + str(count) + '" '
                ts4 = 'color "white" bitmap "SHEEP.BMP")\n'

                fo.write(JoinString(ts1, ts2, ts3, ts4))
                count += 1
        
                col += 1
                print(col)
                if col == int(fieldX):
                    col = 1
                    row += 1
    
    count = 0
    #Write grass robots to file
    for i in range((int(numFields))):
        field_offset = i * int(fieldX)
        for j in range((int(fieldX))-1):
            for k in range((int(fieldY))-1):
                #Construct grass definition
                ts1 = 'grass ('
                ts2 = 'pose [' + str(j+1+field_offset) + ' ' + str(k+1) + ' 0 0] '
                ts3 = 'name "grass' + str(count) + '" '
                ts4 = 'color "green" bitmap "grass.pgm")\n'
                
                fo.write(JoinString(ts1, ts2, ts3, ts4)) 
            
                count += 1
    
    #Close opened file
    fo.close()

if not (len(sys.argv) == 6):
    print("Please input file name and number of robots you wish to have.")
    print("Usage:",sys.argv[0],"<file name> <number of sheep> <number of fields> <fieldX> <fieldY>")
    print("Output: world file <file name> with <number of sheep> sheep and <number of fields> fields, <fieldX> by <fieldY> large filled with grass.")
else:
    fileName = sys.argv[1]
    numSheep = sys.argv[2]
    numFields = sys.argv[3]
    fieldX = sys.argv[4]
    fieldY = sys.argv[5]
    
    field_string = ""
    if int(numFields) == 1:
        field_string = "field."
    else:
        field_string = "fields."
    
    print("Generating world file with "+numSheep+" sheep and "+numFields+" "+field_string)
    WriteFile(fileName, numSheep, numFields, fieldX, fieldY)


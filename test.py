import os
from subprocess import Popen, PIPE, signal
from os.path import join

filename= "world/myworld.world"
num_sheep= 5
num_fields = 4
field_X= 5
field_Y=5

worldGenPro = Popen("python world/worldGenerator.py "+filename + " " + str(num_sheep) + " " + str(num_fields) + " " + str(field_X) + " " + str(field_Y), shell=True)
worldGenPro.communicate()

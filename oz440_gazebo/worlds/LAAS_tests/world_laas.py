#!/usr/bin/env python
# coding: utf8

import random
import sys
import json

#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# World configuration
arguments = sys.argv
json_file = arguments[1]

json_data=open(json_file)
data = json.load(json_data)

name =  data["Nom"]
filename = name+".world"

# World file creation
file = open(filename, "w")

file.write("<sdf version='1.6'> \n \
\t <world name='default'> \n \
\t \t <light name='sun' type='directional'> \n \
\t \t \t <cast_shadows>1</cast_shadows> \n \
\t \t \t <pose frame=''>0 0 10 0 -0 0</pose> \n \
\t \t \t <diffuse>0.5 0.5 0.5 1</diffuse> \n \
\t \t \t <specular>0.1 0.1 0.1 1</specular> \n \
\t \t \t <attenuation> \n \
\t \t \t \t <range>1000</range> \n \
\t \t \t \t <constant>0.9</constant> \n \
\t \t \t \t <linear>0.01</linear> \n \
\t \t \t \t <quadratic>0.001</quadratic> \n \
\t \t \t </attenuation> \n \
\t \t \t <direction>0.9 0.5 -1</direction> \n \
\t \t </light>\n \n \
\t \t <scene> \n \
\t \t \t <background>130 200 255 1 </background>\n \
\t \t \t <ambient>130 140 140 1</ambient>\n \
\t \t </scene>\n \n ")

#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# Ground texture
texture = data["Sol"]

if texture == "sand" :
    file.write("\t \t <include> \n \t \t \t <uri>model://sand_plane</uri> \n \t \t </include>\n \n")

elif texture == "dirt" :
    file.write("\t \t <include> \n \t \t \t <uri>model://dirt_plane</uri> \n \t \t </include>\n \n")

elif texture == "grass" :
    file.write("\t \t <include> \n \t \t \t <uri>model://grass_plane</uri> \n \t \t </include>\n \n")

elif texture == "heightmap" :
    file.write("\t \t <include> \n \t \t \t <uri>model://heightmap</uri> \n \t \t </include>\n \n")

else :
    print("The texture input is no correct \n")

#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# Culture configuration

# Type of vegetable
V = data["Type culture"]
interplant = data["Interplant"]
F = 1/interplant

# Number of rows
N = 2

# Length of the rows
L = data["Longueur rangee"]

# Width of the rows
W = data["Largeur rangee"]

# World file writting
# Red Sticks
file.write("\t \t <population name=\"sticks\"> \n \
\t \t \t <model name=\"Red_stick\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>0</static>\n \
\t \t \t \t \t <uri>model://Red_stick</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t<pose>%f %f 0.1 0 0 0</pose>\n \
\t \t \t<distribution> \n \
\t \t \t \t <type>grid</type> \n \
\t \t \t \t <rows>%d</rows>\n \
\t \t \t \t <cols>%d</cols>\n \
\t \t \t \t <step>%f %f 0</step>\n \
\t \t \t </distribution>\n \
\t \t </population>\n \n" \
%( 1.5 + L/2.0, -(N-1)*W/2 + W/2, N, 2, L+1.0, W))

# ****************************************************************************** #

# Rows of vegetable

# Ecart légumes
E = data["Ecart"] # par rapport à ligne principale

row = 1
Random_quinconce = 0

while row < N+1:

    num = 1
    while num < L * F + 1 :
        Ran = ( random.random() - 0.5 )*2 #entre -1 et 1
        Random_angle = ( random.random() - 0.5 )*2 #entre -1 et 1

        file.write("\t \t<include> \n \
\t \t \t <uri>model://%s</uri> \n \
\t \t \t <pose>%f %f 0.15 0 0 %f</pose> \n \
\t \t \t <static>0</static> \n \
\t \t </include> \n \n" \
%(V, 1.5 + (num - 1.0) / F + Random_quinconce, W/2 - (row - 1.0) * W + Ran * E / 100.0 , Random_angle * 180.0))
        num += 1
    row += 1
    Random_quinconce = ( random.random() - 0.5 ) * interplant

#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# Other objects

# ****************************************************************************** #

# Rocks
R = data["Cailloux"]

file.write("\t \t<population name=\"Rocks\">\n \
\t \t \t <model name=\"Rocks\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>1</static>\n \
\t \t \t \t \t <uri>model://Big_rock</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>%f 0 0 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>%d %f 0.02</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>%d</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>random</type>\n \
\t \t \t </distribution>\n \
\t \t </population> \n \n"\
%( 1.5 + L / 2.0, L + 2, W - 0.05 , ( L + 2 ) * R ))

# ****************************************************************************** #

# Grass
G = data["Herbe"]

file.write("\t \t<population name=\"Grass\">\n \
\t \t \t <model name=\"Grass\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>1</static>\n \
\t \t \t \t \t <uri>model://Grass</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>%f 0 0.02 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>%d %f 0.02</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>%d</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>random</type>\n \
\t \t \t </distribution>\n \
\t \t </population> \n \n"\
%( 1.5 + L / 2.0, L + 2, W + 1.0 , ( L + 2 ) * G ))

# ****************************************************************************** #

#Trees
file.write("\t \t <population name=\"Trees1\"> \n \
\t \t \t <model name=\"Tree\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>1</static>\n \
\t \t \t \t \t <uri>model://Tree</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>17 0 -0.6 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>5 20 3</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>5</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>random</type>\n \
\t \t \t </distribution>\n \
\t \t </population>\n \
\t \t \n \
\t \t <population name=\"Trees2\">\n \
\t \t \t <model name=\"Tree\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>1</static>\n \
\t \t \t \t \t <uri>model://Tree</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>0 -17 -0.6 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>30 6 3</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>5</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>random</type>\n \
\t \t \t </distribution>\n \
\t \t </population>\n \
\n \
\t \t <population name=\"Trees3\">\n \
\t \t \t <model name=\"Tree\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>1</static>\n \
\t \t \t \t \t <uri>model://Tree</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>0 15 -0.6 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>30 6 3</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>7</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>random</type>\n \
\t \t \t </distribution>\n \
\t \t \t </population>\n \
\n \
\t \t <population name=\"Trees4\">\n \
\t \t \t <model name=\"Tree\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>1</static>\n \
\t \t \t \t \t <uri>model://Tree</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>-15 0 -0.6 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>10 30 3</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>8</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>random</type>\n \
\t \t \t </distribution>\n \
\t \t </population>\n \n")


#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# Closing the file
namelaunch = name+".launch"
print "\nYour world is ready ! You can launch it using : \nroslaunch oz440_gazebo",namelaunch,"\n"

file.write("\t </world> \n \
</sdf>")
file.close()

#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# Creating the launch file

launchname = "../../launch/LAAS_tests/"+name+".launch"

launchfile = open(launchname, "w")

launchfile.write("<launch> \n \n \
\t <param name=\"naio01_server_port\" type=\"int\" value=\"5555\" />\n \n \
\t <node name=\"Core\" pkg=\"oz440_gazebo\" type=\"Core\"/>\n \n \
\t <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched --> \n \
\t <include file=\"$(find gazebo_ros)/launch/empty_world.launch\">\n \
\t \t <arg name=\"world_name\" value=\"$(find oz440_gazebo)/worlds/LAAS_tests/%s\"/> \n \
\t \t <arg name=\"gui\" value=\"false\" /> \n \
\n </include> \n \n \
\t <param name=\"robot_description\" command=\"$(find xacro)/xacro '$(find oz440_description)/urdf/oz440.xacro'\" />\n \n \
\t <!-- Spawn a robot into Gazebo --> \n \
\t <node name=\"spawn_urdf\" pkg=\"gazebo_ros\" type=\"spawn_model\" output=\"screen\" args=\"-param robot_description -b -urdf -x 0 -y 0 -z 0.4 -model oz440\" /> \n \
\t <!-- ros_control launch file --> \n \
\t <include file=\"$(find oz440_control)/launch/oz440_control.launch\"/>\n \
</launch>\n " \
%(filename))

launchfile.close()

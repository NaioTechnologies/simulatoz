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

print filename

GFX = 0

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
\t \t \t <background>150 200 255 1 </background>\n \
\t \t \t <ambient>90 100 100 1</ambient>\n \
\t \t </scene>\n \n \
\t \t <include>\n \
\t \t \t <uri>model://Sign</uri>\n \
\t \t \t <pose>12 1 0 0 0 0</pose>\n \
\t \t \t <static>1</static>\n \
\t \t </include>\n \n \
\t \t <include>\n \
\t \t \t <uri>model://Sign</uri>\n \
\t \t \t <pose>-4 1 0 0 0 0</pose>\n \
\t \t \t <static>1</static>\n \
\t \t </include>\n \n")


#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# Ground texture
texture = data["Sol"]

if texture == "sand" :
    print("You chose a sand ground \n")
    file.write("\t \t <include> \n \t \t \t <uri>model://sand_plane</uri> \n \t \t </include>\n \n")

elif texture == "dirt" :
    print("You chose a dirt ground \n")
    file.write("\t \t <include> \n \t \t \t <uri>model://dirt_plane</uri> \n \t \t </include>\n \n")

elif texture == "grass" :
    print("You chose a grass ground \n")
    file.write("\t \t <include> \n \t \t \t <uri>model://grass_plane</uri> \n \t \t </include>\n \n")

elif texture == "heightmap" :
    print("You chose a heightmap \n")
    file.write("\t \t <include> \n \t \t \t <uri>model://heightmap</uri> \n \t \t </include>\n \n")

else :
    print("The texture input is no correct \n")

# Mound
M = data["Butte"]

#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# Culture configuration

   : "70",
  "herbe": "true",
  "Cailloux" : "false"

# Type of vegetable
veggie = data["Type culture"]
interplant = data["Interplant"]
F = 100/interplant

# Number of rows
N = data["Nombre de rangées"]

# Length of the rows
L = data["Longueur rangée"]

# Width of the rows
W = data["Largeur rangée"]

# World file writting
# Red Sticks
if M == true:

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
%( 1.5 + L/2.0, -(N-1)*w/2 + w/2, N, 2, L+1.0, w))

else:

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
%( 1.5 + L/2.0, -(N-1)*w/2 + w/2, N, 2, L+1.0, w))


# Mounds

if M == true:

    file.write("\t \t <population name=\"Mound\"> \n \
\t \t \t <model name=\"Mound\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>1</static>\n \
\t \t \t \t \t <uri>model://Mound</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t<pose>%f %f -0.245 0 0 0</pose>\n \
\t \t \t<distribution> \n \
\t \t \t \t <type>grid</type> \n \
\t \t \t \t <rows>%d</rows>\n \
\t \t \t \t <cols>%d</cols>\n \
\t \t \t \t <step>%f %f 0</step>\n \
\t \t \t </distribution>\n \
\t \t </population>\n \n" \
%(1.5 + L/2.0, -(N-1)*w/2 + w/2, N, 2, L+1.0, w))

    file.write("\t \t<population name=\"Mound_cyl\">\n \
\t \t \t <model name=\"Mound_cyl\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>1</static>\n \
\t \t \t \t \t <uri>model://Mound_cyl</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>%f %f -0.2 0 0 0</pose>\n \
\t \t \t<distribution> \n \
\t \t \t \t <type>grid</type> \n \
\t \t \t \t <rows>%d</rows>\n \
\t \t \t \t <cols>%d</cols>\n \
\t \t \t \t <step>%f %f 0</step>\n \
\t \t \t </distribution>\n \
\t \t </population> \n \n" \
%(1.5 + L/2.0, -(N-1)*w/2 + w/2, N, L*F, interplant/100.0, w))


# Rows of vegetable

if M == true:

    file.write("\t \t<population name=\"Vegetable\">\n \
\t \t \t <model name=\"%s\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>0</static>\n \
\t \t \t \t \t <uri>model://%s</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>%f %f 0.2 0 0 0</pose>\n \
\t \t \t<distribution> \n \
\t \t \t \t <type>grid</type> \n \
\t \t \t \t <rows>%d</rows>\n \
\t \t \t \t <cols>%d</cols>\n \
\t \t \t \t <step>%f %f 0</step>\n \
\t \t \t </distribution>\n \
\t \t </population> \n \n" \
%(V, V, 1.5 + L/2.0, -(N-1)*w/2 + w/2, N, L*F, 1/float(F), w))
#JE SUIS LAAAAAAAAAAA
else :

    quinconce = data["quinconce"]

    row = 1
    while row < N+1:
        num = 1
        while num < L * F + 1 :
            Ran = ( random.random() - 0.5 )/ 20
            file.write("\t \t<include> \n \
\t \t \t <uri>model://%s</uri> \n \
\t \t \t <pose>%f %f 0.15 0 0 %f</pose> \n \
\t \t \t <static>0</static> \n \
\t \t </include> \n \n" \
%(V, 1.2 + (num - 1.0) / F, w/2 - (row - 1.0) * w + Ran, Ran * 180.0))
            num += 1
        row += 1



#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# Other objects

print("Other objects \n")

# Rocks
while again == 1:
    again = 0

    while 1 :
        try :
            R = int(raw_input("Do you want rocks in the ground ? Press : \n - 0 for no \n - 1 for yes \n - 2 if you want big rocks on the left side of the rows \n To confirm, press enter. \n"))
            break
        except ValueError:
            print "This is not a number !"

    if R == 2 :
        print("There will be rocks and big rocks on the ground \n")
        i = 1
        while i < N:
            file.write("\t \t<population name=\"Rocks%d\">\n \
\t \t \t <model name=\"Rock\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>1</static>\n \
\t \t \t \t \t <uri>model://Rock</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>%f %f -0.01 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>%d %f 0.01</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>%d</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>random</type>\n \
\t \t \t </distribution>\n \
\t \t </population> \n \n"\
%(i, 1.0 + L/2.0, -(float(i)-0.5)*w + w/2, L+4, w-0.1, L*6.0))

            file.write("\t \t<population name=\"Big_rocks%d\">\n \
\t \t \t <model name=\"Big_rock\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>1</static>\n \
\t \t \t \t \t <uri>model://Big_rock</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>%f %f -0.01 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>%d 0.2 0.001</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>%d</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>random</type>\n \
\t \t \t </distribution>\n \
\t \t </population> \n \n"\
%(i, 1.0 + L/2.0, -(float(i)-1)*w + w/2-0.15, L, L*3.0))
            i += 1

    elif R == 1 :
        print("There will be rocks on the ground \n")
        i = 1
        while i < N:
            file.write("\t \t<population name=\"Rocks%d\">\n \
\t \t \t <model name=\"Rock\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>1</static>\n \
\t \t \t \t \t <uri>model://Rock</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>%f %f -0.01 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>%d %f 0.01</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>%d</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>random</type>\n \
\t \t \t </distribution>\n \
\t \t </population> \n \n"\
%(i, 1.0 + L/2.0, -(float(i)-0.5)*w + w/2, L+4, w-0.1, L*6.0))
            i += 1

    elif R == 0 :
        print("There wont be any rock \n")

    else :
        print("You did not enter a correct answer \n")
        again = 1
again = 1


# Grass
while again == 1:
    again = 0

    while 1 :
        try :
            G = int(raw_input("Do you want grass on the ground ? Press : \n - 0 for no \n - 1 for yes \n To confirm, press enter. \n"))
            break
        except ValueError:
            print "This is not a number !"

    if G == 1 :
        print("There will be grass on the ground \n")
        file.write("\t \t<population name=\"Grass\">\n \
\t \t \t <model name=\"Grass\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>1</static>\n \
\t \t \t \t \t <uri>model://Grass</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>%f %f -0.01 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>%d %f 0.05</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>%d</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>random</type>\n \
\t \t \t </distribution>\n \
\t \t </population> \n \n"\
%(1.0 + L/2.0, -w*(N-1)/2 + w/2, L+1, w*(N-1), L*N*3))

        i = 1
        while i < N+1:
            file.write("\t \t<population name=\"Grass%d\">\n \
\t \t \t <model name=\"Grass\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>1</static>\n \
\t \t \t \t \t <uri>model://Grass</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>%f %f -0.02 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>%f 0.2 0.05</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>%d</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>random</type>\n \
\t \t \t </distribution>\n \
\t \t </population> \n \n"\
%(i, 1.1 + L/2.0, -(i-1)*w + w/2 , L-0.2, L * 6.0))

            i += 1

    elif G == 0 :
        print("There wont be any grass \n")
    else :
        print("You did not enter a correct answer \n")
        again = 1
again = 1

# Trees
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
print "\nYour world is ready ! You can launch it using : \n roslaunch oz440_gazebo",namelaunch,"\n"

file.write("\t </world> \n \
</sdf>")
file.close()

#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# Creating the launch file

launchname = "../launch/"+name+".launch"

launchfile = open(launchname, "w")

if GFX == 1 :
    launchfile.write("<launch> \n \n \
\t <param name=\"naio01_server_port\" type=\"int\" value=\"5555\" />\n \n \
\t <node name=\"Core\" pkg=\"oz440_gazebo\" type=\"Core\"/>\n \n \
\t <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched --> \n \
\t <include file=\"$(find gazebo_ros)/launch/empty_world.launch\">\n \
\t \t <arg name=\"world_name\" value=\"$(find oz440_gazebo)/worlds/%s\"/> \n \
\n </include> \n \n \
\t <param name=\"robot_description\" command=\"$(find xacro)/xacro '$(find oz440_description)/urdf/oz440.xacro'\" />\n \n \
\t <!-- Spawn a robot into Gazebo --> \n \
\t <node name=\"spawn_urdf\" pkg=\"gazebo_ros\" type=\"spawn_model\" output=\"screen\" args=\"-param robot_description -b -urdf -x 0 -y 0 -z 0.4 -model oz440\" /> \n \
\t <!-- ros_control launch file --> \n \
\t <include file=\"$(find oz440_control)/launch/oz440_control.launch\"/>\n \
</launch>\n " \
%(filename))

else :
    launchfile.write("<launch> \n \n \
\t <param name=\"naio01_server_port\" type=\"int\" value=\"5555\" />\n \n \
\t <node name=\"Core\" pkg=\"oz440_gazebo\" type=\"Core\"/>\n \n \
\t <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched --> \n \
\t <include file=\"$(find gazebo_ros)/launch/empty_world.launch\">\n \
\t \t <arg name=\"world_name\" value=\"$(find oz440_gazebo)/worlds/%s\"/> \n \
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

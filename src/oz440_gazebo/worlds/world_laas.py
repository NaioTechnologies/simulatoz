#!/usr/bin/env python
# coding: utf8

import random
import sys
import json
from math import *

# {
#     "Nom": "LEEK_70_30_CURVE_6",
#     "Longueur rangee" : 6,
#
#     "Sol" : "dirt",
#     "Type culture" : "Leek",
#     "Interplant" : 0.30,
#     "Largeur rangee" : 0.70,
#     "Herbe": 0,
#     "Cailloux" : 0,
#     "Ecart" : 0,
#     "curve" : true
# }

#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# World configuration
arguments = sys.argv
json_file = arguments[1]

json_data = open(json_file)
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
\t \t </scene>\n \n \
\t \t <include>\n \
\t \t \t <pose>5 0 10 0 1.57 1.57</pose>\n \
\t \t \t <uri>model://Camera</uri>\n \
\t \t</include>\n \n")

#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

H = 0
Height = 0.0

# Ground texture
texture = data["Sol"]

if texture == "sand" :
    file.write("\t \t <include> \n \t \t \t <uri>model://sand_plane</uri> \n \t \t </include>\n \n")

elif texture == "dirt" :
    file.write("\t \t <include> \n \t \t \t <uri>model://dirt_plane</uri> \n \t \t </include>\n \n")

elif texture == "grass" :
    file.write("\t \t <include> \n \t \t \t <uri>model://grass_plane</uri> \n \t \t </include>\n \n")

elif texture == "heightmap" :
    H = 1
    file.write("\t \t <include> \n \t \t \t <uri>model://heightmap</uri> \n \t \t </include>\n \n")
    Height = 1.0

else :
    print("The texture input is no correct \n")

#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# Culture configuration

curve = data["Courbe"]

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

if curve :
    file.write("\t \t<include> \n \
\t \t \t <uri>model://Red_stick</uri> \n \
\t \t \t <pose>1.0 0.35 %f 0 0 0</pose> \n \
\t \t \t <static>%d</static> \n \
\t \t </include> \n \n \
\t \t<include> \n \
\t \t \t <uri>model://Red_stick</uri> \n \
\t \t \t <pose>1.0 -0.35 %f 0 0 0</pose> \n \
\t \t \t <static>%d</static> \n \
\t \t </include> \n \n" %( 0.1 + Height, H, 0.12 + Height, H))

else :
    file.write("\t \t <population name=\"sticks\"> \n \
\t \t \t <model name=\"Red_stick\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>%d</static>\n \
\t \t \t \t \t <uri>model://Red_stick</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t<pose>%f %f %f 0 0 0</pose>\n \
\t \t \t<distribution> \n \
\t \t \t \t <type>grid</type> \n \
\t \t \t \t <rows>%d</rows>\n \
\t \t \t \t <cols>%d</cols>\n \
\t \t \t \t <step>%f %f 0</step>\n \
\t \t \t </distribution>\n \
\t \t </population>\n \n" \
%( H, 1.5 + L/2.0, -(N-1)*W/2 + W/2, 0.1 + Height, N, 2, L+1.0, W))

# ****************************************************************************** #

# Rows of vegetable

# Ecart légumes
E = data["Ecart"] # par rapport à ligne principale

if curve :
    alpha_int = interplant / L
    alpha_ext = interplant / ( L + W )

    angle_fin = 1.58

    beta = 0.0

    #row interieur
    while beta < angle_fin :
        file.write("\t \t<include> \n \
\t \t \t <uri>model://%s</uri> \n \
\t \t \t <pose>%f %f %f 0 0 0</pose> \n \
\t \t \t <static>%d</static> \n \
\t \t </include> \n \n" \
         %(V, L * sin(beta) + 1.5 , - W/2 - ( 1 - cos(beta)) * L, 0.05+Height, H ))
        beta += alpha_int

    file.write("\t \t<include> \n \
\t \t \t <uri>model://Red_stick</uri> \n \
\t \t \t <pose>%f %f %f 0 0 0</pose> \n \
\t \t \t <static>%d</static> \n \
\t \t </include> \n \n" \
    %( L * sin(beta) + 1.5 , - W/2 - ( 1 - cos(beta)) * L - 0.5, 0.1+Height, H))

    beta = 0.0
    #row exterieur
    while beta < angle_fin :
        file.write("\t \t<include> \n \
\t \t \t <uri>model://%s</uri> \n \
\t \t \t <pose>%f %f %f 0 0 0</pose> \n \
\t \t \t <static>%d</static> \n \
\t \t </include> \n \n" \
         %(V, ( L + W ) * sin(beta) + 1.5 ,  W/2 - ( 1 - cos(beta)) * ( L + W ), 0.05+Height, H ))
        beta += alpha_ext

    file.write("\t \t<include> \n \
\t \t \t <uri>model://Red_stick</uri> \n \
\t \t \t <pose>%f %f %f 0 0 0</pose> \n \
\t \t \t <static>%d</static> \n \
\t \t </include> \n \n" \
     %( ( L + W ) * sin(beta) + 1.5 , W/2 - ( 1 - cos(beta)) * ( L + W ) - 0.5, 0.1+Height, H))

else:
    row = 1
    Random_quinconce = 0

    while row < 3:
        num = 1
        while num < L * F + 1 :
            Ran = ( random.random() - 0.5 )*2 #entre -1 et 1
            Random_angle = ( random.random() - 0.5 )*2 #entre -1 et 1

            file.write("\t \t<include> \n \
\t \t \t <uri>model://%s</uri> \n \
\t \t \t <pose>%f %f %f 0 0 %f</pose> \n \
\t \t \t <static>%d</static> \n \
\t \t </include> \n \n" \
            %(V, 1.5 + (num - 1.0) / F + Random_quinconce, W/2 - (row - 1.0) * W + Ran * E / 100.0 , 0.05+Height, Random_angle * 180.0, H))
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
\t \t \t \t \t <uri>model://Rock</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>%f 0 %f 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>%d %f 0.02</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>%d</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>random</type>\n \
\t \t \t </distribution>\n \
\t \t </population> \n \n" \
           %( 1.5 + L / 2.0, Height, L + 2, W - 0.05 , ( L + 2 ) * R ))

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
\t \t \t <pose>%f -3 %f 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>%d 6 0.02</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>%d</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>random</type>\n \
\t \t \t </distribution>\n \
\t \t </population> \n \n" \
           %( 1.5 + L / 2.0, 0.02 + Height, L + 2 , ( L + 2 ) * G * 6 ))

# ****************************************************************************** #

#Trees
file.write("\t \t <population name=\"Trees1\"> \n \
\t \t \t <model name=\"Tree\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>1</static>\n \
\t \t \t \t \t <uri>model://Tree</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>18 0 -0.6 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>4 20 3</size>\n \
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
\t \t \t <pose>0 -18 -0.6 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>30 4 3</size>\n \
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
print "\nYour world is ready ! You can launch it using : \nroslaunch oz440_gazebo oz.launch world:=",name,"\n"

file.write("\t </world> \n \
</sdf>")
file.close()

#!/usr/bin/env python
# coding: utf8


#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# World file creation

file = open("my_world.world", "w")
file.write("<sdf version='1.6'> \n \
\t <world name='default'> \n \
\t \t <light name='sun' type='directional'> \n \
\t \t \t <cast_shadows>1</cast_shadows> \n \
\t \t \t <pose frame=''>0 0 10 0 -0 0</pose> \n \
\t \t \t <diffuse>0.8 0.8 0.8 1</diffuse> \n \
\t \t \t <specular>0.1 0.1 0.1 1</specular> \n \
\t \t \t <attenuation> \n \
\t \t \t \t <range>1000</range> \n \
\t \t \t \t <constant>0.9</constant> \n \
\t \t \t \t <linear>0.01</linear> \n \
\t \t \t \t <quadratic>0.001</quadratic> \n \
\t \t \t </attenuation> \n \
\t \t \t <direction>-0.5 0.5 -1</direction> \n \
\t \t </light>\n \n")


#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# World configuration

print("New world Setup\nWorld configuration")
again = 1

# Ground texture
while again == 1:
    again = 0
    while 1 :
        try :
            texture = int(raw_input("Which ground texture do you want ? Press : \n - 1 for sand \n - 2 for dirt \n - 3 for grass \n - 4 if you do not want any texture \n To confirm, press enter. \n"))
            break
        except ValueError:
            print "This is not a number !"

    if texture == 1 :
        print("You chose a sand ground \n")
        file.write("\t \t <include> \n \t \t \t <uri>model://sand_plane</uri> \n \t \t </include>\n \n")

    elif texture == 2 :
        print("You chose a dirt ground \n")
        file.write("\t \t <include> \n \t \t \t <uri>model://dirt_plane</uri> \n \t \t </include>\n \n")

    elif texture == 3 :
        print("You chose a grass ground \n")
        file.write("\t \t <include> \n \t \t \t <uri>model://grass_plane</uri> \n \t \t </include>\n \n")

    elif texture == 4 :
        print("You chose not to have any texture \n")
        file.write("\t \t <include> \n \t \t \t <uri>model://ground_plane</uri> \n \t \t </include>\n \n")

    else :
        print("You did not enter a correct answer. Try again ! \n")
        again = 1
again = 1


#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# Culture configuration

print("Culture configuration\n")


# Type of vegetable
while again == 1:
    again = 0

    while 1 :
        try :
            veggie = int(raw_input("Which vegetable do you want ? Press : \n - 1 for leek \n - 2 for cabbage \n To confirm, press enter. \n"))
            break
        except ValueError:
            print "This is not a number !"

    if veggie == 1 :
        print("You chose leeks \n")
        V = "Leek"
        F = 4 # Number of vegetable per meter
    elif veggie == 2 :
        print("You chose cabbages\n")
        V = "Cabbage"
        F = 3 # Number of vegetable per meter
    else :
        print("You did not enter a correct answer \n")
        again = 1
again = 1


# Length of the rows
while again == 1:
    again = 0

    while 1 :
        try :
            L = int(raw_input("How long are the rows ? Press a number between 1 and 10 (in meters). To confirm, press enter. \n"))
            break
        except ValueError:
            print "This is not a number !"

    if L > 0 and L < 11 :
        print 'Your rows will be ', L, 'm long \n'
    else :
        print("You did not enter a correct answer \n")
        again = 1
again = 1


# Width of the rows
while again == 1:
    again = 0

    while 1 :
        try :
            W = int(raw_input("Chose the width of the rows ? Press a number between 65 and 150 (in cm). To confirm, press enter. \n"))
            break
        except ValueError:
            print "This is not a number !"

    if W > 64 and L < 151 :
        print 'Your rows will be ', W, 'cm wide \n'
        w = W/100.0
    else :
        print("You did not enter a correct answer \n")
        again = 1
again = 1


# Number of rows
while again == 1:
    again = 0

    while 1 :
        try :
            N = int(raw_input("Chose the number of rows ? Press a number between 1 and 6. To confirm, press enter. \n"))
            break
        except ValueError:
            print "This is not a number !"

    if N > 0 and N < 7 :
        print 'There will be ', N, ' rows \n'
    else :
        print("You did not enter a correct answer \n")
        again = 1
again = 1


# World file writting

# Red Sticks
file.write("\t \t <population name=\"sticks\"> \n \
\t \t \t <model name=\"Red_stick\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>0</static>\n \
\t \t \t \t \t <uri>model://Red_stick</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t<pose>1 %f 0.05 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>0.1 %f 0.01</size>\n \
\t \t \t </box>\n \
\t \t \t<model_count>%d</model_count>\n \
\t \t \t <distribution> \n \
\t \t \t \t <type>linear-y</type> \n \
\t \t \t </distribution>\n \
\t \t </population>\n \
\n \
\t \t <population name=\"sticks2\">\n \
\t \t \t <model name=\"Red_stick\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>0</static>\n \
\t \t \t \t \t <uri>model://Red_stick</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t<pose>%d %f 0.05 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>0.1 %f 0.01</size>\n \
\t \t \t </box>\n \
\t \t \t<model_count>%d</model_count>\n \
\t \t \t <distribution> \n \
\t \t \t \t <type>linear-y</type> \n \
\t \t \t </distribution>\n \
\t \t </population>\n \n" \
%(-w*N/2+w, N*w, N, L+1, -w*N/2+w, N*w, N))

# Rows of vegetable
i = 1
while i < N+1:
    file.write("\t \t<population name=\"Vegetable%d\">\n \
\t \t \t <model name=\"%s\">\n \
\t \t \t \t <include>\n \
\t \t \t \t \t <static>0</static>\n \
\t \t \t \t \t <uri>model://%s</uri>\n \
\t \t \t \t </include>\n \
\t \t \t </model>\n \
\t \t \t <pose>%f %f 0.2 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>%f 0.1 0.01</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>%d</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>linear-x</type>\n \
\t \t \t </distribution>\n \
\t \t </population> \n \n"\
%(i, V, V, 1.1 + L/2.0, -(i-1)*w, L-0.2, L*F))
    i += 1


#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# Other objects

print("Other objects \n")


# Rocks
while again == 1:
    again = 0

    while 1 :
        try :
            R = int(raw_input("Do you want rocks in the ground ? Press : \n - 0 for no \n - 1 for yes \n To confirm, press enter. \n"))
            break
        except ValueError:
            print "This is not a number !"

    if R == 1 :
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
\t \t \t <pose>%f %f -0.005 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>%d %f 0.01</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>%d</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>random</type>\n \
\t \t \t </distribution>\n \
\t \t </population> \n \n"\
%(i, 1.0 + L/2.0, -(float(i)-0.5)*w, L+4, w-0.1, 40))
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
\t \t \t <pose>%f %f -0.02 0 0 0</pose>\n \
\t \t \t <box>\n \
\t \t \t \t <size>%d %f 0.05</size>\n \
\t \t \t </box>\n \
\t \t \t <model_count>100</model_count>\n \
\t \t \t <distribution>\n \
\t \t \t \t <type>random</type>\n \
\t \t \t </distribution>\n \
\t \t </population> \n \n"\
%(1.0 + L/2.0, -w*N/2, L+1, w*N))

    elif G == 0 :
        print("There wont be any grass \n")
    else :
        print("You did not enter a correct answer \n")
        again = 1
again = 1


#||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||*||

# Closing the file

file.write("\t </world> \n \
</sdf>")
file.close()

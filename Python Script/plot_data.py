# File for plotting the data received by Bluetooth from the epuck2
# The basic structure of the thread and the counter is from the file
# plotImage.py from the course MICRO-315 used in TP4

import numpy as np
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
from serial import Serial
import struct
import string
import sys
import signal
import time
import math
from threading import Thread

size_int16 = 2
size_float = 4
outer_rim = 1500.0

data_file_path = "testdataNew.txt"

port_file = open("port.txt", 'r')
port_name = port_file.readline()

# class definitions
class Numbers:
    def __init__(self):
        self.nb_walls = 0
        self.nb_corners = 0
        self.nb_surf_lm = 0

    def reset(self):
        self.nb_walls = 0
        self.nb_corners = 0
        self.nb_surf_lm = 0

class Position:
    def __init__(self, x, y, z, phi, theta):
        self.x = x
        self.y = y
        self.z = z
        self.phi = phi
        self.theta = theta
    
    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.phi= 0.0
        self.theta = 0.0

    def print(self):
        print("Current position: x= " + str(self.x) + ", y=" + str(self.y) + ", z= " + str(self.z) + "")
        print("Current angles: phi= " + str(self.phi) + ",  theta= " + str(self.theta))

class Surf_Landmark:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def print(self):
        print("Surface landmark at: x= " + str(self.x) + ",\ty=" + str(self.y) +
                 ",\tz= " + str(self.z) + "\t")

class Corner:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def print(self):
        print("Corner at: x= " + str(self.x) + ",\ty=" + str(self.y) + "\t")

class Wall:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def print(self):
        print("Wall point at: x= " + str(self.x) + ",\ty=" + str(self.y) + "\t")

N = Numbers()
current_pos = Position(0.0, 0.0, 0.0, 0.0, 0.0)
corner_points = []
wall_points_active  = []
wall_points_passive  = []
landmarks_surf = []

#update the plot
def update_plot():
    if(reader_thd.need_to_update_plot()):
        global fig
        corners_xy         = [[],[]]
        walls_active_xy    = [[],[]]
        walls_passive_xy   = [[],[]]
        surf_lm_xyz        = [[],[],[]]
        for i in range(len(corner_points)):
            corners_xy[0].append(corner_points[i].x)
            corners_xy[1].append(corner_points[i].y)
        for i in range(len(wall_points_active)):
            walls_active_xy[0].append(wall_points_active[i].x)
            walls_active_xy[1].append(wall_points_active[i].y)
        for i in range(len(wall_points_passive)):
            walls_passive_xy[0].append(wall_points_passive[i].x)
            walls_passive_xy[1].append(wall_points_passive[i].y)
        for i in range(len(landmarks_surf)):
            surf_lm_xyz[0].append(landmarks_surf[i].x)
            surf_lm_xyz[1].append(landmarks_surf[i].y)
            surf_lm_xyz[2].append(landmarks_surf[i].z)

        # determine the range of the z coordinates of the surface landmarks
        maxZ = 0
        minZ = 0
        if len(landmarks_surf) > 0:
            maxZ = max(surf_lm_xyz[2])
            minZ = min(surf_lm_xyz[2])
        if maxZ == minZ:
            maxZ = maxZ+1

        for z in surf_lm_xyz[2]:
            z = 255*(z-minZ)/(maxZ-minZ)
        
        # refresh the plot with the new data
        fig.clear()
        # draw the landmark points
        plt.plot(corners_xy[0], corners_xy[1], 'black', linewidth=3)
        plt.plot(walls_active_xy[0],  walls_active_xy[1],  'bo', zorder=2)
        plt.plot(walls_passive_xy[0], walls_passive_xy[1], 'bo', zorder=1, alpha=0.02)
        plt.scatter(surf_lm_xyz[0], surf_lm_xyz[1], marker='.', c=surf_lm_xyz[2], cmap='viridis')

        # configure the colorbar
        cbar = plt.colorbar()
        cbar.ax.tick_params(labelsize=20)
        cbar.set_label('z-axis [mm]', size=30)

        # draw the current position
        plt.scatter(current_pos.x, current_pos.y, marker='*', linewidths=4, c='black', s=70, zorder=3)
        plt.arrow(current_pos.x, current_pos.y,
                    30*math.cos(current_pos.phi),
                    30*math.sin(current_pos.phi),
                    width=10,
                    facecolor='black',
                    zorder = 2)

        # configure the plot window
        plt.xlabel('x-axis [mm]', fontsize = 30)
        plt.xticks(fontsize=20)
        plt.ylabel('y-axis [mm]', fontsize = 30)
        plt.yticks(fontsize=20)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.draw()
        reader_thd.plot_updated()

# read the new data on the Bluetooth port
def update_data(port):
    readMessageSerial(port)
    reader_thd.tell_to_update_plot()

# handler when closing the window
def handle_close(evt):
    # write the landmarks and the current pos in the text file
    write_data_to_file()
    # stop the serial thread
    reader_thd.stop()
    timer.stop()

def read_START(char, state):
    if(state == 0):
        if(char == b'S'):
            state = 1
        else:
            state = 0
    elif(state == 1):
        if(char == b'T'):
            state = 2
        elif(char == b'S'):
            state = 1
        else:
            state = 0
    elif(state == 2):
        if(char == b'A'):
            state = 3
        elif(char == b'S'):
            state = 1
        else:
            state = 0
    elif(state == 3):
        if(char == b'R'):
            state = 4
        elif (char == b'S'):
            state = 1
        else:
            state = 0
    elif(state == 4):
        if(char == b'T'):
            state = 5
        elif (char == b'S'):
            state = 1
        else:
            state = 0
    return state

def reset_data():
    N.reset()
    current_pos.reset()
    corner_points.clear()
    wall_points_active.clear()
    wall_points_passive.clear()
    landmarks_surf.clear()
    return

#reads the data in uint8 from the serial
def readMessageSerial(port):
    state = 0
    while(state != 5):
        #reads 1 byte
        c1 = port.read(1)
        if(c1 == b''):
            print('Timout...')
            return
        state = read_START(c1, state)
    
    # check for a reset signal
    first = port.read(5)
    reset = struct.unpack('ccccc', first)
    if(reset[0]==b'R' and reset[1]==b'E' and reset[2]==b'S' and reset[3]==b'E' and reset[4]==b'T'):
        reset_data()
        print("Reset data")
        return

    # read the current position of the robot
    global current_pos
    temp = struct.unpack('fffff', first+port.read(15))
    if temp[0] < outer_rim and temp[0] > -outer_rim and temp[1] < outer_rim and temp[1] > -outer_rim:
        current_pos.x = temp[0]
        current_pos.y = temp[1]
        current_pos.z = temp[2]
    if temp[3] < 4.0  and temp[3] > -4.0 and temp[4] < 4.0  and temp[4] > -4.0:
        current_pos.phi = temp[3]
        current_pos.theta = temp[4]

    # read the number of corners
    global N
    temp = struct.unpack('H', port.read(size_int16))
    N.nb_corners = temp[0]

    # read the N.nb_corners corner points
    global corner_points
    corner_points.clear()
    for i in range(N.nb_corners):
        temp = struct.unpack('hh', port.read(2*size_int16))
        if temp[0] < outer_rim and temp[0] > -outer_rim and temp[1] < outer_rim and temp[1] > -outer_rim:
            new_corner = Corner(0,0)
            new_corner.x = temp[0]
            new_corner.y = temp[1]
            corner_points.append(new_corner)

    #read the number of wall points
    temp = struct.unpack('H', port.read(size_int16))
    N.nb_walls = temp[0]

    # read the N.nb_walls wall points
    global wall_points_active
    global wall_points_passive
    wall_points_active.clear()
    for i in range(N.nb_walls):
        temp = struct.unpack('hh', port.read(2*size_int16))
        if temp[0] < outer_rim and temp[0] > -outer_rim and temp[1] < outer_rim and temp[1] > -outer_rim:
            new_wall = Wall(0,0)
            new_wall.x = temp[0]
            new_wall.y = temp[1]
            wall_points_active.append(new_wall)
            if not new_wall in wall_points_passive:
                wall_points_passive.append(new_wall)


    #read the number of surface landmarks
    temp = struct.unpack('H', port.read(size_int16))
    N.nb_surf_lm = temp[0]

    # read the N.nb_surf_lm surface landmarks
    global landmarks_surf
    for i in range(N.nb_surf_lm):
        temp = struct.unpack('hhh', port.read(3*size_int16))
        if temp[0] < outer_rim and temp[0] > -outer_rim and temp[1] < outer_rim and temp[1] > -outer_rim:
            new_landmark = Surf_Landmark(0,0,0)
            new_landmark.x = temp[0]
            new_landmark.y = temp[1]
            new_landmark.z = temp[2]
            landmarks_surf.append(new_landmark)

    return

def write_data_to_file():
    data_file = open(data_file_path, "w+")

    # print the current position of the robot
    data_file.write(str(current_pos.x) + "\t" + str(current_pos.y) + "\t" + str(current_pos.z) + "\t" + str(current_pos.phi) + "\t" + str(current_pos.theta) + "\n")

    # print all the corners
    print("Wrote " + str(N.nb_corners) + " Corners to the file.\n")
    data_file.write(str(N.nb_corners) + "\n")
    for c in corner_points:
        data_file.write(str(c.x) + "\t" + str(c.y) + "\n")

    # print all the walls
    print("Wrote " + str(N.nb_walls) + " Walls to the file.\n")
    data_file.write(str(N.nb_walls) + "\n")
    for w in wall_points_passive:
        data_file.write(str(w.x) + "\t" + str(w.y) + "\n")
    for w in wall_points_active:
        data_file.write(str(w.x) + "\t" + str(w.y) + "\n")


    # print all the surface landmarks
    print("Wrote " + str(N.nb_surf_lm) + " surface landmarks to the file.\n")
    data_file.write(str(N.nb_surf_lm) + "\n")
    for l in landmarks_surf:
        data_file.write(str(l.x) + "\t" + str(l.y) + "\t" + str(l.z) + "\n")
    data_file.close()
    return

#thread used to control the communication part
class serial_thread(Thread):
    #init function called when the thread begins
    def __init__(self, port):
        Thread.__init__(self)
        self.alive = True
        self.need_to_update = False

        print('Connecting to port {}'.format(port))
        try:
            self.port = Serial(port, timeout=4)
        except:
            print('Cannot connect to the e-puck2')
            sys.exit(0)

    #function called after the init
    def run(self):
        while self.alive:
            update_data(self.port)

    #tell the plot need to be updated
    def tell_to_update_plot(self):
        self.need_to_update = True

    #tell the plot has been updated
    def plot_updated(self):
        self.need_to_update = False

    #tell if the plot need to be updated
    def need_to_update_plot(self):
        return self.need_to_update

    #clean exit of the thread if we need to stop it
    def stop(self):
        self.alive = False
        self.join()
        if(self.port.isOpen()):
            while(self.port.inWaiting() > 0):
                self.port.read(self.port.inWaiting())
                time.sleep(0.01)
            self.port.close()

#figure config
fig = plt.figure()
fig.canvas.set_window_title('Map')
fig.canvas.mpl_connect('close_event', handle_close)

#serial reader thread config and start
reader_thd = serial_thread(port_name)
reader_thd.start()

timer = fig.canvas.new_timer(interval=50)
timer.add_callback(update_plot)
timer.start()

plt.show()
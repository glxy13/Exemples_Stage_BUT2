# -*- coding: utf-8 -*-
"""
Created on Fri Feb 24 11:18:12 2023

@author: Asus
"""

# Creating a simulated drone object in Python:
# ```python
from tello_sim import Simulator
import time
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, MaxNLocator
import plotly.express as px
from itertools import zip_longest
import threading
import logging
import math
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# creer un repertoire Test avec un ID
# enregistrer les images du drones
import os
count = 6
newpath = "Resultats_TrouveObjet"
if not os.path.exists(newpath):
    os.makedirs(newpath)
newpath1 = "Resultats_TrouveObjet" + "/AvecObstacle_"+str(count)
if not os.path.exists(newpath1):
    os.makedirs(newpath1)
newpath2 = "Resultats_TrouveObjet" + "/AvecObstacle_"+str(count) + "/Images"
if not os.path.exists(newpath2):
    os.makedirs(newpath2)

# Variables globales (Les limites de l'espace)
x_max = 400
x_min = 0
y_max = 400
y_min = 0

# LES FONCTIONS
# Génerer un objet aléatoirement
def point_aleatoire():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    while True:
        # Génère deux nombres aléatoires entre 1 et 400.
        x, y = np.random.randint(1, 400, 2)
        z = 81
        
        # Vérifie si le reste de z / 45 est égal à zéro.
        if x % 45 == 0 and y % 45 == 0:
            # Trace le point sur le graphique 3D.
            ax.scatter(x, y, z, c='k', marker='X')
            break

    objet_coor = (x, y, z)
    print("Objet_coor =", objet_coor)
    # plt.show()
    return objet_coor

def obstacle_table1():
    # Define table dimensions
    table_width = 35
    table_length = 50
    table_height = 81

    # Define table legs
    leg_width = 0
    leg_length = 1
    leg_height = table_height - 1

    # Define table surface
    x = np.linspace(0, table_length, 2)
    y = np.linspace(90, table_width+90, 2)
    X, Y = np.meshgrid(x, y)
    Z = np.full((2, 2), leg_height)

    # Define leg positions
    leg1_pos = [leg_width, leg_length]
    leg2_pos = [table_length - leg_width, leg_length]
    leg3_pos = [leg_width, table_width - leg_length]
    leg4_pos = [table_length - leg_width, table_width - leg_length]

    leg1_corners = np.array([
        [0, 90, leg_height],
        [0, 90, 0],
        [leg_length, 90, 0],
        [leg_length, 90, leg_height]
    ])

    # Define leg 2 corners using the same formula as leg 1 corners but with a different offset
    # We create a 2D numpy array of shape (4,3) by multiplying a 1D array [table_length - 2 * leg_width, 0, 0]
    # by a 2D array of shape (4,1). This ensures that the shapes match for element-wise addition.
    leg2_corners = leg1_corners + np.tile([[table_length - leg_length, 0, 0]], (4, 1))
    leg3_corners = leg1_corners + np.array([[0, table_width, 0]] * 4)
    leg4_corners = leg3_corners + np.array([[table_length - leg_length, 0, 0]] * 4)

    # Plot the table legs
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for corners in [leg1_corners, leg2_corners, leg3_corners, leg4_corners]:
        xs, ys, zs = zip(*corners)
        ax.plot(xs, ys, zs)

    # Plot the table surface
    ax.plot_surface(X, Y, Z, alpha=0.8)

    # Add labels and title
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('3D Furniture Table')

    # # Coordinates of the surface of the table
    # SurfaceX = X[0][1]-X[0][0]
    # SurfaceY = Y[1][0]-Y[0][0]
    # SurfaceZ = table_height

    # TableSurface = (SurfaceX, SurfaceY, SurfaceZ)

    # print('TableSurface=',TableSurface)

    # # Coordinate of the legs of the table
    # xs1, ys1, zs1 = zip(*leg1_corners)
    # print(xs1, ys1, zs1)

    # Coordinates of the surface of the table
    SurfaceX_min = 0
    SurfaceX_max = table_length

    SurfaceX = [SurfaceX_min]
    for i in range(SurfaceX_min+1, SurfaceX_max):
        SurfaceX.append(i)
    SurfaceX.append(SurfaceX_max)

    SurfaceY_min = 90
    SurfaceY_max = table_width+90

    SurfaceY = [SurfaceY_min]
    for i in range(SurfaceY_min+1, SurfaceY_max):
        SurfaceY.append(i)
    SurfaceY.append(SurfaceY_max)

    SurfaceZ = table_height

    # print('SurfaceX=',SurfaceX)
    # print('SurfaceY=',SurfaceY)
    # print('SurfaceZ=',SurfaceZ)

    TableSurface1 = (SurfaceX, SurfaceY, SurfaceZ)
    print('TableSurface1=',TableSurface1)
    
    return TableSurface1
    
def collision_obstacle(ObstacleCoords, d, p, t):
    if t == 1:
        if ((d.cur_loc[0]+p) in ObstacleCoords[0]) and ((d.cur_loc[1]+p) in ObstacleCoords[1]) and (d.cur_loc[2] == ObstacleCoords[2]):
            return True
        else:
            return False
    elif t == 2:
        if ((d.cur_loc[0]-p) in ObstacleCoords[0]) and ((d.cur_loc[1]-p) in ObstacleCoords[1]) and (d.cur_loc[2] == ObstacleCoords[2]):
            return True
        else:
            return False
    elif t == 3:
        if (d.cur_loc[0] in ObstacleCoords[0]) and ((d.cur_loc[1]-p) in ObstacleCoords[1]) and (d.cur_loc[2] == ObstacleCoords[2]):
            return True
        else:
            return False
    
# Comparer leurs coordonnées futurs pour estimer la collision
def collision(d0, d1, p0, p1):
    if (d0.cur_loc[0] + p0) == (d1.cur_loc[0] + p1) and (d0.cur_loc[1] + p0) == (d1.cur_loc[1] + p1):
        print("Attention! Crash between", d0.name, "and", d1.name,"!")
        return True
    else:
        return False

# Avancer et aller à droite pour le 1er drone
def forward_ext2ext(my_drone, cur, ext, pas1, t, objet_coor):
    while cur < ext:
        print("Strategie1 is running")
        if objet_coor == my_drone.cur_loc:
            print("D0 has found the object!")
            break
        else:
            collision(my_drone, my_drone1, pas1, 45) and collision(my_drone, my_drone2, pas1, 45) and collision(my_drone, my_drone3, pas1, 45)
            if (collision(my_drone, my_drone1, pas1, 45) or collision(my_drone, my_drone2, pas1, 45) or collision(my_drone, my_drone3, pas1, 45)) == True:
                my_drone.up(60)
                time.sleep(0.1)
                my_drone.down(30)
                time.sleep(0.1)
                my_drone.down(30)
                time.sleep(0.01)
                print(my_drone.cur_loc) 
            else:
                collision_obstacle(TableSurface1, my_drone, pas1, 1)
                if collision_obstacle(TableSurface1, my_drone, pas1, 1) == True:
                    print("There's an obstacle in front!")
                    my_drone.up(20)
                    my_drone.forward(int(TableSurface1[1][-1]+5-my_drone.cur_loc[1]))
                    my_drone.down(20)
                    print(my_drone.cur_loc) 
                else:
                    if (cur + pas1) > ext:
                        my_drone.forward(int(ext-cur))
                    else:
                        my_drone.forward(pas1)
                    if t == 1:
                        cur = my_drone.cur_loc[1]
                    elif t == 2:
                        cur = my_drone.cur_loc[0]
                    print(my_drone.cur_loc)  

# Reculer et aller à gauche pour le 1er drone 
def forward_ext2ext2(my_drone, cur, ext, pas1, t, objet_coor):
    while cur > ext:
        print("Strategie1 is running")
        if objet_coor == my_drone.cur_loc:
            print("D0 has found the object!")
            break
        else:
            collision(my_drone, my_drone1, pas1, 45) and collision(my_drone, my_drone2, pas1, 45) and collision(my_drone, my_drone3, pas1, 45)
            if (collision(my_drone, my_drone1, pas1, 45) or collision(my_drone, my_drone2, pas1, 45) or collision(my_drone, my_drone3, pas1, 45)) == True:
                my_drone.up(60)
                time.sleep(0.1)
                my_drone.down(30)
                time.sleep(0.1)
                my_drone.down(30)
                time.sleep(0.01)
                print(my_drone.cur_loc) 
            else:
                collision_obstacle(TableSurface1, my_drone, pas1, 1)
                if collision_obstacle(TableSurface1, my_drone, pas1, 1) == True:
                    print("There's an obstacle in front!")
                    my_drone.up(20)
                    my_drone.forward(int(TableSurface1[1][-1]+5-my_drone.cur_loc[1]))
                    my_drone.down(20)
                    print(my_drone.cur_loc) 
                else:
                    if (cur - pas1) < ext:
                        my_drone.forward(int(cur-ext))
                    else:
                        my_drone.forward(pas1)
                    if t == 3:
                        cur = my_drone.cur_loc[1]
                    elif t == 4:
                        cur = my_drone.cur_loc[0]
                    print(my_drone.cur_loc) 
        
# Strategie pour le 1er drone qui tourne de l'extérieur vers le centre
def Strategie1(my_drone, x_max, x_min, y_max, y_min, objet_coor):
    r = 0
    pas1 = 45
    etat_curr = my_drone.cur_loc
    z = etat_curr[2]
    etat_init = (0,0,81)
    if etat_init == etat_curr: 
        ext1 = (x_min, y_min, z)
        ext2 = (x_min, y_max, z)
        ext3 = (x_max, y_max, z)
        ext4 = (x_max, y_min, z)
        
        while ((my_drone.cur_loc[0] >= ((x_max+x_min)/2)+round(pas1/2)) or (my_drone.cur_loc[0] <= ((x_max+x_min)/2)-round(pas1/4))) and ((my_drone.cur_loc[1] >= ((y_max+y_min)/2)+round(pas1/4)) or (my_drone.cur_loc[1] <= ((y_max+y_min)/2)-round(pas1/2))):
            if objet_coor == my_drone.cur_loc:
                print("D0 has found the object!")
                break
            else:
                ext = ext2[1]
                cur = etat_curr[1]
                forward_ext2ext(my_drone, cur, ext, pas1, 1, objet_coor)
                my_drone.cw(90)
                ext = ext3[0]
                cur = my_drone.cur_loc[0]  
                forward_ext2ext(my_drone, cur, ext, pas1, 2, objet_coor)
                my_drone.cw(90)
                ext = ext4[1]
                cur = my_drone.cur_loc[1]  
                forward_ext2ext2(my_drone, cur, ext, pas1, 3, objet_coor)
                my_drone.cw(90)
                
                r += 1
                ext1 = ((x_min+pas1*r), (y_min+pas1*(r-1)), z)
                ext2 = ((x_min+pas1*r), (y_max-pas1*r), z)
                ext3 = ((x_max-pas1*r), (y_max-pas1*r), z)
                ext4 = ((x_max-pas1*r), (y_min+pas1*r), z)
                
                ext = ext1[0]
                cur = my_drone.cur_loc[0]  
                forward_ext2ext2(my_drone, cur, ext, pas1, 4, objet_coor)
                my_drone.cw(90)
        print("Drone1 a fini de parcourir la salle.")
            
    else: 
        print("Drone n'est pas a la position initiale!")
        
# Avancer sur l'axe horizontale pour le 2e drone
def forward_x2x(my_drone1, cur, x_min, x_max, pas2, t, objet_coor):
    # Avancer de gauche à droite
    if t == 1:
        while cur < x_max:
            if objet_coor == my_drone1.cur_loc:
                print("D1 has found the object!")
                break
            else:
                # obstacle_table(my_drone1, pas2)
                # if obstacle_table(my_drone1, pas2) == True:
                #     print("Il y a un obstacle devant!!")
                collision(my_drone1, my_drone, pas2, 45) and collision(my_drone1, my_drone2, pas2, 45) and collision(my_drone1, my_drone3, pas2, 45)
                if ((collision(my_drone1, my_drone, pas2, 45) and collision(my_drone1, my_drone2, pas2, 45) and collision(my_drone1, my_drone3, pas2, 45)) or (collision(my_drone1, my_drone, pas2, 45) and collision(my_drone1, my_drone2, pas2, 45)) or (collision(my_drone1, my_drone, pas2, 45) and collision(my_drone1, my_drone3, pas2, 45)))== True:
                    my_drone1.up(90)
                    time.sleep(0.1)
                    my_drone1.down(30)
                    time.sleep(0.1)
                    my_drone1.down(30)
                    time.sleep(5)
                    my_drone1.down(30)
                    time.sleep(0.01)
                    print(my_drone1.cur_loc) 
                elif (collision(my_drone1, my_drone2, pas2, 45) or collision(my_drone1, my_drone3, pas2, 45)) == True:
                    my_drone1.up(60)
                    time.sleep(0.1)
                    my_drone1.down(30)
                    time.sleep(0.1)
                    my_drone1.down(30)
                    time.sleep(0.01)
                    print(my_drone1.cur_loc) 
                else:
                    collision_obstacle(TableSurface1, my_drone1, pas2, 1)
                    if collision_obstacle(TableSurface1, my_drone1, pas2, 1) == True:
                        print("There's an obstacle in front!")
                        my_drone1.up(20)
                        my_drone1.forward(pas2)
                        my_drone1.right(int(TableSurface1[0][-1]+5-my_drone1.cur_loc[0]))
                        my_drone1.down(20)
                        print(my_drone1.cur_loc) 
                    else:
                        if (cur + pas2) > x_max :
                            my_drone1.right(int(x_max-cur))
                            cur = my_drone1.cur_loc[0]
                        else:
                            my_drone1.right(pas2)
                            cur = my_drone1.cur_loc[0]
                        print(my_drone1.cur_loc)  
    
    # Avancer de droite à gauche
    elif t == 2:
        while cur > x_min:
            if objet_coor == my_drone1.cur_loc:
                print("D1 has found the object!")
                break
            else:
                collision(my_drone1, my_drone, pas2, 45) and collision(my_drone1, my_drone2, pas2, 45) and collision(my_drone1, my_drone3, pas2, 45)
                if ((collision(my_drone1, my_drone, pas2, 45) and collision(my_drone1, my_drone2, pas2, 45) and collision(my_drone1, my_drone3, pas2, 45)) or (collision(my_drone1, my_drone, pas2, 45) and collision(my_drone1, my_drone2, pas2, 45)) or (collision(my_drone1, my_drone, pas2, 45) and collision(my_drone1, my_drone3, pas2, 45)))== True:
                    my_drone1.up(90)
                    time.sleep(0.1)
                    my_drone1.down(30)
                    time.sleep(0.1)
                    my_drone1.down(30)
                    time.sleep(0.1)
                    my_drone1.down(30)
                    time.sleep(0.01)
                    print(my_drone1.cur_loc) 
                elif (collision(my_drone1, my_drone2, pas2, 45) or collision(my_drone1, my_drone3, pas2, 45)) == True:
                    my_drone1.up(60)
                    time.sleep(0.1)
                    my_drone1.down(30)
                    time.sleep(0.1)
                    my_drone1.down(30)
                    time.sleep(0.01)
                    print(my_drone1.cur_loc) 
                else:
                    collision_obstacle(TableSurface1, my_drone1, pas2, 1)
                    if collision_obstacle(TableSurface1, my_drone1, pas2, 1) == True:
                        print("There's an obstacle in front!")
                        my_drone1.up(20)
                        my_drone1.forward(pas2)
                        my_drone1.right(int(TableSurface1[0][-1]+5-my_drone1.cur_loc[0]))
                        my_drone1.down(20)
                        print(my_drone1.cur_loc) 
                    else:
                        if (cur - pas2) < x_min:
                            my_drone1.right(int(cur-x_min))
                            cur = my_drone1.cur_loc[0]
                        else:
                            my_drone1.right(pas2)
                            cur = my_drone1.cur_loc[0]
                        print(my_drone1.cur_loc)  
            
# Avancer sur l'axe verticale pour le 2e drone
def forward_y2y(my_drone1, cur1, y_max, pas2, objet_coor):
    cur1 = my_drone1.cur_loc[1]
    if objet_coor == my_drone1.cur_loc:
        print("D1 has found the object!")
        pass
    else:
        collision(my_drone1, my_drone, pas2, 45) and collision(my_drone1, my_drone2, pas2, 45) and collision(my_drone1, my_drone3, pas2, 45)
        if ((collision(my_drone1, my_drone, pas2, 45) and collision(my_drone1, my_drone2, pas2, 45) and collision(my_drone1, my_drone3, pas2, 45)) or (collision(my_drone1, my_drone, pas2, 45) and collision(my_drone1, my_drone2, pas2, 45)) or (collision(my_drone1, my_drone, pas2, 45) and collision(my_drone1, my_drone3, pas2, 45)))== True:
            my_drone1.up(90)
            time.sleep(0.1)
            my_drone1.down(30)
            time.sleep(0.1)
            my_drone1.down(30)
            time.sleep(0.1)
            my_drone1.down(30)
            time.sleep(0.01)
            print(my_drone1.cur_loc) 
        elif (collision(my_drone1, my_drone2, pas2, 45) or collision(my_drone1, my_drone3, pas2, 45)) == True:
            my_drone1.up(60)
            time.sleep(0.1)
            my_drone1.down(30)
            time.sleep(0.1)
            my_drone1.down(30)
            time.sleep(0.01)
            print(my_drone1.cur_loc) 
        else:
            collision_obstacle(TableSurface1, my_drone1, pas2, 1) and collision_obstacle(TableSurface1, my_drone1, pas2, 2)
            if collision_obstacle(TableSurface1, my_drone1, pas2, 1) == True:
                print("There's an obstacle in front!")
                my_drone1.up(20)
                my_drone1.forward(pas2)
                my_drone1.right(int(TableSurface1[0][-1]+5-my_drone1.cur_loc[0]))
                my_drone1.down(20)
                print(my_drone1.cur_loc) 
            elif collision_obstacle(TableSurface1, my_drone1, pas2, 2) == True:
                print("There's an obstacle in front!")
                my_drone1.up(20)
                my_drone1.left(int(my_drone1.cur_loc[0]-TableSurface1[0][0]))
                my_drone1.forward(pas2)
                my_drone1.right(int(TableSurface1[0][-1]+5-my_drone1.cur_loc[0]))
                my_drone1.down(20)
                print(my_drone1.cur_loc)
            else:
                if (cur1 + pas2) > y_max:
                    my_drone1.forward(int(y_max-cur1))
                else:
                    my_drone1.forward(pas2)
                print(my_drone1.cur_loc)
       
# Strategie pour le 2e drone qui avance dans la forme 'S' du bas jusqu'au haut
def Strategie2(my_drone1, x_min, x_max, y_min, y_max, objet_coor):
    print("Strategie2 is running")
    pas2 = 45
    etat_curr = my_drone1.cur_loc
    z = etat_curr[2]
    etat_init = (0,0,81)
    if etat_init == etat_curr: 
        ext = (x_min, y_max, z)
        
        while my_drone1.cur_loc != ext:
            if objet_coor == my_drone1.cur_loc:
                print("D1 has found the object!")
                break
            else:
                cur = my_drone1.cur_loc[0]
                forward_x2x(my_drone1, cur, x_min, x_max, pas2, 1, objet_coor)
                cur1 = my_drone1.cur_loc[1]
                forward_y2y(my_drone1, cur1, y_max, pas2, objet_coor)
                my_drone1.cw(180)
                cur = my_drone1.cur_loc[0]
                forward_x2x(my_drone1, cur, x_min, x_max, pas2, 2, objet_coor)
                my_drone1.cw(180)
                cur1 = my_drone1.cur_loc[1]
                forward_y2y(my_drone1, cur1, y_max, pas2, objet_coor)
        print("Drone2 a fini de parcourir la salle.")
            
    else: 
        print("Drone n'est pas a la position initiale!")

# Avancer de bas jusqu'au haut et de gauche à droite pour le 3e drone
def forward_lim2lim(my_drone2, pas3, angle, objet_coor):
    angle_degrees = abs(180 - angle)
    angle_radians = math.radians(angle_degrees)
    angle_compa = math.tan(angle_radians)
    
    if objet_coor == my_drone2.cur_loc:
        print("D2 has found the object!")
        pass
    else:
        collision(my_drone2, my_drone3, pas3, 45)
        if collision(my_drone2, my_drone3, pas3, 45) == True:
            my_drone2.up(30)
            time.sleep(0.1)
            my_drone2.down(30)
            time.sleep(0.01)
            print(my_drone2.cur_loc)
        else:
            collision_obstacle(TableSurface1, my_drone2, pas3, 1) and collision_obstacle(TableSurface1, my_drone2, pas3, 3)
            if collision_obstacle(TableSurface1, my_drone2, pas3, 1) == True:
                print("There's an obstacle in front!")
                my_drone2.up(20)
                my_drone2.forward(int(TableSurface1[1][-1]+5-my_drone2.cur_loc[1]))
                my_drone2.down(20)
                print(my_drone2.cur_loc)
            elif collision_obstacle(TableSurface1, my_drone2, pas3, 3) == True:
                print("There's an obstacle in front!")
                my_drone2.up(20)
                my_drone2.forward(int(my_drone2.cur_loc[1]-(TableSurface1[1][0]-5)))
                my_drone2.down(20)
                print(my_drone2.cur_loc)
            elif (my_drone2.cur_loc[0] + round(pas3*angle_compa)) >= x_max:
                print("d2 running first condition!")
                if my_drone2.cur_loc[0] == x_max:
                    pass
                else:
                    my_drone2.forward(int(int(x_max-my_drone2.cur_loc[0])/angle_compa))
                print(my_drone2.cur_loc)
                my_drone2.ccw(angle)
                while my_drone2.cur_loc[1] > y_min:
                    if objet_coor == my_drone2.cur_loc:
                        print("D2 has found the object!")
                        break
                    else:
                        collision(my_drone2, my_drone3, pas3, 45)
                        if collision(my_drone2, my_drone3, pas3, 45) == True:
                            my_drone2.up(30)
                            time.sleep(0.1)
                            my_drone2.down(30)
                            time.sleep(0.01)
                            print(my_drone2.cur_loc)
                        else:
                            collision_obstacle(TableSurface1, my_drone2, pas3, 3)
                            if collision_obstacle(TableSurface1, my_drone2, pas3, 3) == True:
                                print("There's an obstacle in front!")
                                my_drone2.up(20)
                                my_drone2.back(int(my_drone2.cur_loc[1]-TableSurface1[1][0]-5))
                                my_drone2.down(20)
                                print(my_drone2.cur_loc)
                            elif (my_drone2.cur_loc[1] - pas3) < y_min:
                                my_drone2.back(int(my_drone2.cur_loc[1]-y_min))
                            else:
                                my_drone2.back(pas3)
                            print(my_drone2.cur_loc)
            else:
                my_drone2.forward(pas3)
                print(my_drone2.cur_loc)
                if (my_drone2.cur_loc[1] + pas3) > y_max:
                    print("D2 running 2a")
                    my_drone2.forward(int(y_max-my_drone2.cur_loc[1]))
                    print(my_drone2.cur_loc)
                    my_drone2.cw(angle)
                elif my_drone2.cur_loc[1] == y_max:
                    print("D2 running 2b")
                    my_drone2.cw(angle)
                elif (my_drone2.cur_loc[1] - pas3) < y_min:
                    print("D2 running 2c")
                    my_drone2.forward(int(my_drone2.cur_loc[1]-y_min))
                    print(my_drone2.cur_loc)
                    my_drone2.ccw(angle)
                elif my_drone2.cur_loc[1] == y_min:
                    print("D2 running 2d")
                    my_drone2.ccw(angle)
        
# Strategie pour le 3e drone qui vole en zig-zag de gauche à droite
def Strategie3(my_drone2, x_min, x_max, y_min, y_max, objet_coor):
    print("Strategie3 is running")
    pas3 = 45
    angle = 175
    etat_curr = my_drone2.cur_loc
    etat_init = (0,0,81)
    if etat_init == etat_curr: 
        while my_drone2.cur_loc[0] < x_max:
            if objet_coor == my_drone2.cur_loc:
                print("D2 has found the object!")
                break
            else:
                forward_lim2lim(my_drone2, pas3, angle, objet_coor)
        print("Drone3 a fini de parcourir la salle.")
    else:
        print("Drone n'est pas a la position initiale!")
    
# Avancer de bas jusqu'au haut et de droite à gauche pour le 4e drone
def forward_lim2lim2(my_drone3, pas4, angle, objet_coor):
    angle_degrees = abs(180 - angle)
    angle_radians = math.radians(angle_degrees)
    angle_compa = math.tan(angle_radians)
    
    if objet_coor == my_drone3.cur_loc:
        print("D3 has found the object!")
        pass
    else:
        collision_obstacle(TableSurface1, my_drone3, pas4, 3)
        if collision_obstacle(TableSurface1, my_drone3, pas4, 3) == True:
            print("There's an obstacle in front!")
            my_drone3.up(20)
            my_drone3.forward(int(my_drone3.cur_loc[1]-(TableSurface1[1][0]-5)))
            my_drone3.down(20)
            print(my_drone3.cur_loc) 
        elif (my_drone3.cur_loc[0] - round(pas4*angle_compa)) <= x_min:
            print("d3 running second condition!")
            if my_drone3.cur_loc[0] == x_min:
                pass
            else:
                my_drone3.forward(int(int(my_drone3.cur_loc[0]-x_min)/angle_compa))
            print(my_drone3.cur_loc)
            my_drone3.ccw(angle)
            while my_drone3.cur_loc[1] > y_min:
                if objet_coor == my_drone3.cur_loc:
                    print("D3 has found the object!")
                    break
                else:
                    collision_obstacle(TableSurface1, my_drone3, pas4, 3)
                    if collision_obstacle(TableSurface1, my_drone3, pas4, 3) == True:
                        print("There's an obstacle in front!")
                        my_drone3.up(20)
                        my_drone3.back(int(my_drone3.cur_loc[1]-(TableSurface1[1][0]-5)))
                        my_drone3.down(20)
                        print(my_drone3.cur_loc)
                    elif (my_drone3.cur_loc[1] - pas4) < y_min:
                        my_drone3.back(int(my_drone3.cur_loc[1]-y_min))
                    else:
                        my_drone3.back(pas4)
                    print(my_drone3.cur_loc)
        else:
            my_drone3.forward(pas4)
            print(my_drone3.cur_loc)
            if (my_drone3.cur_loc[1] + pas4) > y_max:
                my_drone3.forward(int(y_max-my_drone3.cur_loc[1]))
                print(my_drone3.cur_loc)
                my_drone3.cw(angle)
            elif my_drone3.cur_loc[1] == y_max:
                my_drone3.cw(angle)
            elif (my_drone3.cur_loc[1] - pas4) < y_min:
                my_drone3.forward(int(my_drone3.cur_loc[1]-y_min))
                print(my_drone3.cur_loc)
                my_drone3.ccw(angle)
            elif my_drone3.cur_loc[1] == y_min:
                my_drone3.ccw(angle)
        
# Strategie pour le 4e drone qui parts de gauche à droite et puis vole en zig-zag de droite à gauche (l'inverse de strategie pour le 3e drone)
def Strategie4(my_drone3, x_min, x_max, y_min, y_max, objet_coor):
        print("Strategie4 is running")
        pas4 = 45
        angle = 185
        etat_curr = my_drone3.cur_loc
        etat_init = (0,0,81)
        if etat_init == etat_curr: 
            cur = my_drone3.cur_loc[0]
            while cur < x_max:
                if objet_coor == my_drone3.cur_loc:
                    print("D3 has found the object!")
                    break
                else:
                    if (cur + pas4) > x_max:
                        my_drone3.right(int(x_max-cur))
                        cur = my_drone3.cur_loc[0]
                    else:
                        my_drone3.right(pas4)
                        cur = my_drone3.cur_loc[0]
                    print(my_drone3.cur_loc)
            while my_drone3.cur_loc[0] > x_min:
                if objet_coor == my_drone3.cur_loc:
                    print("D3 has found the object!")
                    break
                else:
                    forward_lim2lim2(my_drone3, pas4, angle, objet_coor)
            print("Drone4 a fini de parcourir la salle.")
                
        else:
            print("Drone n'est pas a la position initiale!")
    
# # Plot de tous les drones sur un meme graphe de 3D
# def plot_all_in_a_graph(d0, d1, d2, d3, obj, color0, color1, color2, color3, col, e):
#     ax = plt.figure().add_subplot(projection='3d')
    
#     coords0_df = pd.DataFrame(d0.path_coors)
#     coords1_df = pd.DataFrame(d1.path_coors)
#     coords2_df = pd.DataFrame(d2.path_coors)
#     coords3_df = pd.DataFrame(d3.path_coors)
#     coords4_df = obj
    
#     xlow = min(coords0_df[0]) and min(coords1_df[0]) and min(coords2_df[0]) and min(coords3_df[0])
#     xhi = max(coords0_df[0]) and max(coords1_df[0]) and max(coords2_df[0]) and max(coords3_df[0])
#     ylow = min(coords0_df[1]) and min(coords1_df[1]) and min(coords2_df[1]) and min(coords3_df[1])
#     yhi = max(coords0_df[1]) and max(coords1_df[1]) and max(coords2_df[1]) and max(coords3_df[1])
#     zlow = min(coords0_df[2]) and min(coords1_df[2]) and min(coords2_df[2]) and min(coords3_df[2])
#     zhi = max(coords0_df[2]) and max(coords1_df[2]) and max(coords2_df[2]) and max(coords3_df[2])
     
#     xlowlim = 0 if xlow > 0 else xlow - 40
#     xhilim = 400 if xhi < 400 else xhi + 40
#     ylowlim = 0 if ylow > 0 else ylow - 40
#     yhilim = 400 if yhi < 400 else yhi + 40
#     zlowlim = 0 if zlow > 0 else zlow - 40
#     zhilim = 400 if zhi < 400 else zhi + 40
     
#     ax.set_xlim([xlowlim, xhilim])
#     ax.set_ylim([ylowlim, yhilim])
#     ax.set_zlim([zlowlim, zhilim])
    
#     # Plot the arrays in a 3D graph 
#     x0, y0, z0 = coords0_df[0], coords0_df[1], coords0_df[2]
#     ax.plot(x0, y0, z0, color0, linestyle='dashed', linewidth=1, markersize=6, label="Drone Moves")
#     ax.plot(x0, y0, z0, linewidth=2, alpha=.15)
    
#     x1, y1, z1 = coords1_df[0], coords1_df[1], coords1_df[2]
#     ax.plot(x1, y1, z1, color1, linestyle='dashed', linewidth=1, markersize=6, label="Drone Moves")
#     ax.plot(x1, y1, z1, linewidth=2, alpha=.15)
    
#     x2, y2, z2 = coords2_df[0], coords2_df[1], coords2_df[2]
#     ax.plot(x2, y2, z2, color2, linestyle='dashed', linewidth=1, markersize=6, label="Drone Moves")
#     ax.plot(x2, y2, z2, linewidth=2, alpha=.15)
    
#     x3, y3, z3 = coords3_df[0], coords3_df[1], coords3_df[2]
#     ax.plot(x3, y3, z3, color3, linestyle='dashed', linewidth=1, markersize=6, label="Drone Moves")
#     ax.plot(x3, y3, z3, linewidth=2, alpha=.15)
    
#     x4, y4, z4 = coords4_df[0], coords4_df[1], coords4_df[2]
#     ax.plot(x4, y4, z4, col, linestyle='dashed', linewidth=2, markersize=6, label="Object")
#     ax.plot(x4, y4, z4, linewidth=2, alpha=.15)
    
#     # ax.plot(xs, ys, zs)
#     # ax.plot_surface(X, Y, Z, alpha=0.8)
     
#     if len(d0.flip_coors) > 0:
#         flip_df = pd.DataFrame(d0.flip_coors)
#         ax.plot(flip_df[0], flip_df[1], color0, markersize=6, label="Drone Flips")
        
#     if len(d1.flip_coors) > 0:
#         flip_df = pd.DataFrame(d1.flip_coors)
#         ax.plot(flip_df[0], flip_df[1], color1, markersize=6, label="Drone Flips")
        
#     if len(d2.flip_coors) > 0:
#         flip_df = pd.DataFrame(d2.flip_coors)
#         ax.plot(flip_df[0], flip_df[1], color2, markersize=6, label="Drone Flips")
        
#     ax.xaxis.set_major_locator(MaxNLocator(integer=True))
#     ax.grid()
#     ax.legend()
#     ax.set(xlabel='X Distance from Takeoff', ylabel='Y Distance from Takeoff', zlabel='Altitude in Centimeters', title='Tello Altitude')

#     z0[-1:] = [x0*5 + 2 for x0 in z0[-1:]]
#     categories = "A "*1 + "B "*1
#     categories = categories.split(" ")
#     categories.pop(2)
#     df0 = pd.DataFrame(list(zip_longest(categories, x0, y0, z0)), columns=['cat','col_x','col_y','col_z'])
#     df0.head()
#     d0.df = df0
#     fig = px.scatter_3d(df0, x='col_x', y='col_y', z='col_z', color='cat', title="3D Scatter Plot")
    
#     z1[-1:] = [x1*5 + 2 for x1 in z1[-1:]]
#     categories = "A "*1 + "B "*1
#     categories = categories.split(" ")
#     categories.pop(2)
#     df1 = pd.DataFrame(list(zip_longest(categories, x1, y1, z1)), columns=['cat','col_x','col_y','col_z'])
#     df1.head()
#     d1.df = df1
#     fig = px.scatter_3d(df1, x='col_x', y='col_y', z='col_z', color='cat', title="3D Scatter Plot")
    
#     z2[-1:] = [x2*5 + 2 for x2 in z2[-1:]]
#     categories = "A "*1 + "B "*1
#     categories = categories.split(" ")
#     categories.pop(2)
#     df2 = pd.DataFrame(list(zip_longest(categories, x2, y2, z2)), columns=['cat','col_x','col_y','col_z'])
#     df2.head()
#     d2.df = df2
#     fig = px.scatter_3d(df2, x='col_x', y='col_y', z='col_z', color='cat', title="3D Scatter Plot")
#     fig.show()
    
#     plt.show()  
    
#     # Construct the full path to the file
#     file_path = os.path.join(newpath2, "graph.png")

#     # Save the graph to a file
#     plt.savefig(file_path)
    
# Plot de tous les drones sur un meme graphe de 3D
def plot_all_in_a_graph(d0, d1, d2, d3, obj, color0, color1, color2, color3, col, e):
    
    # Define table dimensions
    table_width = 35
    table_length = 50
    table_height = 81

    # Define table legs
    leg_width = 0
    leg_length = 1
    leg_height = table_height - 1

    # Define table surface
    x = np.linspace(0, table_length, 2)
    y = np.linspace(90, table_width+90, 2)
    X, Y = np.meshgrid(x, y)
    Z = np.full((2, 2), leg_height)

    # # Define leg positions
    # leg1_pos = [leg_width, leg_length]
    # leg2_pos = [table_length - leg_width, leg_length]
    # leg3_pos = [leg_width, table_width - leg_length]
    # leg4_pos = [table_length - leg_width, table_width - leg_length]

    leg1_corners = np.array([
        [0, 90, leg_height],
        [0, 90, 0],
        [leg_length, 90, 0],
        [leg_length, 90, leg_height]
    ])

    # Define leg 2 corners using the same formula as leg 1 corners but with a different offset
    # We create a 2D numpy array of shape (4,3) by multiplying a 1D array [table_length - 2 * leg_width, 0, 0]
    # by a 2D array of shape (4,1). This ensures that the shapes match for element-wise addition.
    leg2_corners = leg1_corners + np.tile([[table_length - leg_length, 0, 0]], (4, 1))
    leg3_corners = leg1_corners + np.array([[0, table_width, 0]] * 4)
    leg4_corners = leg3_corners + np.array([[table_length - leg_length, 0, 0]] * 4)
    
    ax = plt.figure().add_subplot(111, projection='3d')
    
    coords0_df = pd.DataFrame(d0.path_coors)
    coords1_df = pd.DataFrame(d1.path_coors)
    coords2_df = pd.DataFrame(d2.path_coors)
    coords3_df = pd.DataFrame(d3.path_coors)
    coords4_df = obj
    
    xlow = min(coords0_df[0]) and min(coords1_df[0]) and min(coords2_df[0]) and min(coords3_df[0])
    xhi = max(coords0_df[0]) and max(coords1_df[0]) and max(coords2_df[0]) and max(coords3_df[0])
    ylow = min(coords0_df[1]) and min(coords1_df[1]) and min(coords2_df[1]) and min(coords3_df[1])
    yhi = max(coords0_df[1]) and max(coords1_df[1]) and max(coords2_df[1]) and max(coords3_df[1])
    zlow = min(coords0_df[2]) and min(coords1_df[2]) and min(coords2_df[2]) and min(coords3_df[2])
    zhi = max(coords0_df[2]) and max(coords1_df[2]) and max(coords2_df[2]) and max(coords3_df[2])
     
    xlowlim = 0 if xlow > 0 else xlow - 40
    xhilim = 400 if xhi < 400 else xhi + 40
    ylowlim = 0 if ylow > 0 else ylow - 40
    yhilim = 400 if yhi < 400 else yhi + 40
    zlowlim = 0 if zlow > 0 else zlow - 40
    zhilim = 400 if zhi < 400 else zhi + 40
     
    ax.set_xlim([xlowlim, xhilim])
    ax.set_ylim([ylowlim, yhilim])
    ax.set_zlim([zlowlim, zhilim])
    
    # Plot the arrays in a 3D graph 
    x0, y0, z0 = coords0_df[0], coords0_df[1], coords0_df[2]
    ax.plot(x0, y0, z0, color0, linestyle='dashed', linewidth=1, markersize=6, label="Drone Moves")
    ax.plot(x0, y0, z0, linewidth=2, alpha=.15)
    
    x1, y1, z1 = coords1_df[0], coords1_df[1], coords1_df[2]
    ax.plot(x1, y1, z1, color1, linestyle='dashed', linewidth=1, markersize=6, label="Drone Moves")
    ax.plot(x1, y1, z1, linewidth=2, alpha=.15)
    
    x2, y2, z2 = coords2_df[0], coords2_df[1], coords2_df[2]
    ax.plot(x2, y2, z2, color2, linestyle='dashed', linewidth=1, markersize=6, label="Drone Moves")
    ax.plot(x2, y2, z2, linewidth=2, alpha=.15)
    
    x3, y3, z3 = coords3_df[0], coords3_df[1], coords3_df[2]
    ax.plot(x3, y3, z3, color3, linestyle='dashed', linewidth=1, markersize=6, label="Drone Moves")
    ax.plot(x3, y3, z3, linewidth=2, alpha=.15)
    
    x4, y4, z4 = coords4_df[0], coords4_df[1], coords4_df[2]
    ax.plot(x4, y4, z4, col, linestyle='dashed', linewidth=2, markersize=6, label="Object")
    ax.plot(x4, y4, z4, linewidth=2, alpha=.15)
    
    # Plot the table legs
    for corners in [leg1_corners, leg2_corners, leg3_corners, leg4_corners]:
        xs, ys, zs = zip(*corners)
        ax.plot(xs, ys, zs)

    # Plot the table surface
    ax.plot_surface(X, Y, Z, alpha=0.8)
     
    if len(d0.flip_coors) > 0:
        flip_df = pd.DataFrame(d0.flip_coors)
        ax.plot(flip_df[0], flip_df[1], color0, markersize=6, label="Drone Flips")
        
    if len(d1.flip_coors) > 0:
        flip_df = pd.DataFrame(d1.flip_coors)
        ax.plot(flip_df[0], flip_df[1], color1, markersize=6, label="Drone Flips")
        
    if len(d2.flip_coors) > 0:
        flip_df = pd.DataFrame(d2.flip_coors)
        ax.plot(flip_df[0], flip_df[1], color2, markersize=6, label="Drone Flips")
        
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax.grid()
    ax.legend()
    ax.set(xlabel='X Distance from Takeoff', ylabel='Y Distance from Takeoff', zlabel='Altitude in Centimeters', title='Tello Altitude')

    z0[-1:] = [x0*5 + 2 for x0 in z0[-1:]]
    categories = "A "*1 + "B "*1
    categories = categories.split(" ")
    categories.pop(2)
    df0 = pd.DataFrame(list(zip_longest(categories, x0, y0, z0)), columns=['cat','col_x','col_y','col_z'])
    df0.head()
    d0.df = df0
    fig = px.scatter_3d(df0, x='col_x', y='col_y', z='col_z', color='cat', title="3D Scatter Plot")
    
    z1[-1:] = [x1*5 + 2 for x1 in z1[-1:]]
    categories = "A "*1 + "B "*1
    categories = categories.split(" ")
    categories.pop(2)
    df1 = pd.DataFrame(list(zip_longest(categories, x1, y1, z1)), columns=['cat','col_x','col_y','col_z'])
    df1.head()
    d1.df = df1
    fig = px.scatter_3d(df1, x='col_x', y='col_y', z='col_z', color='cat', title="3D Scatter Plot")
    
    z2[-1:] = [x2*5 + 2 for x2 in z2[-1:]]
    categories = "A "*1 + "B "*1
    categories = categories.split(" ")
    categories.pop(2)
    df2 = pd.DataFrame(list(zip_longest(categories, x2, y2, z2)), columns=['cat','col_x','col_y','col_z'])
    df2.head()
    d2.df = df2
    fig = px.scatter_3d(df2, x='col_x', y='col_y', z='col_z', color='cat', title="3D Scatter Plot")
    fig.show()
    
    plt.show()  


# MAIN CODE
# Initialiser tous les drones
my_drone = Simulator('D0')
my_drone1 = Simulator('D1')
my_drone2 = Simulator('D2')
my_drone3 = Simulator('D3')

# Commencer de voler
my_drone.takeoff()
my_drone1.takeoff()
my_drone2.takeoff()
my_drone3.takeoff()

# Generer un point aléatoirement
objet_coor = point_aleatoire()

TableSurface1 = obstacle_table1()

# Start all functions in separate threads
t1 = threading.Thread(target=Strategie1, args=(my_drone, x_max, x_min, y_max, y_min, objet_coor,))
t2 = threading.Thread(target=Strategie2, args=(my_drone1, x_min, x_max, y_min, y_max, objet_coor,))
t3 = threading.Thread(target=Strategie3, args=(my_drone2, x_min, x_max, y_min, y_max, objet_coor,))
t4 = threading.Thread(target=Strategie4, args=(my_drone3, x_min, x_max, y_min, y_max, objet_coor,))
t1.start()
t2.start()
t3.start()
t4.start()

# Wait until all threads finish
t1.join()
t2.join()
t3.join()
t4.join()

# Finir de voler
my_drone.land('rP')
my_drone1.land('gP')
my_drone2.land('cP')
my_drone3.land('yP')

# Demontrer sur le graphe
plot_all_in_a_graph(my_drone, my_drone1, my_drone2, my_drone3, objet_coor, 'rP', 'gP', 'cP', 'yP', 'k^', 15)

print('TableSurface1=',TableSurface1)
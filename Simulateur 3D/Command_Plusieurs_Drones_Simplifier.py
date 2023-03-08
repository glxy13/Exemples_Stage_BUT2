# -*- coding: utf-8 -*-
"""
Created on Thu Feb 16 00:01:09 2023

@author: Asus
"""

# -*- coding: utf-8 -*-
"""
Created on Tue Feb  7 08:05:32 2023

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

# creer un repertoire Test avec un ID
# enregistrer les images du drones
import os
count = 0 
newpath = "Resultats_Testes_Strategies"
if not os.path.exists(newpath):
    os.makedirs(newpath)
newpath1 = "Resultats_Testes_Strategies" + "/Test_"+str(count)
if not os.path.exists(newpath1):
    os.makedirs(newpath1)
newpath2 = "Resultats_Testes_Strategies" + "/Test_"+str(count) + "/Images"
if not os.path.exists(newpath2):
    os.makedirs(newpath2)

# Variables globales (Les limites de l'espace)
x_max = 400
x_min = 0
y_max = 400
y_min = 0

# LES FONCTIONS
# Avancer et aller à droite pour le 1er drone
def forward_ext2ext(my_drone, cur, ext, pas, t):
    while cur < ext:
        if (cur + pas) > ext:
            my_drone.forward(int(ext-cur))
        else:
            my_drone.forward(pas)
        if t == 1:
            cur = my_drone.cur_loc[1]
        elif t == 2:
            cur = my_drone.cur_loc[0]
        print(my_drone.cur_loc)  

# Reculer et aller à gauche pour le 1er drone 
def forward_ext2ext2(my_drone, cur, ext, pas, t):
    while cur > ext:
        print("Strategie1 is running")
        if (cur - pas) < ext:
            my_drone.forward(int(cur-ext))
        else:
            my_drone.forward(pas)
        if t == 3:
            cur = my_drone.cur_loc[1]
        elif t == 4:
            cur = my_drone.cur_loc[0]
        print(my_drone.cur_loc) 
        
# Strategie pour le 1er drone qui tourne de l'extérieur vers le centre
def Strategie1(my_drone, x_max, x_min, y_max, y_min):
    r = 0
    pas = 45
    etat_curr = my_drone.cur_loc
    z = etat_curr[2]
    etat_init = (0,0,81)
    if etat_init == etat_curr: 
        ext1 = (x_min, y_min, z)
        ext2 = (x_min, y_max, z)
        ext3 = (x_max, y_max, z)
        ext4 = (x_max, y_min, z)
        
        while ((my_drone.cur_loc[0] >= ((x_max+x_min)/2)+round(pas/2)) or (my_drone.cur_loc[0] <= ((x_max+x_min)/2)-round(pas/4))) and ((my_drone.cur_loc[1] >= ((y_max+y_min)/2)+round(pas/4)) or (my_drone.cur_loc[1] <= ((y_max+y_min)/2)-round(pas/2))):
            ext = ext2[1]
            cur = etat_curr[1]
            forward_ext2ext(my_drone,cur,ext,pas,1)
            my_drone.cw(90)
            ext = ext3[0]
            cur = my_drone.cur_loc[0]  
            forward_ext2ext(my_drone,cur,ext,pas,2)
            my_drone.cw(90)
            ext = ext4[1]
            cur = my_drone.cur_loc[1]  
            forward_ext2ext2(my_drone,cur,ext,pas,3)
            my_drone.cw(90)
            
            r += 1
            ext1 = ((x_min+pas*r), (y_min+pas*(r-1)), z)
            ext2 = ((x_min+pas*r), (y_max-pas*r), z)
            ext3 = ((x_max-pas*r), (y_max-pas*r), z)
            ext4 = ((x_max-pas*r), (y_min+pas*r), z)
            
            ext = ext1[0]
            cur = my_drone.cur_loc[0]  
            forward_ext2ext2(my_drone,cur,ext,pas,4)
            my_drone.cw(90)
        print("Drone1 a fini de parcourir la salle.")
            
    else: 
        print("Drone n'est pas a la position initiale!")
        
# Avancer sur l'axe horizontale pour le 2e drone
def forward_x2x(my_drone1, cur, x_min, x_max, pas, t):
    # Avancer de gauche à droite
    if t == 1:
        while cur < x_max:
            if (cur + pas) > x_max :
                my_drone1.right(int(x_max-cur))
                cur = my_drone1.cur_loc[0]
            else:
                my_drone1.right(pas)
                cur = my_drone1.cur_loc[0]
            print(my_drone1.cur_loc)  
    
    # Avancer de droite à gauche
    elif t == 2:
        while cur > x_min:
            if (cur - pas) < x_min:
                my_drone1.right(int(cur-x_min))
                cur = my_drone1.cur_loc[0]
            else:
                my_drone1.right(pas)
                cur = my_drone1.cur_loc[0]
            print(my_drone1.cur_loc)  
            
# Avancer sur l'axe verticale pour le 2e drone
def forward_y2y(my_drone1, cur1, y_max, pas):
    cur1 = my_drone1.cur_loc[1]
    if (cur1 + pas) > y_max:
        my_drone1.forward(int(y_max-cur1))
    else:
        my_drone1.forward(pas)
    print(my_drone1.cur_loc)
       
# Strategie pour le 2e drone qui avance dans la forme 'S' du bas jusqu'au haut
def Strategie2(my_drone1, x_min, x_max, y_min, y_max):
    print("Strategie2 is running")
    pas = 45
    etat_curr = my_drone1.cur_loc
    z = etat_curr[2]
    etat_init = (0,0,81)
    if etat_init == etat_curr: 
        ext = (x_min, y_max, z)
        
        while my_drone1.cur_loc != ext:
            cur = my_drone1.cur_loc[0]
            forward_x2x(my_drone1, cur, x_min, x_max, pas, 1)
            cur1 = my_drone1.cur_loc[1]
            forward_y2y(my_drone1, cur1, y_max, pas)
            my_drone1.cw(180)
            cur = my_drone1.cur_loc[0]
            forward_x2x(my_drone1, cur, x_min, x_max, pas, 2)
            my_drone1.cw(180)
            cur1 = my_drone1.cur_loc[1]
            forward_y2y(my_drone1, cur1, y_max, pas)
        print("Drone2 a fini de parcourir la salle.")
            
    else: 
        print("Drone n'est pas a la position initiale!")

# Avancer de bas jusqu'au haut et de gauche à droite pour le 3e drone
def forward_lim2lim(my_drone2, pas, angle):
    angle_degrees = abs(180 - angle)
    angle_radians = math.radians(angle_degrees)
    angle_compa = math.tan(angle_radians)
    
    if (my_drone2.cur_loc[0] + round(pas*angle_compa)) >= x_max:
        print("d2 running first condition!")
        if my_drone2.cur_loc[0] == x_max:
            pass
        else:
            my_drone2.forward(int(int(x_max-my_drone2.cur_loc[0])/angle_compa))
        print(my_drone2.cur_loc)
        my_drone2.ccw(angle)
        while my_drone2.cur_loc[1] > y_min:
            if (my_drone2.cur_loc[1] - pas) < y_min:
                my_drone2.back(int(my_drone2.cur_loc[1]-y_min))
            else:
                my_drone2.back(pas)
            print(my_drone2.cur_loc)
    else:
        my_drone2.forward(pas)
        print(my_drone2.cur_loc)
        if (my_drone2.cur_loc[1] + pas) > y_max:
            print("D2 running 2a")
            my_drone2.forward(int(y_max-my_drone2.cur_loc[1]))
            print(my_drone2.cur_loc)
            my_drone2.cw(angle)
        elif my_drone2.cur_loc[1] == y_max:
            print("D2 running 2b")
            my_drone2.cw(angle)
        elif (my_drone2.cur_loc[1] - pas) < y_min:
            print("D2 running 2c")
            my_drone2.forward(int(my_drone2.cur_loc[1]-y_min))
            print(my_drone2.cur_loc)
            my_drone2.ccw(angle)
        elif my_drone2.cur_loc[1] == y_min:
            print("D2 running 2d")
            my_drone2.ccw(angle)
        
# Strategie pour le 3e drone qui vole en zig-zag de gauche à droite
def Strategie3(my_drone2, x_min, x_max, y_min, y_max):
    print("Strategie3 is running")
    pas = 45
    angle = 175
    etat_curr = my_drone2.cur_loc
    etat_init = (0,0,81)
    if etat_init == etat_curr: 
        while my_drone2.cur_loc[0] < x_max:
            forward_lim2lim(my_drone2, pas, angle)
        print("Drone3 a fini de parcourir la salle.")
    else:
        print("Drone n'est pas a la position initiale!")
    
# Avancer de bas jusqu'au haut et de droite à gauche pour le 4e drone
def forward_lim2lim2(my_drone3, pas, angle):
    angle_degrees = abs(180 - angle)
    angle_radians = math.radians(angle_degrees)
    angle_compa = math.tan(angle_radians)
    
    if (my_drone3.cur_loc[0] - round(pas*angle_compa)) <= x_min:
        print("d3 running second condition!")
        if my_drone3.cur_loc[0] == x_min:
            pass
        else:
            my_drone3.forward(int(int(my_drone3.cur_loc[0]-x_min)/angle_compa))
        print(my_drone3.cur_loc)
        my_drone3.ccw(angle)
        while my_drone3.cur_loc[1] > y_min:
            if (my_drone3.cur_loc[1] - pas) < y_min:
                my_drone3.back(int(my_drone3.cur_loc[1]-y_min))
            else:
                my_drone3.back(pas)
            print(my_drone3.cur_loc)
    else:
        my_drone3.forward(pas)
        print(my_drone3.cur_loc)
        if (my_drone3.cur_loc[1] + pas) > y_max:
            my_drone3.forward(int(y_max-my_drone3.cur_loc[1]))
            print(my_drone3.cur_loc)
            my_drone3.cw(angle)
        elif my_drone3.cur_loc[1] == y_max:
            my_drone3.cw(angle)
        elif (my_drone3.cur_loc[1] - pas) < y_min:
            my_drone3.forward(int(my_drone3.cur_loc[1]-y_min))
            print(my_drone3.cur_loc)
            my_drone3.ccw(angle)
        elif my_drone3.cur_loc[1] == y_min:
            my_drone3.ccw(angle)
        
# Strategie pour le 4e drone qui parts de gauche à droite et puis vole en zig-zag de droite à gauche (l'inverse de strategie pour le 3e drone)
def Strategie4(my_drone3, x_min, x_max, y_min, y_max):
        print("Strategie4 is running")
        pas = 45
        angle = 185
        etat_curr = my_drone3.cur_loc
        etat_init = (0,0,81)
        if etat_init == etat_curr: 
            cur = my_drone3.cur_loc[0]
            while cur < x_max:
                if (cur + pas) > x_max:
                    my_drone3.right(int(x_max-cur))
                    cur = my_drone3.cur_loc[0]
                else:
                    my_drone3.right(pas)
                    cur = my_drone3.cur_loc[0]
                print(my_drone3.cur_loc)
            while my_drone3.cur_loc[0] > x_min:
                forward_lim2lim2(my_drone3, pas, angle)
            print("Drone4 a fini de parcourir la salle.")
                
        else:
            print("Drone n'est pas a la position initiale!")
    
# Plot de tous les drones sur un meme graphe de 3D
def plot_all_in_a_graph(d0, d1, d2, d3, color0, color1, color2, color3, e):
    ax = plt.figure().add_subplot(projection='3d')
    
    coords0_df = pd.DataFrame(d0.path_coors)
    coords1_df = pd.DataFrame(d1.path_coors)
    coords2_df = pd.DataFrame(d2.path_coors)
    coords3_df = pd.DataFrame(d3.path_coors)
    
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
    ax.plot(x0, y0, z0, color0, linestyle='dashed', linewidth=2, markersize=4, label="Drone Moves")
    ax.plot(x0, y0, z0, linewidth=e, alpha=.15)
    
    x1, y1, z1 = coords1_df[0], coords1_df[1], coords1_df[2]
    ax.plot(x1, y1, z1, color1, linestyle='dashed', linewidth=2, markersize=4, label="Drone Moves")
    ax.plot(x1, y1, z1, linewidth=e, alpha=.15)
    
    x2, y2, z2 = coords2_df[0], coords2_df[1], coords2_df[2]
    ax.plot(x2, y2, z2, color2, linestyle='dashed', linewidth=2, markersize=4, label="Drone Moves")
    ax.plot(x2, y2, z2, linewidth=e, alpha=.15)
    
    x3, y3, z3 = coords3_df[0], coords3_df[1], coords3_df[2]
    ax.plot(x3, y3, z3, color3, linestyle='dashed', linewidth=2, markersize=4, label="Drone Moves")
    ax.plot(x3, y3, z3, linewidth=e, alpha=.15)
     
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
    
    # Construct the full path to the file
    file_path = os.path.join(newpath2, "graph.png")

    # Save the graph to a file
    plt.savefig(file_path)
    

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

# Start all functions in separate threads
t1 = threading.Thread(target=Strategie1, args=(my_drone, x_max, x_min, y_max, y_min,))
t2 = threading.Thread(target=Strategie2, args=(my_drone1, x_min, x_max, y_min, y_max,))
t3 = threading.Thread(target=Strategie3, args=(my_drone2, x_min, x_max, y_min, y_max,))
t4 = threading.Thread(target=Strategie4, args=(my_drone3, x_min, x_max, y_min, y_max,))
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
my_drone.land('ro')
my_drone1.land('go')
my_drone2.land('co')
my_drone3.land('yo')

# Demontrer sur le graphe
plot_all_in_a_graph(my_drone, my_drone1, my_drone2, my_drone3, 'ro', 'go', 'co', 'yo', 15)
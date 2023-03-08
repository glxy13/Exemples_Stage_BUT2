# -*- coding: utf-8 -*-
"""
Created on Tue Jan 17 17:22:00 2023

@author: Asus
"""

# Creating a simulated drone object in Python:
# ```python
from tello_sim import Simulator
import time

# creer un repertoire Test avec un ID
# enregistrer les datas du drone + images + video
import os
import csv
count = 1
newpath0 = "../../Test_"+str(count) 
if not os.path.exists(newpath0):
    os.makedirs(newpath0)
newpath1 = "../../Test_"+str(count) + "/Datas" 
if not os.path.exists(newpath1):
    os.makedirs(newpath1)
newpath2 = "../../Test_"+str(count) + "/Images" 
if not os.path.exists(newpath2):
    os.makedirs(newpath2)
newpath3 = "../../Test_"+str(count) + "/Videos"
if not os.path.exists(newpath3):
    os.makedirs(newpath3)

my_drone = Simulator('D0')
my_drone1 = Simulator('D1')
my_drone2 = Simulator('D2')
# ![](/images/ready.png)

# ```python
my_drone.takeoff()
# record_data(my_drone, my_drone1, my_drone2)
my_drone1.takeoff()
# record_data(my_drone, my_drone1, my_drone2)
my_drone2.takeoff()
# record_data(my_drone, my_drone1, my_drone2)
# ![](/images/takeoff.png)

# ```python
my_drone.forward(100)
my_drone1.forward(160)
my_drone2.forward(50)
# ![](/images/forward.png)

# By default, the simulator plots a 25 cm error region in light blue around the flight path. For more advanced projects, you can override the default parameter by including the optional second error bar parameter in centimeters like ```forward(40, e=50)```. This is useful when you want to change the model (simulation) based on empirical (actual) testing.

# Rotation clockwise
# ```python
my_drone.cw(90)
# ```
# ![](/images/cw.png)

# ```python
my_drone.back(180)
# ```

my_drone.up(50)
my_drone.forward(50)
my_drone.right(50)
# ![](/images/forward_2.png)

# ```python
my_drone1.flip("r", e=50)
# ```
# ![](/images/flip.png)

# ```python
my_drone2.up(50)
my_drone2.forward(50)

# enregistrer dans un fichier CSV 

my_drone2.right(60)
# ```
# ![](/images/right.png)

# ```python
my_drone.land('ro')
my_drone1.land('go')
my_drone2.land('bo')
# ```
# ![](/images/land.png)

# Plot all values on a 3D graph
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, MaxNLocator
import plotly.express as px
from itertools import zip_longest

# plot de tt les drones sur un meme graphe - a modifier 
def plot_all_in_a_graph(d0, d1, d2, color0, color1, color2, e):
    ax = plt.figure().add_subplot(projection='3d')
    
    coords0_df = pd.DataFrame(d0.path_coors)
    coords1_df = pd.DataFrame(d1.path_coors)
    coords2_df = pd.DataFrame(d2.path_coors)
    
    xlow = min(coords0_df[0]) and min(coords1_df[0]) and min(coords2_df[0])
    xhi = max(coords0_df[0]) and max(coords1_df[0]) and max(coords2_df[0])
    ylow = min(coords0_df[1]) and min(coords1_df[1]) and min(coords2_df[1])
    yhi = max(coords0_df[1]) and max(coords1_df[1]) and max(coords2_df[1])
    zlow = min(coords0_df[2]) and min(coords1_df[2]) and min(coords2_df[2])
    zhi = max(coords0_df[2]) and max(coords1_df[2]) and max(coords2_df[2])
     
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
    
plot_all_in_a_graph(my_drone, my_drone1, my_drone2, 'ro', 'go', 'bo', 15)

# -*- coding: utf-8 -*-
"""
Created on Mon Feb 27 19:58:08 2023

@author: Asus
"""

# from djitellopy import Tello
from tello_sim import Simulator

my_drone = Simulator('d0')

# tello.takeoff()
my_drone.takeoff()

my_drone.forward(20)
my_drone.up(50)
my_drone.forward(20)
my_drone.right(40)

# tello.land()
my_drone.land()
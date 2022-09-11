#!/usr/bin/env python

#  REFERENCE: https://github.com/MichaelGrupp/evo/issues/20#issuecomment-368425480
# https://github.com/MichaelGrupp/evo/issues/20
# https://github.com/MichaelGrupp/evo/wiki/Formats#bag---ros-bagfile
# https://github.com/MichaelGrupp/evo/blob/master/evo/tools/plot.py

#  PUT EVO LICENCE HERE AND SAY MODIFIED


import sys
import os
import numpy as np
import seaborn as sns

import math

import collections
import logging
import pickle
import typing
from enum import Enum

import matplotlib.cm as cm
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.art3d as art3d
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backend_bases import FigureCanvasBase
from matplotlib.collections import LineCollection
from matplotlib.transforms import Affine2D
import matplotlib.cm as cm
from matplotlib.patches import Rectangle

# Reference: https://github.com/superjax/plotWindow
from plotWindow.plotWindow import plotWindow




number_of_parameters = len(sys.argv)-1
#  Declaring arrays to plot

legend_list = [None] * number_of_parameters
legend_index = [0] * number_of_parameters


speed_list = [0.0] * number_of_parameters
time_list = [0.0] * number_of_parameters

for i in range(len(legend_index)):
    legend_index[i] = i



for parameter_idx in range(number_of_parameters):
    est_file_name = sys.argv[parameter_idx+1]

    # print(est_file_name)

    # File name examples:
    # KITTI_06_cyl_0_2_100_ff
    # KITTI_06_rad_0_20_ff
    # KITTI_06_vanilla_ff


    results = {
        "title": "",
        "time": [],
        "speed": []
    }


    # print(thisdict)
    keys_list = ['title', 'time', 'speed']

    with open(str(est_file_name + ".txt")) as f:
        lines = f.readlines()
        i = 0
        for line in lines:
            text = line.split(',')
            # print(line)
            results["title"] = est_file_name
            results["time"].append((float(text[0].strip('\n'))) - 1317384506.40)
            results["speed"].append(float(text[1].strip('\n')))
            i = i + 1

    # print(translation)
    # print(rotation)
    # print(points)

    Dataset = "_".join(est_file_name.split("_")[:2])
    # filter_name = "_".join(est_file_name.split("_")[2:])
    filter_name = ""
    filter_name_found = False
    if ("vanilla" in est_file_name.split("_")[2:]):
        filter_name = filter_name + "Vanilla" + "\n"
        filter_name_found = True

    if ("rad" in est_file_name.split("_")[2:]):
        min_rad_index = est_file_name.split("_")[2:].index('rad') + 1
        min_rad = est_file_name.split("_")[2:][min_rad_index]
        max_rad_index = est_file_name.split("_")[2:].index('rad') + 2
        max_rad = est_file_name.split("_")[2:][max_rad_index]
        filter_name = filter_name + "Radius\n" + " Min: " + min_rad + " Max: " + max_rad + "\n"
        filter_name_found = True

    if ("cyl" in est_file_name.split("_")[2:]):
        origin_index = est_file_name.split("_")[2:].index('cyl') + 1
        origin = est_file_name.split("_")[2:][origin_index]
        radius_index = est_file_name.split("_")[2:].index('cyl') + 2
        radius = est_file_name.split("_")[2:][radius_index]
        height_index = est_file_name.split("_")[2:].index('cyl') + 3
        height = est_file_name.split("_")[2:][height_index]
        filter_name = filter_name + "Cylinder\n" + " O: " + origin + " R: " + radius + " H: " + height + "\n"
        filter_name_found = True

    if (not filter_name_found):
        filter_name = "_".join(est_file_name.split("_")[2:-1]) + "\n"

    if ("ff" in est_file_name.split("_")[2:]):
        filter_name = filter_name + "W/O Floor"

    legend_list[parameter_idx] = filter_name

    time_list[parameter_idx] = results["time"]
    speed_list[parameter_idx] = results["speed"]


#############################################################################################
#############################################################################################
#############################################################################################

def createFigure():
    # Creating the figure
    fig = plt.figure()  
    # Setting the background color of the plot 
    # using set_facecolor() method
    # Link with list with colours: https://matplotlib.org/stable/gallery/color/named_colors.html
    ax = plt.axes()
    ax.set_facecolor("seashell")
    

    fig.patch.set_facecolor('gainsboro')
    # Setting the axis scales
    plt.yscale('linear')
    plt.xscale('linear')
    plt.ticklabel_format(style='plain',useOffset=False)
    
    # plt.xticks(range(len(legend_list)),legend_list)
    # ax.set_xticklabels(ax.get_xticklabels(), rotation=45)
    # Adding a grid
    plt.grid()
    return fig, plt


#############################################################################################
#############################################################################################
#############################################################################################

# print(legend_list)

# Creating a plot window to compare data for translation
pw_results = plotWindow()

pw_results.MainWindow.setWindowTitle("SpeedVSTime")


# Creating the plot
fig, plt = createFigure()

for i in range(number_of_parameters):
    plt.plot(time_list[i], speed_list[i], linewidth=2, label = legend_list[i])
plt.xlabel("Time (s)", fontsize=13, fontweight='bold')
plt.ylabel("Speed (m/s)", fontsize=13, fontweight='bold')
plt.title("Speed vs Time Chart with " + Dataset, fontsize=15, fontweight='bold')
plt.legend(loc='upper left')
pw_results.addPlot("Speed Vs Time", fig)

pw_results.show()


#############################################################################################
#############################################################################################
#############################################################################################


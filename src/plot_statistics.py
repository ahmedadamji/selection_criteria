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


# This loop itterates through the statistics of each of the files provided in the arguments list
# This can be used to save the metrics for each in an array that can be used to plot
# Name of the plot can be the metric and legend of the plot can be the parameter used

#  Declaring arrays to plot

legend_list = [None] * number_of_parameters
legend_index = [0] * number_of_parameters

for i in range(len(legend_index)):
    legend_index[i] = i

std_trans_list = [0.0] * number_of_parameters
rmse_trans_list = [0.0] * number_of_parameters
max_trans_list = [0.0] * number_of_parameters
min_trans_list = [0.0] * number_of_parameters
median_trans_list = [0.0] * number_of_parameters
sse_trans_list = [0.0] * number_of_parameters
mean_trans_list = [0.0] * number_of_parameters


std_rot_list = [0.0] * number_of_parameters
rmse_rot_list = [0.0] * number_of_parameters
max_rot_list = [0.0] * number_of_parameters
min_rot_list = [0.0] * number_of_parameters
median_rot_list = [0.0] * number_of_parameters
sse_rot_list = [0.0] * number_of_parameters
mean_rot_list = [0.0] * number_of_parameters


total_input_points_list = [0.0] * number_of_parameters
total_output_points_list = [0.0] * number_of_parameters
total_filtered_points_list = [0.0] * number_of_parameters
average_input_points_list = [0.0] * number_of_parameters
average_output_points_list = [0.0] * number_of_parameters
average_filtered_points_list = [0.0] * number_of_parameters

mean_translational_error_per_filtered_point_list = [0.0] * number_of_parameters
mean_rotational_error_per_filtered_point_list = [0.0] * number_of_parameters


for parameter_idx in range(number_of_parameters):
    est_file_name = sys.argv[parameter_idx+1]

    # print(est_file_name)

    # File name examples:
    # KITTI_06_cyl_0_2_100_ff
    # KITTI_06_rad_0_20_ff
    # KITTI_06_vanilla_ff

    


    translation = {
        "title": "",
        "std": 0.0,
        "rmse": 0.0,
        "max": 0.0,
        "min": 0.0,
        "median": 0.0,
        "sse": 0.0,
        "mean": 0.0
    }

    rotation = {
        "title": "",
        "std": 0.0,
        "rmse": 0.0,
        "max": 0.0,
        "min": 0.0,
        "median": 0.0,
        "sse": 0.0,
        "mean": 0.0
    }

    points = {
        "total_input_points": 0.0,
        "total_output_points": 0.0,
        "total_filtered_points": 0.0,
        "average_input_points": 0.0,
        "average_output_points": 0.0,
        "average_filtered_points": 0.0
    }

    additional_metrics = {
        "mean_translational_error_per_filtered_point": 0.0,
        "mean_rotational_error_per_filtered_point": 0.0
    }

    # print(thisdict)
    keys_list = ['title', 'std', 'rmse', 'max', 'min', 'median', 'sse', 'mean']
    points_keys_list = ['total_input_points', 'total_output_points', 'total_filtered_points', 'average_input_points', 'average_output_points', 'average_filtered_points']
    additional_metrics_keys_list = ['mean_translational_error_per_filtered_point','mean_rotational_error_per_filtered_point']

    with open(str(est_file_name + "_statistics.txt")) as f:
        lines = f.readlines()
        i = 0
        for line in lines:
            dictionary = line.split(' ')[0]
            text = line.split(' ')[1:]
            # print(line)
            if (i <= 7):
                a_key = keys_list[i]
                if (i == 0):
                    translation[a_key] = line.strip('\n')
                else:
                    translation[a_key] = float(text[0].strip('\n'))
            elif (i >= 9):
                a_key = keys_list[i-9]
                if (i == 9):
                    rotation[a_key] = line.strip('\n')
                else:
                    rotation[a_key] = float(text[0].strip('\n'))
            i = i + 1

    with open("points/"+str(est_file_name + ".txt")) as f:
        lines = f.readlines()
        i = 0
        for line in lines:
            # split by space and converting
            # string to list and
            sentence = list(line.split(" "))
            # length of sentence
            length = len(sentence)
            # returning last element in sentence
            dictionary = " ".join(sentence[:length-1])
            num_points = sentence[length-1]
            # print(dictionary)
            # print(points)
            a_key = points_keys_list[i]
            points[a_key] = float(num_points.strip('\n'))
            i = i + 1
    
    if points["average_filtered_points"] > 0:
        additional_metrics['mean_translational_error_per_filtered_point'] = translation['mean'] / points["average_filtered_points"]
        additional_metrics['mean_rotational_error_per_filtered_point'] = rotation['mean'] / points["average_filtered_points"]
    else:
        additional_metrics['mean_translational_error_per_filtered_point'] = 0.0
        additional_metrics['mean_rotational_error_per_filtered_point'] = 0.0


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

    std_trans_list[parameter_idx] = translation["std"]
    rmse_trans_list[parameter_idx] = translation["rmse"]
    max_trans_list[parameter_idx] = translation["max"]
    min_trans_list[parameter_idx] = translation["min"]
    median_trans_list[parameter_idx] = translation["median"]
    sse_trans_list[parameter_idx] = translation["sse"]
    mean_trans_list[parameter_idx] = translation["mean"]

    std_rot_list[parameter_idx] = rotation["std"]
    rmse_rot_list[parameter_idx] = rotation["rmse"]
    max_rot_list[parameter_idx] = rotation["max"]
    min_rot_list[parameter_idx] = rotation["min"]
    median_rot_list[parameter_idx] = rotation["median"]
    sse_rot_list[parameter_idx] = rotation["sse"]
    mean_rot_list[parameter_idx] = rotation["mean"]

    total_input_points_list[parameter_idx] = points["total_input_points"]
    total_output_points_list[parameter_idx] = points["total_output_points"]
    total_filtered_points_list[parameter_idx] = points["total_filtered_points"]
    average_input_points_list[parameter_idx] = points["average_input_points"]
    average_output_points_list[parameter_idx] = points["average_output_points"]
    average_filtered_points_list[parameter_idx] = points["average_filtered_points"]

    mean_translational_error_per_filtered_point_list[parameter_idx] = additional_metrics["mean_translational_error_per_filtered_point"]
    mean_rotational_error_per_filtered_point_list[parameter_idx] = additional_metrics["mean_rotational_error_per_filtered_point"]



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
    plt.ticklabel_format(style='plain',useOffset=False)
    plt.xticks(range(len(legend_list)),legend_list)
    # ax.set_xticklabels(ax.get_xticklabels(), rotation=45)
    # Adding a grid
    plt.grid()
    return fig, plt


#############################################################################################
#############################################################################################
#############################################################################################

# print(legend_list)

# Creating a plot window to compare data for translation
pw_trans = plotWindow()

pw_trans.MainWindow.setWindowTitle("Translation Statistics")

# Creating the plot
fig, plt = createFigure()
plt.plot(std_trans_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Standard Deviation in meters (m)")
plt.title("STD Comparison With " + Dataset)
pw_trans.addPlot("std", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(rmse_trans_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("RMSE in meters (m)")
plt.title("RMSE Comparison With " + Dataset)
pw_trans.addPlot("rmse", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(max_trans_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Max Error in meters (m)")
plt.title("Max Error Comparison With " + Dataset)
pw_trans.addPlot("max", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(min_trans_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Min error in meters (m)")
plt.title("Min Error Comparison With " + Dataset)
pw_trans.addPlot("min", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(median_trans_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Median error in meters (m)")
plt.title("Median Error Comparison With " + Dataset)
pw_trans.addPlot("median", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(sse_trans_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("SSE error in meters (m)")
plt.title("SSE Error Comparison With " + Dataset)
pw_trans.addPlot("sse", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(mean_trans_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Mean error in meters (m)")
plt.title("Mean Error Comparison With " + Dataset)
pw_trans.addPlot("mean", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(total_input_points_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Total Input Points")
plt.title("Total Input Points Comparison With " + Dataset)
pw_trans.addPlot("total_input_points", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(total_filtered_points_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Total Filtered Points")
plt.title("Total Filtered Points Comparison With " + Dataset)
pw_trans.addPlot("total_filtered_points", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(average_input_points_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Average Input Points")
plt.title("Average Input Points Comparison With " + Dataset)
pw_trans.addPlot("average_input_points", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(average_filtered_points_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Average Filtered Points")
plt.title("Average Filtered Points Comparison With " + Dataset)
pw_trans.addPlot("average_filtered_points", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(mean_translational_error_per_filtered_point_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Mean Translational Error Per Filtered Point in meters (m)")
plt.title("Mean Translational Error Per Filtered Point Comparison With " + Dataset)
pw_trans.addPlot("mean_translational_error_per_filtered_point", fig)

pw_trans.show()


#############################################################################################
#############################################################################################
#############################################################################################


# Creating a plot window to compare data for rotation
pw_rot = plotWindow()

pw_rot.MainWindow.setWindowTitle("Rotation Statistics")


# Creating the plot
fig, plt = createFigure()
plt.plot(std_rot_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Standard Deviation in degrees (deg)")
plt.title("STD Comparison With " + Dataset)
pw_rot.addPlot("std", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(rmse_rot_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("RMSE in degrees (deg)")
plt.title("RMSE Error Comparison With " + Dataset)
pw_rot.addPlot("rmse", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(max_rot_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Max error in degrees (deg)")
plt.title("Max Error Comparison With " + Dataset)
pw_rot.addPlot("max", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(min_rot_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Min error in degrees (deg)")
plt.title("Min Error Comparison With " + Dataset)
pw_rot.addPlot("min", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(median_rot_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Median error in degrees (deg)")
plt.title("Median Error Comparison With " + Dataset)
pw_rot.addPlot("median", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(sse_rot_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("SSE error in degrees (deg)")
plt.title("SSE Error Comparison With " + Dataset)
pw_rot.addPlot("sse", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(mean_rot_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Mean error in degrees (deg)")
plt.title("Mean Error Comparison With " + Dataset)
pw_rot.addPlot("mean", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(total_input_points_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Total Input Points")
plt.title("Total Input Points Comparison With " + Dataset)
pw_rot.addPlot("total_input_points", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(total_filtered_points_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Total Filtered Points")
plt.title("Total Filtered Points Comparison With " + Dataset)
pw_rot.addPlot("total_filtered_points", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(average_input_points_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Average Input Points")
plt.title("Average Input Points Comparison With " + Dataset)
pw_rot.addPlot("average_input_points", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(average_filtered_points_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Average Filtered Points")
plt.title("Average Filtered Points Comparison With " + Dataset)
pw_rot.addPlot("average_filtered_points", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(mean_rotational_error_per_filtered_point_list, '--')
plt.xlabel("Selection Algorithm")
plt.ylabel("Mean Rotational Error Per Filtered Point in degrees (deg)")
plt.title("Mean Translational Error Per Filtered Point Comparison With " + Dataset)
pw_rot.addPlot("mean_rotational_error_per_filtered_point", fig)

pw_rot.show()

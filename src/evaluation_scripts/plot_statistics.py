#!/usr/bin/env python



# MIT License

# Copyright (c) 2022 Ahmed Adamjee

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


# The tools provided by the Evo package (github.com/MichaelGrupp/evo.) was used in this repository to compute APE metrics



from cProfile import label
import sys
import os
import numpy as np
import seaborn as sns
import matplotlib as mpl

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
z_trans_list = [0.0] * number_of_parameters


std_rot_list = [0.0] * number_of_parameters
rmse_rot_list = [0.0] * number_of_parameters
max_rot_list = [0.0] * number_of_parameters
min_rot_list = [0.0] * number_of_parameters
median_rot_list = [0.0] * number_of_parameters
sse_rot_list = [0.0] * number_of_parameters
mean_rot_list = [0.0] * number_of_parameters
z_rot_list = [0.0] * number_of_parameters


total_input_points_list = [0.0] * number_of_parameters
total_output_points_list = [0.0] * number_of_parameters
total_filtered_points_list = [0.0] * number_of_parameters
average_input_points_list = [0.0] * number_of_parameters
average_output_points_list = [0.0] * number_of_parameters
average_filtered_points_list = [0.0] * number_of_parameters
percentage_of_cloud_filtered_list = [0.0] * number_of_parameters

# Here from mean can change this to the metric that I identified from literature that is most useful.
# Also they had created their own metric, can calculate this and add it here as well.

# Can replace this metric where the denominator is normaized with respect to total number of points, but that would only change the scale -->
scaled_mean_translational_Z_score_per_filtered_cloud_list = [0.0] * number_of_parameters
scaled_mean_rotational_Z_score_per_filtered_cloud_list = [0.0] * number_of_parameters
scaled_average_error_z_score_per_filtered_cloud_list = [0.0] * number_of_parameters
# Probably create graph that compares gradient of error per filtered cloud as well.


# Z score Translation
mean_of_means_trans = 1.03054728218
std_of_means_trans = 0.00582771126119
# Z score Rotation
mean_of_means_rot = 0.483219724822
std_of_means_rot = 0.00661430552111


time_list = [0.0] * number_of_parameters


for parameter_idx in range(number_of_parameters):
    est_file_name = sys.argv[parameter_idx+1]
    dataset = "KITTI"
    sequence = "06"
    sub_folder = ""

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
        "average_filtered_points": 0.0,
        "percentage_of_cloud_filtered": 0.0
    }

    additional_metrics = {
        "scaled_average_error_z_score_per_filtered_cloud": 0.0,
        "time": 0.0
    }

    # print(thisdict)
    keys_list = ['title', 'std', 'rmse', 'max', 'min', 'median', 'sse', 'mean']
    points_keys_list = ['total_input_points', 'total_output_points', 'total_filtered_points', 'average_input_points', 'average_output_points', 'average_filtered_points','percentage_of_cloud_filtered']
    additional_metrics_keys_list = ['time']

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
            elif (i >= 9) and (i<=16):
                a_key = keys_list[i-9]
                if (i == 9):
                    rotation[a_key] = line.strip('\n')
                else:
                    rotation[a_key] = float(text[0].strip('\n'))
            elif (i == 19):
                a_key = additional_metrics_keys_list[0]
                additional_metrics[a_key] = float(text[0].strip('\n'))
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
        points["percentage_of_cloud_filtered"] = (100*points["average_filtered_points"]/points["average_input_points"])


    else:
        points["percentage_of_cloud_filtered"] = 0.0
    


    # print(translation)
    # print(rotation)
    # print(points)

    Dataset = "_".join(est_file_name.split("_")[:2])
    # filter_name = "_".join(est_file_name.split("_")[2:])
    filter_name = ""
    filter_name_found = False
    # Will need to improve the labeling of the vanilla later

    if ("vanilla" in est_file_name.split("_")[2:]):
        filter_name = filter_name + "Vanilla" + "\n"
        
        filter_name_found = True



    floor_indexes = [i for i in range(len(est_file_name.split("_")[2:])) if est_file_name.split("_")[2:][i] == 'ff']
    for index in floor_indexes:
        if ((len(est_file_name.split("_")[2:]) == 3) and  (not("beta" in est_file_name.split("_")[2:]))):
            min_rad_index = index + 1
            min_rad = est_file_name.split("_")[2:][min_rad_index]
            max_rad_index = index + 2
            max_rad = est_file_name.split("_")[2:][max_rad_index]
            filter_name = filter_name + "Floor\n" + " Min: " + min_rad + " Max: " + max_rad + "\n"
            filter_name_found = True


    if ("ang" in est_file_name.split("_")[2:]):
        min_angle_index = est_file_name.split("_")[2:].index('ang') + 2
        min_angle = est_file_name.split("_")[2:][min_angle_index]
        max_angle_index = est_file_name.split("_")[2:].index('ang') + 3
        max_angle = est_file_name.split("_")[2:][max_angle_index]
        filter_name = filter_name + "Angle Dev\n" + " Min: " + min_angle + "\n" + " Max: " + max_angle + "\n"
        filter_name_found = True


    if ("beta" in est_file_name.split("_")[2:]):
        std_index = est_file_name.split("_")[2:].index('beta') + 1
        std = est_file_name.split("_")[2:][std_index]
        filter_name = filter_name + "Beta\n" + " $Z_{beta}$: " + std + "\n"
        filter_name_found = True

    rad_indexes = [i for i in range(len(est_file_name.split("_")[2:])) if est_file_name.split("_")[2:][i] == 'rad']
    for index in rad_indexes:
        min_rad_index = index + 1
        min_rad = est_file_name.split("_")[2:][min_rad_index]
        max_rad_index = index + 2
        max_rad = est_file_name.split("_")[2:][max_rad_index]
        filter_name = filter_name + "Radius\n" + " Min: " + min_rad + " Max: " + max_rad + "\n"
        filter_name_found = True


    cyl_indexes = [i for i in range(len(est_file_name.split("_")[2:])) if est_file_name.split("_")[2:][i] == 'cyl']
    for index in cyl_indexes:
        origin_index = index + 1
        origin = est_file_name.split("_")[2:][origin_index]
        radius_index = index + 2
        radius = est_file_name.split("_")[2:][radius_index]
        height_index = index + 3
        height = est_file_name.split("_")[2:][height_index]
        filter_name = filter_name + "Cylinder\n" + " O: " + origin + " R: " + radius + " H: " + height + "\n"
        filter_name_found = True


    if (not filter_name_found):
        filter_name = "_".join(est_file_name.split("_")[2:-1]) + "\n"


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
    percentage_of_cloud_filtered_list[parameter_idx] = points["percentage_of_cloud_filtered"]


    time_list[parameter_idx] = additional_metrics["time"]

# The mean and std I use here should be the mean and std from multiple runs of the vanilla itself:
# Translational Statistics for mean error: 
# mean_of_means_trans: 
# 1.03054728218
# std_of_means_trans: 
# 0.00582771126119
# Rotational Statistics for mean error: 
# mean_of_means_rot: 
# 0.483219724822
# std_of_means_rot: 
# 0.00661430552111
# Based on: https://www.investopedia.com/terms/c/central_limit_theorem.asp I need to  have a sample size of 30

z_trans_list = np.subtract(mean_trans_list,mean_of_means_trans)/std_of_means_trans
z_rot_list = np.subtract(mean_rot_list,mean_of_means_rot)/std_of_means_rot



for parameter_idx in range(number_of_parameters):
    if (legend_list[parameter_idx] == 'vanilla'):
        scaled_mean_translational_Z_score_per_filtered_cloud_list[parameter_idx] = 1
    else:
        scaled_mean_translational_Z_score_per_filtered_cloud_list[parameter_idx] = (z_trans_list[parameter_idx]/np.max(z_trans_list)) / (np.array(percentage_of_cloud_filtered_list[parameter_idx])/100)


for parameter_idx in range(number_of_parameters):
    if (legend_list[parameter_idx] == 'vanilla'):
        scaled_mean_rotational_Z_score_per_filtered_cloud_list[parameter_idx] = 1
    else:
        scaled_mean_rotational_Z_score_per_filtered_cloud_list[parameter_idx] = (z_rot_list[parameter_idx]/np.max(z_rot_list)) / (np.array(percentage_of_cloud_filtered_list[parameter_idx])/100)
  

for parameter_idx in range(number_of_parameters):

    if (legend_list[parameter_idx] == 'vanilla'):
        scaled_average_error_z_score_per_filtered_cloud_list[parameter_idx] = 1
    else:
        scaled_average_error_z_score_per_filtered_cloud_list[parameter_idx] = (((z_trans_list[parameter_idx]/np.max(z_trans_list))+(z_rot_list[parameter_idx]/np.max(z_rot_list)))) / (2*(percentage_of_cloud_filtered_list[parameter_idx])/100)
    




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
    ax.set_facecolor('white')
    ax.patch.set_edgecolor('black')
    ax.patch.set_linewidth('3')  
    # plt.style.context("seaborn-whitegrid")
    # fig.patch.set_facecolor('gainsboro')

    # Setting the axis scales
    plt.rc('xtick', labelsize=16)
    plt.rc('ytick', labelsize=16)
    mpl.rc('xtick', labelsize=16)
    mpl.rc('ytick', labelsize=16)
    plt.yscale('linear')
    plt.ticklabel_format(style='plain',useOffset=False)
    plt.xticks(range(len(legend_list)),legend_list)
    # ax.set_xticklabels(ax.get_xticklabels(), rotation=45)
    # Adding a grid
    # plt.grid(color='black', linestyle='--', linewidth=0.5)
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
for i in range(number_of_parameters):
    plt.plot()
plt.xlabel("", fontsize=18, fontweight='bold')
plt.ylabel("", fontsize=18, fontweight='bold')
pw_trans.addPlot("", fig)

# Creating the plot
fig, plt = createFigure()
plt.plot(std_trans_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Standard Deviation in meters (m)", fontsize=18, fontweight='bold')
# plt.title("STD Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_trans.addPlot("std", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(rmse_trans_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("RMSE in meters (m)", fontsize=18, fontweight='bold')
# plt.title("RMSE Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_trans.addPlot("rmse", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(max_trans_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Max Error in meters (m)", fontsize=18, fontweight='bold')
# plt.title("Max Error Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_trans.addPlot("max", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(min_trans_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Min error in meters (m)", fontsize=18, fontweight='bold')
# plt.title("Min Error Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_trans.addPlot("min", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(median_trans_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Median error in meters (m)", fontsize=18, fontweight='bold')
# plt.title("Median Error Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_trans.addPlot("median", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(sse_trans_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("SSE error in meters (m)", fontsize=18, fontweight='bold')
# plt.title("SSE Error Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_trans.addPlot("sse", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(mean_trans_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Mean error in meters (m)", fontsize=18, fontweight='bold')
# plt.title("Mean Error Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_trans.addPlot("mean", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(z_trans_list, '--', marker='o')
# Translational Statistics for mean error: 
# mean_of_means: 
# 1.03054728218
# std_of_means: 
# 0.00582771126119

# plt.Rectangle((0,0),1000,2,label='Region of unsignificance', color='lightblue')
plt.plot(([3] * number_of_parameters), '-.', color='red', label='3 $\sigma$')
# plt.plot(([-3] * number_of_parameters), '-.', color='red', label='3 $\sigma$')
plt.plot(([2] * number_of_parameters), '--', color='orange', label='2 $\sigma$')
# plt.plot(([-2] * number_of_parameters), '--', color='orange', label='2 $\sigma$')
plt.plot(([1] * number_of_parameters), '-', color='green', label='1 $\sigma$')
# plt.plot(([-1] * number_of_parameters), '-', color='green', label='1 $\sigma$')
ax = plt.axes()
ax.legend()
plt.legend(loc='upper left', fontsize=18)
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Z score", fontsize=18, fontweight='bold')
# plt.title("Z score Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_trans.addPlot("Z", fig)



# pw_trans.show()


#############################################################################################
#############################################################################################
#############################################################################################


# Creating a plot window to compare data for rotation
pw_rot = plotWindow()

pw_rot.MainWindow.setWindowTitle("Rotation Statistics")


# Creating the plot
fig, plt = createFigure()
plt.plot(std_rot_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Standard Deviation in degrees (deg)", fontsize=18, fontweight='bold')
# plt.title("STD Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_rot.addPlot("std", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(rmse_rot_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("RMSE in degrees (deg)", fontsize=18, fontweight='bold')
# plt.title("RMSE Error Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_rot.addPlot("rmse", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(max_rot_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Max error in degrees (deg)", fontsize=18, fontweight='bold')
# plt.title("Max Error Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_rot.addPlot("max", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(min_rot_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Min error in degrees (deg)", fontsize=18, fontweight='bold')
# plt.title("Min Error Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_rot.addPlot("min", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(median_rot_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Median error in degrees (deg)", fontsize=18, fontweight='bold')
# plt.title("Median Error Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_rot.addPlot("median", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(sse_rot_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("SSE error in degrees (deg)", fontsize=18, fontweight='bold')
# plt.title("SSE Error Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_rot.addPlot("sse", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(mean_rot_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Mean error in degrees (deg)", fontsize=18, fontweight='bold')
# plt.title("Mean Error Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_rot.addPlot("mean", fig)


# Creating the plot
fig, plt = createFigure()
plt.plot(z_rot_list, '--', marker='o')
# Rotational Statistics for mean error: 
# mean_of_means: 
# 0.483219724822
# std_of_means: 
# 0.00661430552111
# plt.Rectangle((0,0),1000,2,label='Region of unsignificance', color='lightblue')
plt.plot(([3] * number_of_parameters), '-.', color='red', label='3 $\sigma$')
# plt.plot(([-3] * number_of_parameters), '-.', color='red', label='3 $\sigma$')
plt.plot(([2] * number_of_parameters), '--', color='orange', label='2 $\sigma$')
# plt.plot(([-2] * number_of_parameters), '--', color='orange', label='2 $\sigma$')
plt.plot(([1] * number_of_parameters), '-', color='green', label='1 $\sigma$')
# plt.plot(([-1] * number_of_parameters), '-', color='green', label='1 $\sigma$')
ax = plt.axes()
ax.legend()
plt.legend(loc='upper left', fontsize=18)
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Z score", fontsize=18, fontweight='bold')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Z score", fontsize=18, fontweight='bold')
# plt.title("Z score Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_rot.addPlot("Z", fig)


# pw_rot.show()


#############################################################################################
#############################################################################################
#############################################################################################

# print(legend_list)

# Creating a plot window to compare data for translation
pw_additional = plotWindow()

pw_additional.MainWindow.setWindowTitle("Additional Statistics")


# Creating the plot
fig, plt = createFigure()
plt.plot(percentage_of_cloud_filtered_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Percentage of Points Filtered", fontsize=18, fontweight='bold')
# plt.title("Percentage of Cloud Filtered Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_additional.addPlot("percentage_of_cloud_filtered", fig)




# Creating the plot
# This plot expresses the metric of scaled sum of squares of normalized mean errors of translation and rotation in relation to the percentage of points filtered to give an overall measurement
fig, plt = createFigure()
plt.plot(scaled_mean_translational_Z_score_per_filtered_cloud_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Scaled Mean Translational Z Score Per Filtered Points", fontsize=18, fontweight='bold')
# plt.title("Overall Rotation and Translation Performance In Relation To Filtered Points Advantage Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_additional.addPlot("scaled_mean_translational_Z_score_per_filtered_cloud_list", fig)



# Creating the plot
# This plot expresses the metric of scaled sum of squares of normalized mean errors of translation and rotation in relation to the percentage of points filtered to give an overall measurement
fig, plt = createFigure()
plt.plot(scaled_mean_rotational_Z_score_per_filtered_cloud_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Scaled Mean Rotational Z Score Per Filtered Points", fontsize=18, fontweight='bold')
# plt.title("Overall Rotation and Translation Performance In Relation To Filtered Points Advantage Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_additional.addPlot("scaled_mean_rotational_Z_score_per_filtered_cloud_list", fig)


# Creating the plot
# This plot expresses the metric of scaled sum of squares of normalized mean errors of translation and rotation in relation to the percentage of points filtered to give an overall measurement
fig, plt = createFigure()
plt.plot(scaled_average_error_z_score_per_filtered_cloud_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("$\kappa$", fontsize=18, fontweight='bold')
# plt.title("Overall Rotation and Translation Performance In Relation To Filtered Points Advantage Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_additional.addPlot("scaled_average_error_z_score_per_filtered_cloud", fig)



# Creating the plot
# This plot expresses the metric of error per portion of cloud that is filtered, instead of per point, which makes more sense intuitively
fig, plt = createFigure()
plt.plot(time_list, '--', marker='o')
plt.xlabel("Selection Algorithm", fontsize=18, fontweight='bold')
plt.ylabel("Total Localization Time (s)", fontsize=18, fontweight='bold')
# plt.title("Computation Time Comparison With " + Dataset, fontsize=15, fontweight='bold')
pw_additional.addPlot("time", fig)

pw_additional.show()
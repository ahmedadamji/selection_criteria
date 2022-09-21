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
# Reference for the line of best fit: np.polyfit and https://stackoverflow.com/a/31800660


import sys
import os
import numpy as np
import seaborn as sns

# importing pandas module 
import pandas as pd 

import math
# from __future__ import print_function


from evo import EvoException
from evo.core import trajectory, sync, metrics
from evo.tools import file_interface
from evo.tools import plot
from evo.tools import user

import collections
import logging
import pickle
import typing
from enum import Enum

import matplotlib as mpl
from evo.tools.settings import SETTINGS

mpl.use(SETTINGS.plot_backend)
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

# This is to plot speed vs errors
####################################################################
number_of_parameters = len(sys.argv)-1
#  Declaring arrays to plot

legend_list = [None] * number_of_parameters
legend_index = [0] * number_of_parameters


linear_velocity_list = [0.0] * number_of_parameters
angular_velocity_list = [0.0] * number_of_parameters
time_list = [0.0] * number_of_parameters
xyz_diff_magnitude_list = [0.0] * number_of_parameters

synced_linear_velocity_list = [0.0] * number_of_parameters
synced_angular_velocity_list = [0.0] * number_of_parameters
synced_time_list = [0.0] * number_of_parameters
synced_xyz_diff_magnitude_list = [0.0] * number_of_parameters
synced_rpy_diff_pitch_list = [0.0] * number_of_parameters

for i in range(len(legend_index)):
    legend_index[i] = i
#####################################################################


# Can use this code to find the path of the current working directory.
# print('getcwd:      ', os.getcwd())
# print('__file__:    ', __file__)

#  Finding statistics and saving results for all files provided in argument
for file_num in range(len(sys.argv)-1):

    est_file_name = sys.argv[file_num+1]
    dataset = "KITTI"
    sequence = "06"
    sub_folder = ""

    print("loading trajectories")
    traj_ref = file_interface.read_tum_trajectory_file("/root/catkin_ws/src/project_ws/catkin_ws/src/data/" + dataset + "/" + sequence + "/results/localization/" + sub_folder + sequence + "_gt_tum")
    traj_est = file_interface.read_tum_trajectory_file("/root/catkin_ws/src/project_ws/catkin_ws/src/data/" + dataset + "/" + sequence + "/results/localization/" + sub_folder + est_file_name + ".tum")

    print("registering and aligning trajectories")
    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est)
    traj_est = trajectory.align_trajectory(traj_est, traj_ref, correct_scale=False)

    traj_ref_orientations_r_p_y = np.array([[0,0,0]])
    rpy_ref = np.array([])

    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    for w,x,y,z in traj_ref.orientations_quat_wxyz:
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)* 180 / math.pi

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)* 180 / math.pi

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)* 180 / math.pi
        
        rpy_ref = np.array([roll,pitch,yaw])
        # traj_ref_orientations_r_p_y.append(rpy_ref)
        traj_ref_orientations_r_p_y = np.append(traj_ref_orientations_r_p_y, [rpy_ref], axis= 0)


    traj_est_orientations_r_p_y = np.array([[0,0,0]])
    rpy_est = np.array([])

    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    for w,x,y,z in traj_est.orientations_quat_wxyz:
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)* 180 / math.pi

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)* 180 / math.pi

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)* 180 / math.pi
        
        rpy_est = np.array([roll,pitch,yaw])
        # traj_est_orientations_r_p_y.append(rpy_est)
        traj_est_orientations_r_p_y = np.append(traj_est_orientations_r_p_y, [rpy_est], axis= 0)




    print("calculating rpy differences")
    rpy_diff = np.abs(traj_ref_orientations_r_p_y - traj_est_orientations_r_p_y)
    print(rpy_diff)

    rpy_diff_roll = rpy_diff[:,0]
    rpy_diff_pitch = rpy_diff[:,1]
    rpy_diff_yaw = rpy_diff[:,2]

    # #  CODE FOR FINDING INDIVIDUAL TRANSLATION DIFFERENCES:

    data = (traj_ref, traj_est)

    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)

    ape_metric.process_data(data)

    ape_statistics = ape_metric.get_all_statistics()

    print("mean:", ape_statistics["mean"])

    print("calculating xyz differences")
    import numpy as np
    xyz_diff = np.abs(traj_ref.positions_xyz - traj_est.positions_xyz)
    # print(xyz_diff)

    xyz_diff_x = xyz_diff[:,0]
    xyz_diff_y = xyz_diff[:,1]
    xyz_diff_z = xyz_diff[:,2]

    # Only finding for x and y as z does not matter
    xyz_diff_magnitude = np.sqrt(np.square(xyz_diff_x)+np.square(xyz_diff_y))
    # xyz_diff_magnitude_list[file_num] = xyz_diff_magnitude








    # Following code is to plot speed vs error:
    ####################################################################################################
    ####################################################################################################
    ####################################################################################################
    results = {
        "title": "",
        "time": [],
        "linear_velocity": [],
        "angular_velocity": []
    }


    # print(thisdict)
    keys_list = ['title', 'time', 'linear_velocity', 'angular_velocity']

    with open(str("statistics/time_and_speed/" + est_file_name + ".txt")) as f:
        lines = f.readlines()
        i = 0
        for line in lines:
            text = line.split(',')
            # print(line)
            results["title"] = est_file_name
            results["time"].append((float(text[0].strip('\n'))))
            results["linear_velocity"].append(float(text[1].strip('\n')))
            results["angular_velocity"].append(float(text[2].strip('\n')))
            i = i + 1


    Dataset = "_".join(est_file_name.split("_")[:2])
    # filter_name = "_".join(est_file_name.split("_")[2:])
    filter_name = ""
    filter_name_found = False
    if ("vanilla" in est_file_name.split("_")[2:]):
        if (len(est_file_name.split("_")[2:]) > 3):
            min_rad_index = est_file_name.split("_")[2:].index('vanilla') + 1
            min_rad = est_file_name.split("_")[2:][min_rad_index]
            max_rad_index = est_file_name.split("_")[2:].index('vanilla') + 2
            max_rad = est_file_name.split("_")[2:][max_rad_index]
            filter_name = filter_name + "Vanilla\n" + " Min: " + min_rad + " Max: " + max_rad + "\n"
        else:
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

    if ("ang" in est_file_name.split("_")[2:]):
        min_angle_index = est_file_name.split("_")[2:].index('ang') + 2
        min_angle = est_file_name.split("_")[2:][min_angle_index]
        max_angle_index = est_file_name.split("_")[2:].index('ang') + 3
        max_angle = est_file_name.split("_")[2:][max_angle_index]
        filter_name = filter_name + "Angle Dev\n" + " Min: " + min_angle + " Max: " + max_angle + "\n"
        filter_name_found = True

    if (not filter_name_found):
        filter_name = "_".join(est_file_name.split("_")[2:-1]) + "\n"

    if ("ff" in est_file_name.split("_")[2:]):
        filter_name = filter_name + "W/O Floor"

    legend_list[file_num] = str(file_num)

    time_list[file_num] = results["time"]
    linear_velocity_list[file_num] = results["linear_velocity"]
    angular_velocity_list[file_num] = results["angular_velocity"]


    # calling .searchsorted() method
    # result = series.searchsorted(value = val)

    # creating series
    time_series = pd.Series(np.float64(time_list[file_num]))
    linear_velocity_series = pd.Series(np.float64(linear_velocity_list[file_num]))
    angular_velocity_series = pd.Series(np.float64(angular_velocity_list[file_num]))
    index_list = np.array(time_series[time_series.searchsorted(np.float64(traj_est.timestamps))].index)
    synced_time_list[file_num] = np.array([time_series[i] for i in index_list])
    synced_linear_velocity_list[file_num] = np.array([linear_velocity_series[i] for i in index_list])
    synced_angular_velocity_list[file_num] = np.array([angular_velocity_series[i] for i in index_list])
    synced_xyz_diff_magnitude_list[file_num] = np.array([xyz_diff_magnitude[i] for i in range(len(index_list))])
    synced_rpy_diff_pitch_list[file_num] = np.array([rpy_diff_pitch[i] for i in range(len(index_list))])



    print(est_file_name)



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
    plt.xscale('linear')
    plt.ticklabel_format(style='plain',useOffset=False)
    

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
    plt.plot()
plt.xlabel("", fontsize=18, fontweight='bold')
plt.ylabel("", fontsize=18, fontweight='bold')
pw_results.addPlot("", fig)


# Creating the plot
fig, plt = createFigure()

for i in range(number_of_parameters):
    plt.xlim([0,115])
    plt.ylim([0,np.max(synced_linear_velocity_list)])
    plt.plot((synced_time_list[i]-1317384506.40), synced_linear_velocity_list[i], label = legend_list[i])
plt.xlabel("Time (s)", fontsize=18, fontweight='bold')
plt.ylabel("Linear Velocity (m/s)", fontsize=18, fontweight='bold')
# plt.title("Linear Velocity vs Time Chart with " + Dataset, fontsize=15, fontweight='bold')
# plt.legend(loc='upper left')
pw_results.addPlot("Linear Velocity vs Time", fig)


# Creating the plot
fig, plt = createFigure()

for i in range(number_of_parameters):
    plt.xlim([0,115])
    plt.ylim([0,np.max(synced_angular_velocity_list)])
    plt.plot((synced_time_list[i]-1317384506.40), synced_angular_velocity_list[i], label = legend_list[i])
plt.xlabel("Time (s)", fontsize=18, fontweight='bold')
plt.ylabel("Angular Velocity (deg/s)", fontsize=18, fontweight='bold')
# plt.title("Angular Velocity vs Time Chart with " + Dataset, fontsize=15, fontweight='bold')
# plt.legend(loc='upper left')
pw_results.addPlot("Angular Velocity vs Time", fig)



# Creating the plot
fig, plt = createFigure()


for i in range(number_of_parameters):
    plt.xlim([0,2.45])
    plt.ylim([0,25])
    plt.scatter(synced_xyz_diff_magnitude_list[i], synced_linear_velocity_list[i], s=2, label = str("Linear Velocity: " + legend_list[i]))
    x = synced_xyz_diff_magnitude_list[i]
    y = synced_linear_velocity_list[i]
    plt.plot(np.unique(x), np.poly1d(np.polyfit(x, y, 1))(np.unique(x)), label = str("Linear Velocity: " + legend_list[i]))

plt.ylabel("Velocity (m/s)", fontsize=18, fontweight='bold')
plt.xlabel("APE Translation (m)", fontsize=18, fontweight='bold')
# plt.title("Absolute Translational Error vs Velocity Chart with " + Dataset, fontsize=15, fontweight='bold')
# plt.legend(loc='upper left')
pw_results.addPlot("ATE vs Velocity", fig)



# Creating the plot
fig, plt = createFigure()

for i in range(number_of_parameters):
    plt.xlim([0,3.3])
    plt.ylim([0,25])
    plt.scatter(synced_rpy_diff_pitch_list[i], synced_linear_velocity_list[i], s=2, label = str("Linear Velocity: " + legend_list[i]))
    x = synced_rpy_diff_pitch_list[i]
    y = synced_linear_velocity_list[i]
    plt.plot(np.unique(x), np.poly1d(np.polyfit(x, y, 1))(np.unique(x)), label = str("Linear Velocity: " + legend_list[i]))

plt.ylabel("Velocity (m/s)", fontsize=18, fontweight='bold')
plt.xlabel("APE Rotation (deg)", fontsize=18, fontweight='bold')
# plt.title("Absolute Rotational Error vs Velocity Chart with " + Dataset, fontsize=15, fontweight='bold')
# plt.legend(loc='upper left')
pw_results.addPlot("ARE vs Velocity", fig)








pw_results.show()

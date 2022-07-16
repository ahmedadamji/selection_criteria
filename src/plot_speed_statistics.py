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

for i in range(len(legend_index)):
    legend_index[i] = i
#####################################################################


# Can use this code to find the path of the current working directory.
# print('getcwd:      ', os.getcwd())
# print('__file__:    ', __file__)

#  Finding statistics and saving results for all files provided in argument
for file_num in range(len(sys.argv)-1):

    est_file_name = sys.argv[file_num+1]

    print("loading trajectories")
    traj_ref = file_interface.read_tum_trajectory_file("/root/catkin_ws/src/project_ws/catkin_ws/src/data/KITTI/06/results/localization/06_gt_tum")
    traj_est = file_interface.read_tum_trajectory_file("/root/catkin_ws/src/project_ws/catkin_ws/src/data/KITTI/06/results/localization/" + est_file_name + ".tum")

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


    # print (np.size(xyz_diff_magnitude))
    # print (np.size(x))


    # for t_idx in range(len(x)):
        # print(x[t_idx])

    
    # # creating series
    # time_series = pd.Series(x)
    
    # # values to be inserted
    # val =[1317384617.16,1317384617.47,1317384617.57,1317384617.99,1317384618.09]
    
    # # calling .searchsorted() method
    # # result = time_series.searchsorted(value = val)
    
    # # display
    # print(time_series[time_series.searchsorted(val)])








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

    legend_list[file_num] = filter_name

    time_list[file_num] = results["time"]
    linear_velocity_list[file_num] = results["linear_velocity"]
    angular_velocity_list[file_num] = results["angular_velocity"]


    # calling .searchsorted() method
    # result = series.searchsorted(value = val)

    # creating series
    time_series = pd.Series(np.float64(time_list[file_num]))
    linear_velocity_series = pd.Series(np.float64(linear_velocity_list[file_num]))
    angular_velocity_series = pd.Series(np.float64(angular_velocity_list[file_num]))
    # find synced index list
    index_list = np.array(time_series[time_series.searchsorted(np.float64(traj_est.timestamps))].index)
    # print(index_list)
    synced_time_list[file_num] = np.array([time_series[i] for i in index_list])
    synced_linear_velocity_list[file_num] = np.array([linear_velocity_series[i] for i in index_list])
    synced_angular_velocity_list[file_num] = np.array([angular_velocity_series[i] for i in index_list])
    xyz_diff_magnitude_list[file_num] = np.array([xyz_diff_magnitude[i] for i in range(len(index_list))])
    # print(np.size(time_list))
    # print(synced_time_list)
    # print(np.float64(time_list))
    # print(np.float64(x))
    # print(len(synced_time_list[file_num]))
    # print(len(synced_linear_velocity_list[file_num]))
    # print(len(xyz_diff_magnitude))



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
    plt.plot((synced_time_list[i]-1317384506.40), synced_linear_velocity_list[i], linewidth=2, label = legend_list[i])
plt.xlabel("Time (s)", fontsize=13, fontweight='bold')
plt.ylabel("Linear Velocity (m/s)", fontsize=13, fontweight='bold')
plt.title("Linear Velocity vs Time Chart with " + Dataset, fontsize=15, fontweight='bold')
plt.legend(loc='upper left')
pw_results.addPlot("Linear Velocity vs Time", fig)


# Creating the plot
fig, plt = createFigure()

for i in range(number_of_parameters):
    plt.plot((synced_time_list[i]-1317384506.40), synced_angular_velocity_list[i], linewidth=2, label = legend_list[i])
plt.xlabel("Time (s)", fontsize=13, fontweight='bold')
plt.ylabel("Angular Velocity (m/s)", fontsize=13, fontweight='bold')
plt.title("Angular Velocity vs Time Chart with " + Dataset, fontsize=15, fontweight='bold')
plt.legend(loc='upper left')
pw_results.addPlot("Angular Velocity vs Time", fig)



# # Creating the plot
# fig, plt = createFigure()

# for i in range(number_of_parameters):
#     plt.plot((synced_time_list[i]-1317384506.40), xyz_diff_magnitude_list[i], linewidth=2, label = legend_list[i])
#     # plt.plot(synced_time_list[i], synced_linear_velocity_list[i], linewidth=2, label = legend_list[i])
# plt.xlabel("Time (s)", fontsize=13, fontweight='bold')
# plt.ylabel("ATE (m)", fontsize=13, fontweight='bold')
# plt.title("Absolute Translational Error vs Time Chart with " + Dataset, fontsize=15, fontweight='bold')
# plt.legend(loc='upper left')
# pw_results.addPlot("ATE vs Time", fig)



# # Creating the plot
# fig, plt = createFigure()

# for i in range(number_of_parameters):
#     plt.scatter(synced_linear_velocity_list[i], xyz_diff_magnitude_list[i], s=2, label = legend_list[i])
#     # plt.plot(synced_time_list[i], synced_linear_velocity_list[i], linewidth=2, label = legend_list[i])
# plt.xlabel("Linear Velocity (m/s)", fontsize=13, fontweight='bold')
# plt.ylabel("ATE (m)", fontsize=13, fontweight='bold')
# plt.title("Absolute Translational Error vs Linear Velocity Chart with " + Dataset, fontsize=15, fontweight='bold')
# plt.legend(loc='upper left')
# pw_results.addPlot("ATE vs Linear Velocity", fig)



# # Creating the plot
# fig, plt = createFigure()

# for i in range(number_of_parameters):
#     plt.scatter(synced_angular_velocity_list[i], xyz_diff_magnitude_list[i], s=2, label = legend_list[i])
#     # plt.plot(synced_time_list[i], synced_angular_velocity_list[i], linewidth=2, label = legend_list[i])
# plt.xlabel("Angular Velocity (m/s)", fontsize=13, fontweight='bold')
# plt.ylabel("ATE (m)", fontsize=13, fontweight='bold')
# plt.title("Absolute Translational Error vs Angular Velocity Chart with " + Dataset, fontsize=15, fontweight='bold')
# plt.legend(loc='upper left')
# pw_results.addPlot("ATE vs Angular Velocity", fig)


# Creating the plot
fig, plt = createFigure()

# BIG UP THE FACT THAT I MADE A LINE OF BEST FIT
# Reference for the line of best fit: np.polyfit and https://stackoverflow.com/a/31800660

for i in range(number_of_parameters):
    plt.scatter(xyz_diff_magnitude_list[i], synced_linear_velocity_list[i]/np.max(synced_linear_velocity_list[i]), s=2, label = str("Linear Velocity: " + legend_list[i]))
    x = xyz_diff_magnitude_list[i]
    y = synced_linear_velocity_list[i]/np.max(synced_linear_velocity_list[i])
    plt.plot(np.unique(x), np.poly1d(np.polyfit(x, y, 1))(np.unique(x)), label = str("Linear Velocity: " + legend_list[i]))
    plt.scatter(xyz_diff_magnitude_list[i], synced_angular_velocity_list[i]/np.max(synced_angular_velocity_list[i]), s=2, label = str("Angular Velocity: " + legend_list[i]))
    # plt.plot(synced_time_list[i], synced_linear_velocity_list[i], linewidth=2, label = legend_list[i])
    y = synced_angular_velocity_list[i]/np.max(synced_angular_velocity_list[i])
    plt.plot(np.unique(x), np.poly1d(np.polyfit(x, y, 1))(np.unique(x)), label = str("Angular Velocity: " + legend_list[i]))
plt.ylabel("Velocity", fontsize=13, fontweight='bold')
plt.xlabel("ATE (m)", fontsize=13, fontweight='bold')
plt.title("Absolute Translational Error vs Velocity Chart with " + Dataset, fontsize=15, fontweight='bold')
plt.legend(loc='upper left')
pw_results.addPlot("ATE vs Velocity", fig)




# # Creating the plot
# fig, plt = createFigure()

# for i in range(number_of_parameters):
#     plt.plot((synced_time_list[i]-1317384506.40), rpy_diff_pitch[:np.size(synced_time_list[i])], linewidth=2, label = legend_list[i])
#     # plt.plot(synced_time_list[i], synced_linear_velocity_list[i], linewidth=2, label = legend_list[i])
# plt.xlabel("Time (s)", fontsize=13, fontweight='bold')
# plt.ylabel("ARE (deg)", fontsize=13, fontweight='bold')
# plt.title("Absolute Rotational Error vs Time Chart with " + Dataset, fontsize=15, fontweight='bold')
# plt.legend(loc='upper left')
# pw_results.addPlot("ARE vs Time", fig)



# # Creating the plot
# fig, plt = createFigure()

# for i in range(number_of_parameters):
#     plt.scatter(synced_linear_velocity_list[i], rpy_diff_pitch[:np.size(synced_time_list[i])], s=2, label = legend_list[i])
#     # plt.plot(synced_time_list[i], synced_linear_velocity_list[i], linewidth=2, label = legend_list[i])
# plt.xlabel("Linear Velocity (m/s)", fontsize=13, fontweight='bold')
# plt.ylabel("ARE (deg)", fontsize=13, fontweight='bold')
# plt.title("Absolute Rotational Error vs Linear Velocity Chart with " + Dataset, fontsize=15, fontweight='bold')
# plt.legend(loc='upper left')
# pw_results.addPlot("ARE vs Linear Velocity", fig)



# # Creating the plot
# fig, plt = createFigure()

# for i in range(number_of_parameters):
#     plt.scatter(synced_angular_velocity_list[i], rpy_diff_pitch[:np.size(synced_time_list[i])], s=2, label = legend_list[i])
#     # plt.plot(synced_time_list[i], synced_angular_velocity_list[i], linewidth=2, label = legend_list[i])
# plt.xlabel("Angular Velocity (m/s)", fontsize=13, fontweight='bold')
# plt.ylabel("ARE (deg)", fontsize=13, fontweight='bold')
# plt.title("Absolute Rotational Error vs Angular Velocity Chart with " + Dataset, fontsize=15, fontweight='bold')
# plt.legend(loc='upper left')
# pw_results.addPlot("ARE vs Angular Velocity", fig)



# Creating the plot
fig, plt = createFigure()

for i in range(number_of_parameters):
    plt.scatter(rpy_diff_pitch[:np.size(synced_time_list[i])], synced_linear_velocity_list[i]/np.max(synced_linear_velocity_list[i]), s=2, label = str("Linear Velocity: " + legend_list[i]))
    x = rpy_diff_pitch[:np.size(synced_time_list[i])]
    y = synced_linear_velocity_list[i]/np.max(synced_linear_velocity_list[i])
    plt.plot(np.unique(x), np.poly1d(np.polyfit(x, y, 1))(np.unique(x)), label = str("Linear Velocity: " + legend_list[i]))
    plt.scatter(rpy_diff_pitch[:np.size(synced_time_list[i])], synced_angular_velocity_list[i]/np.max(synced_angular_velocity_list[i]), s=2, label = str("Angular Velocity: " + legend_list[i]))
    x = rpy_diff_pitch[:np.size(synced_time_list[i])]
    y = synced_angular_velocity_list[i]/np.max(synced_angular_velocity_list[i])
    plt.plot(np.unique(x), np.poly1d(np.polyfit(x, y, 1))(np.unique(x)), label = str("Angular Velocity: " + legend_list[i]))
    # plt.plot(synced_time_list[i], synced_linear_velocity_list[i], linewidth=2, label = legend_list[i])
plt.ylabel("Velocity", fontsize=13, fontweight='bold')
plt.xlabel("ARE (deg)", fontsize=13, fontweight='bold')
plt.title("Absolute Rotational Error vs Velocity Chart with " + Dataset, fontsize=15, fontweight='bold')
plt.legend(loc='upper left')
pw_results.addPlot("ARE vs Velocity", fig)








pw_results.show()


#############################################################################################
#############################################################################################
#############################################################################################







    # print("loading plot modules")

    # #TODO:plot three pic like subplots(3)
    # print("plotting")
    # plot_collection = plot.PlotCollection("Example")
    # # metric values
    # fig_1 = plt.figure(figsize=(8, 8))
    # plot.error_array(fig_1, xyz_diff_x, color='grey',
    #                  name="msckf", title=str(ape_metric), linestyle="-")
    # #plot.error_array(fig_1, diff_z, color='blue',
    # #                 name="APE", title=str(ape_metric3), linestyle="-")

    # plot_collection.add_figure("raw_x", fig_1)
    # #
    # fig_2 = plt.figure(figsize=(8, 8))
    # plot.error_array(fig_2, xyz_diff_y, color='grey',
    #                  name="msckf", title=str(ape_metric), linestyle="-")
    # #plot.error_array(fig_2, diff2_y, color='blue',
    #  #                name="vins", title=str(ape_metric2), linestyle="dashed")
    # #plot.error_array(fig_2, diff3_y, color='blue',
    # #                 name="APE", title=str(ape_metric), linestyle="-")

    # plot_collection.add_figure("raw_y", fig_2)
    # #
    # fig_3 = plt.figure(figsize=(8, 8))
    # plot.error_array(fig_3, xyz_diff_z, color='grey',
    #                  name="msckf", title=str(ape_metric), linestyle="-")
    # #plot.error_array(fig_3, diff2_z, color='blue',
    # #                 name="vins", title=str(ape_metric2), linestyle="dashed")
    # #plot.error_array(fig_3, diff3_z, color='blue',
    # #                 name="APE", title=str(ape_metric), linestyle="-")

    # plot_collection.add_figure("raw_z", fig_3)

    # plot_collection.show()

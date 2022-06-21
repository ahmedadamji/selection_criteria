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


for parameter_idx in range(number_of_parameters):
    est_file_name = sys.argv[parameter_idx+1]
    # print(est_file_name)


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
    }

    # print(thisdict)
    keys_list = ['title', 'std', 'rmse', 'max', 'min', 'median', 'sse', 'mean']
    points_keys_list = ['total_input_points', 'total_output_points', 'total_filtered_points', 'average_input_points', 'average_output_points', 'average_filtered_points']



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

    # print(translation)
    # print(rotation)
    # print(points)


    legend_list[parameter_idx] = est_file_name

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


# print(legend_list)

# Creating a plot window to compare data for translation
pw_trans = plotWindow()

pw_trans.MainWindow.setWindowTitle("Translation Statistics")

fig = plt.figure()
plt.plot(legend_index, std_trans_list, '--')
pw_trans.addPlot("std", fig)

fig = plt.figure()
plt.plot(legend_index, rmse_trans_list, '--')
pw_trans.addPlot("rmse", fig)

fig = plt.figure()
plt.plot(legend_index, max_trans_list, '--')
pw_trans.addPlot("max", fig)

fig = plt.figure()
plt.plot(legend_index, min_trans_list, '--')
pw_trans.addPlot("min", fig)

fig = plt.figure()
plt.plot(legend_index, median_trans_list, '--')
pw_trans.addPlot("median", fig)

fig = plt.figure()
plt.plot(legend_index, sse_trans_list, '--')
pw_trans.addPlot("sse", fig)

fig = plt.figure()
plt.plot(legend_index, mean_trans_list, '--')
pw_trans.addPlot("mean", fig)

fig = plt.figure()
plt.plot(legend_index, total_input_points_list, '--')
pw_trans.addPlot("total_input_points", fig)

fig = plt.figure()
plt.plot(legend_index, total_filtered_points_list, '--')
pw_trans.addPlot("total_filtered_points", fig)

fig = plt.figure()
plt.plot(legend_index, average_input_points_list, '--')
pw_trans.addPlot("average_input_points", fig)

fig = plt.figure()
plt.plot(legend_index, average_filtered_points_list, '--')
pw_trans.addPlot("average_filtered_points", fig)

pw_trans.show()


# Creating a plot window to compare data for rotation
pw_rot = plotWindow()

pw_rot.MainWindow.setWindowTitle("Rotation Statistics")

fig = plt.figure()
plt.plot(legend_index, std_rot_list, '--')
pw_rot.addPlot("std", fig)

fig = plt.figure()
plt.plot(legend_index, rmse_rot_list, '--')
pw_rot.addPlot("rmse", fig)

fig = plt.figure()
plt.plot(legend_index, max_rot_list, '--')
pw_rot.addPlot("max", fig)

fig = plt.figure()
plt.plot(legend_index, min_rot_list, '--')
pw_rot.addPlot("min", fig)

fig = plt.figure()
plt.plot(legend_index, median_rot_list, '--')
pw_rot.addPlot("median", fig)

fig = plt.figure()
plt.plot(legend_index, sse_rot_list, '--')
pw_rot.addPlot("sse", fig)

fig = plt.figure()
plt.plot(legend_index, mean_rot_list, '--')
pw_rot.addPlot("mean", fig)

fig = plt.figure()
plt.plot(legend_index, total_input_points_list, '--')
pw_rot.addPlot("total_input_points", fig)

fig = plt.figure()
plt.plot(legend_index, total_filtered_points_list, '--')
pw_rot.addPlot("total_filtered_points", fig)

fig = plt.figure()
plt.plot(legend_index, average_input_points_list, '--')
pw_rot.addPlot("average_input_points", fig)

fig = plt.figure()
plt.plot(legend_index, average_filtered_points_list, '--')
pw_rot.addPlot("average_filtered_points", fig)

pw_rot.show()

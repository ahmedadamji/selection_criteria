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



number_of_parameters = len(sys.argv)-1
# This loop itterates through the statistics of each of the files provided in the arguments list
# This can be used to save the metrics for each in an array that can be used to plot
# Name of the plot can be the metric and legend of the plot can be the parameter used
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

    # print(thisdict)
    keys_list = ['title', 'std', 'rmse', 'max', 'min', 'median', 'sse', 'mean']


    with open(str(est_file_name + ".txt")) as f:
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

    print(translation)
    print(rotation)



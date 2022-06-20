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

# configure matplotlib and seaborn according to package settings
# TODO: 'color_codes=False' to work around this bug:
# https://github.com/mwaskom/seaborn/issues/1546
sns.set(style=SETTINGS.plot_seaborn_style, font=SETTINGS.plot_fontfamily,
        font_scale=SETTINGS.plot_fontscale, color_codes=False,
        palette=SETTINGS.plot_seaborn_palette)
rc = {
    "lines.linewidth": SETTINGS.plot_linewidth,
    "text.usetex": SETTINGS.plot_usetex,
    "font.family": SETTINGS.plot_fontfamily,
    "pgf.texsystem": SETTINGS.plot_texsystem
}
mpl.rcParams.update(rc)

logger = logging.getLogger(__name__)

ListOrArray = typing.Union[typing.Sequence[float], np.ndarray]





# Can use this code to find the path of the current working directory.
# print('getcwd:      ', os.getcwd())
# print('__file__:    ', __file__)

est_file_name = sys.argv[1]

print("loading trajectories")
traj_ref = file_interface.read_tum_trajectory_file("/root/catkin_ws/src/project_ws/catkin_ws/src/data/KITTI/06/results/trajectories/tum/06_gt_tum")
traj_est = file_interface.read_tum_trajectory_file("/root/catkin_ws/src/project_ws/catkin_ws/src/data/KITTI/06/results/trajectories/tum/" + est_file_name + ".tum")

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

# APE: ROTATION PART:

# data = (traj_ref, traj_est)
# ape_metric = metrics.APE(metrics.PoseRelation.rotation_part)
# ape_metric.process_data(data)

# ape_statistics = ape_metric.get_all_statistics()
# print(ape_statistics)
# print("mean:", ape_statistics["mean"])


# Calculating these metrics myself manually for the pitch as KITTI rotations only relevant for pitch and not yaw:
# 'std', 'rmse', 'max', 'min', 'median', 'sse', 'mean'

std_pitch = rpy_diff_pitch.std()
rmse_pitch = np.sqrt(np.mean((rpy_diff_pitch)**2))
max_pitch = rpy_diff_pitch.max()
min_pitch = rpy_diff_pitch.min()
median_pitch = np.median(rpy_diff_pitch)
sse_pitch = np.sum((rpy_diff_pitch)**2)
mean_pitch = np.mean(rpy_diff_pitch)


# Printing statistics for APE for pitch
print("ape_statistics for Pitch:")
print("std:", std_pitch)
print("rmse:", rmse_pitch)
print("max:", max_pitch)
print("min:", min_pitch)
print("median:", median_pitch)
print("sse:", sse_pitch)
print("mean:", mean_pitch)


# Plotting APE for rotation part
print("loading plot modules")


fig_pitch, ax = plt.subplots()

if isinstance(traj_est, trajectory.PoseTrajectory3D):
    x = traj_est.timestamps
    xlabel = "$t$ (s)"
else:
    x = np.arange(0., len(rpy_diff_pitch))
    xlabel = "index"

ylabel = "$Pitch$ (deg)"
# ax.plot(x, rpy_diff_pitch[1:], label = 'pitch')
c = np.tan(x)
ax.plot(x, rpy_diff_pitch[1:], label = 'APE (deg)', alpha = 1.0, color='grey')
ax.plot(x, [rmse_pitch for i in range(len(rpy_diff_pitch[1:]))], label = 'rmse', color='purple')
# ax.plot(x, [max_pitch for i in range(len(rpy_diff_pitch[1:]))], label = 'max', color='yellow')
# ax.plot(x, [min_pitch for i in range(len(rpy_diff_pitch[1:]))], label = 'min', color='blue')
ax.plot(x, [median_pitch for i in range(len(rpy_diff_pitch[1:]))], label = 'median', color='red')
# ax.plot(x, [sse_pitch for i in range(len(rpy_diff_pitch[1:]))], label = 'sse')
ax.plot(x, [mean_pitch for i in range(len(rpy_diff_pitch[1:]))], label = 'mean', color='green')
ax.add_patch(Rectangle((x.min(), mean_pitch-std_pitch), x.max(), mean_pitch+std_pitch,label='std', color='lightblue'))
ax.set_xlabel(xlabel)
ax.set_ylabel(ylabel)
ax.legend(frameon=True, loc='upper right')
plt.savefig(str(est_file_name + "_" + "aper.png"))
# plt.show()


# #  CODE FOR FINDING INDIVIDUAL TRANSLATION DIFFERENCES:

data = (traj_ref, traj_est)

ape_metric = metrics.APE(metrics.PoseRelation.translation_part)

ape_metric.process_data(data)

ape_statistics = ape_metric.get_all_statistics()


std_trans = ape_statistics["std"]
rmse_trans = ape_statistics["rmse"]
max_trans = ape_statistics["max"]
min_trans = ape_statistics["min"]
median_trans = ape_statistics["median"]
sse_trans = ape_statistics["sse"]
mean_trans = ape_statistics["mean"]


print("ape_statistics for Translation:")
print("std:", std_trans)
print("rmse:", rmse_trans)
print("max:", max_trans)
print("min:", min_trans)
print("median:", median_trans)
print("sse:", sse_trans)
print("mean:", mean_trans)


#  Saving results to statistics text file:
lines = ['APE Statistics for Translation: ',
         'std: '+ str(std_trans),
         'rmse: '+ str(rmse_trans),
         'max: '+ str(max_trans),
         'min: '+ str(min_trans),
         'median: '+ str(median_trans),
         'sse: '+ str(sse_trans),
         'mean: '+ str(mean_trans),
         '------------------------------',
         'APE Statistics for Pitch: ',
         'std: '+ str(std_pitch),
         'rmse: '+ str(rmse_pitch),
         'max: '+ str(max_pitch),
         'min: '+ str(min_pitch),
         'median: '+ str(median_pitch),
         'sse: '+ str(sse_pitch),
         'mean: '+ str(mean_pitch)]

with open(str(est_file_name + "_" + "statistics.txt"), 'w') as f:
    for line in lines:
        f.write(line)
        f.write('\n')






# #TODO:plot three pic like subplots(3)
# print("plotting")
# plot_collection = plot.PlotCollection("APE Rotation Part")


# # metric values
# fig_roll = plt.figure(figsize=(8, 8))
# plot.error_array(fig_roll, rpy_diff_roll, color='grey',
#                  name="msckf", title="Errors In Roll", linestyle="-")
# #plot.error_array(fig_roll, diff_z, color='blue',
# #                 name="APE", title=str(ape_metric3), linestyle="-")

# plot_collection.add_figure("raw_roll", fig_roll)



# #
# fig_pitch = plt.figure(figsize=(8, 8))
# plot.error_array(fig_pitch, rpy_diff_pitch, color='grey',
#                  name="APE rotation part", title="Errors In Pitch", linestyle="-")
# #plot.error_array(fig_pitch, diff2_y, color='blue',
#  #                name="vins", title=str(ape_metric2), linestyle="dashed")
# #plot.error_array(fig_pitch, diff3_y, color='blue',
# #                 name="APE", title=str(ape_metric), linestyle="-")

# plot_collection.add_figure("raw_pitch", fig_pitch)


# #
# fig_yaw = plt.figure(figsize=(8, 8))
# plot.error_array(fig_yaw, rpy_diff_yaw, color='grey',
#                  name="msckf", title="Errors In Yaw", linestyle="-")
# #plot.error_array(fig_yaw, diff2_z, color='blue',
# #                 name="vins", title=str(ape_metric2), linestyle="dashed")
# #plot.error_array(fig_yaw, diff3_z, color='blue',
# #                 name="APE", title=str(ape_metric), linestyle="-")

# plot_collection.add_figure("raw_yaw", fig_yaw)




# plot_collection.show()



# ADD COLOUR CODING TO THESE GRAPHS




# #  CODE FOR FINDING INDIVIDUAL TRANSLATION DIFFERENCES:

# data = (traj_ref, traj_est)

# ape_metric = metrics.APE(metrics.PoseRelation.translation_part)

# ape_metric.process_data(data)

# ape_statistics = ape_metric.get_all_statistics()

# print("mean:", ape_statistics["mean"])

# print("calculating xyz differences")
# import numpy as np
# xyz_diff = np.abs(traj_ref.positions_xyz - traj_est.positions_xyz)
# print(xyz_diff)

# xyz_diff_x = xyz_diff[:,0]
# xyz_diff_y = xyz_diff[:,1]
# xyz_diff_z = xyz_diff[:,2]


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

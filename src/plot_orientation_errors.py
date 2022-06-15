#!/usr/bin/env python

#  REFERENCE: https://github.com/MichaelGrupp/evo/issues/20#issuecomment-368425480

from __future__ import print_function
import os
import numpy as np
import matplotlib.pyplot as plt
import math

print("loading required evo modules")
from evo.core import trajectory, sync, metrics
from evo.tools import file_interface
from evo.tools import plot



# Can use this code to find the path of the current working directory.
# print('getcwd:      ', os.getcwd())
# print('__file__:    ', __file__)

print("loading trajectories")
traj_ref = file_interface.read_tum_trajectory_file("/root/catkin_ws/src/project_ws/catkin_ws/src/data/KITTI/06/results/trajectories/06_gt_tum")
traj_est = file_interface.read_tum_trajectory_file("/root/catkin_ws/src/project_ws/catkin_ws/src/data/KITTI/06/results/trajectories/06_odom.tum")

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

print("ape_statistics for Pitch:")
print("std:", rpy_diff_pitch.std())
print("rmse:", np.sqrt(np.mean((rpy_diff_pitch)**2)))
print("max:", rpy_diff_pitch.max())
print("min:", rpy_diff_pitch.min())
print("median:", np.median(rpy_diff_pitch))
print("sse:", np.sum((rpy_diff_pitch)**2))
print("mean:", np.mean(rpy_diff_pitch))




# print("ape_statistics rotation part:", APE_rot)


print("loading plot modules")

#TODO:plot three pic like subplots(3)
print("plotting")
plot_collection = plot.PlotCollection("Example")
# metric values
fig_1 = plt.figure(figsize=(8, 8))
plot.error_array(fig_1, rpy_diff_roll, color='grey',
                 name="msckf", title="RAW ROLL ERRORS", linestyle="-")
#plot.error_array(fig_1, diff_z, color='blue',
#                 name="APE", title=str(ape_metric3), linestyle="-")

plot_collection.add_figure("raw_roll", fig_1)
#
fig_2 = plt.figure(figsize=(8, 8))
plot.error_array(fig_2, rpy_diff_pitch, color='grey',
                 name="msckf", title="RAW PITCH ERRORS", linestyle="-")
#plot.error_array(fig_2, diff2_y, color='blue',
 #                name="vins", title=str(ape_metric2), linestyle="dashed")
#plot.error_array(fig_2, diff3_y, color='blue',
#                 name="APE", title=str(ape_metric), linestyle="-")

plot_collection.add_figure("raw_pitch", fig_2)
#
fig_3 = plt.figure(figsize=(8, 8))
plot.error_array(fig_3, rpy_diff_yaw, color='grey',
                 name="msckf", title="RAW YAW ERRORS", linestyle="-")
#plot.error_array(fig_3, diff2_z, color='blue',
#                 name="vins", title=str(ape_metric2), linestyle="dashed")
#plot.error_array(fig_3, diff3_z, color='blue',
#                 name="APE", title=str(ape_metric), linestyle="-")

plot_collection.add_figure("raw_yaw", fig_3)

plot_collection.show()



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

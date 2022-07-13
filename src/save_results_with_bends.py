#!/usr/bin/env python

import sys
import os
import numpy as np
import seaborn as sns

import math

# This script was used to check the timestamp from the tum files and only save the results to a new file within a specific time range.
# This was used firstly to extract the straight portion of the trajectory, and can be used to even extract specifically the portion with turns.

upper_time_bound = float(1317384541)
lower_time_bound = float(1317384535)
directory_to_save = 'bends/'


# Saving the ground truth to the requested directory
with open("/root/catkin_ws/src/project_ws/catkin_ws/src/data/KITTI/06/results/localization/06_gt_tum") as odometries:
    for odometry in odometries:
        # Open a file with access mode 'a'
        file_object = open(directory_to_save + "06_gt_tum", 'a')

        timestamp = float(odometry.split(" ")[0])
        
        if ( (timestamp < upper_time_bound) and (timestamp > lower_time_bound) ) :
            # print('Condition matched')
            # Append odometry at the end of file
            file_object.write(odometry)
            # Close the file
            file_object.close()



# Saving the requested trajectories to the requested directory
for file_num in range(len(sys.argv)-1):

    est_file_name = sys.argv[file_num+1]

    # path: "/root/catkin_ws/src/project_ws/catkin_ws/src/data/KITTI/06/results/localization/" + est_file_name + ".tum"

    with open("/root/catkin_ws/src/project_ws/catkin_ws/src/data/KITTI/06/results/localization/" + est_file_name + ".tum") as odometries:
        for odometry in odometries:
            # Open a file with access mode 'a'
            file_object = open(directory_to_save + est_file_name + ".tum", 'a')

            timestamp = float(odometry.split(" ")[0])
            
            if ( (timestamp < upper_time_bound) and (timestamp > lower_time_bound) ) :
                # print('Condition matched')
                # Append odometry at the end of file
                file_object.write(odometry)
                # Close the file
                file_object.close()
                
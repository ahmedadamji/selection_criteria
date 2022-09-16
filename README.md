# Selection Criteria
----------------- 

[**Selection Criteria**](https://github.com/ahmedadamji/selection_criteria) is a project focused on **Study of the effect of selection criteria of point features on SLAM performance.**  

It is authored by [**Ahmed Adamjee**](https://www.linkedin.com/in/ahmedadamjee/) and is licensed under the [MIT License](https://github.com/ahmedadamji/selection_criteria/blob/main/LICENSE).  

Please contact the author via the E-Mail IDs provided below regarding any requests relating to the topic:
Email: [adamjiahmed@gmail.com](mailto:adamjiahmed@gmail.com); [Ahmed.Adamjee.21@ucl.ac.uk](mailto:Ahmed.Adamjee.21@ucl.ac.uk)  
<!-- 
## Testing Videos

Pending -->

## Installation

   

* Please install the [**Melodic Workspace**](https://github.com/ahmedadamji/melodic_ws) repository that creates an environment with the correct ros version, packages and messages required for a quick startup.  



* To install The hdl\_graph\_slam package for utilising the supplied launch files for creating a map with integrated nodes for selection criteria, please follow: [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam).  

   

* To install The hdl\_localization package for utilising the supplied launch files for localizing in a built map with integrated nodes for selection criteria, please follow: [hdl_localization](https://github.com/koide3/hdl_localization).  

   

* To install The Evo package for running the evaluation scripts, please follow: [Evo](https://pypi.org/project/evo/).  

   

* To install plotWindow, a package used to create plots for evaluation of recorded trajectories, please follow: [plotWindow](https://github.com/superjax/plotWindow).  

   

* To install the KITTI odometry sequences, to visualise the affect of selection criteria against a benchmark, please follow: [KITTI Visual Odometry / SLAM Evaluation 2012](https://www.cvlibs.net/datasets/kitti/eval_odometry.php).  

   

* The KITTI odometry sequences were converted into rosbag files with the LiDAR data in form of PointCloud2 type messages for this project. This was done using the [kitti2bag](https://github.com/tomas789/kitti2bag) Python package. This is a recommended pre-processing step if working with the KITTI dataset.  

   

* To install the CloudCompare software, to visualise the affect of selection criteria on map building, please follow: [CloudCompare](https://www.danielgm.net/cc/).  

- - - -  


To install this project's package, clone the repository into the src folder of the workspace:  

```
git clone https://github.com/ahmedadamji/selection_criteria.git
```

- - - -  


## Quick Start



Open the installed workspace folder in the terminal  

```
cd melodic_ws/
```

Following that, shell into the corresponding ROS melodic workspace  

```
./run_docker_container.sh 
```

Run this command to build all packages:  

```
catkin build -DCMAKE_BUILD_TYPE=Release
```

- - - -  

The [**Melodic Workspace**](https://github.com/ahmedadamji/melodic_ws) repository can be referred how to shell into multiple instances of this image from different tabs of the terminal.  

- - - -  


**Please note the parameters for the implemented filters should be set by calling the function for the required filter inside the callback function of the corresponding "cpp" file.**  


* For mapping the required changes need to be made to the **selection_criteria_mapping.cpp** script.  

* For localization the required changes need to be made to the **selection_criteria_localization.cpp** script.  

- - - -  


The documentation for each function in these scripts are provided in their corresponding header file.

To fine tune the parameters for the alpha filter, for mapping, this can be done inside the AlphaFilter function, by modifying the min_angle and max_angle variables. It is suggested that these parameters must be a set to a multiple of the statistics of the alpha angle found in the previous frame, which are are makde available from lines 1204 - 1207 of the **selection_criteria_mapping.cpp** script.  

- - - -  


**The results are saved in the data folder, inside the src folder of the workspace.**  

**The results recorded for this project are made available at the following link: [data](https://liveuclac-my.sharepoint.com/:f:/g/personal/ucaban4_ucl_ac_uk/EjILZSOOLhRJsH_uLDgDNyYBBYDeQaTyg6IZOxn7z3xxVw?e=zR40JR)**. This can be downloaded to evaluated the recorded trajectories, else the file structure may be copied if needed to record your own trajectories for evauation.  


The launch files must be modified with the name of the correct dataset and sequence used, as it will save the results to corresponding folders (The folders should however pre-exist with the skeleton similar to the KITTI 00 and 06 results folders).  


For either localization or mapping, ensure that in the corresponding cpp scripts and the launch files, the file paths are modified to save the results of the trajectory as well as corresponding statistics.  

- - - -  



**For the SLAM algorithm to function, the dataset with recorded LiDAR data should be run simultaneously.**  

- - - -  

**As the name of the launch files suggests these algorithms are configured to only process LiDAR points from the "/kitti/velo/pointcloud" points topic, published in the bag files corresponding to each sequence of the KITTI dataset. Users are welcome to make the required changes to configure these to work with their datasets.**  

- - - -  


### Mapping

To launch hdl\_graph\_slam, using the selection criteria scripts, run the following command:  

```
roslaunch selection_criteria KITTI_graph_slam_sc_hdl.launch dataset:=$(rosparam get /dataset) sequence:=$(rosparam get /sequence) filter_name:=$(rosparam get /filter_name)
```

- - - -  

#### Evaluation

To evaluate the map, the map must be saved as a pcd file and can be compared using the [CloudCompare](https://www.danielgm.net/cc/) Software.  

Saving built map to pcd:  

```
rosrun pcl_ros pointcloud_to_pcd input:=/hdl_graph_slam/map_points 
```

- - - -  


### Localization

To launch hdl\_localization, using the selection criteria scripts, run the following command:  

```
roslaunch selection_criteria KITTI_localization_sc_hdl.launch dataset:=$(rosparam get /dataset) sequence:=$(rosparam get /sequence) filter_name:=$(rosparam get /filter_name)
```

- - - -  


#### Evaluation

To evaluate the localization performance, convert the recorded odometry to the tum trajectory format using  
```
evo_traj bag KITTI_06.bag /odom_transformed --save_as_tum
```

- - - -  

The trajectory statistics, such as the number of points filtered and the time and speed at each recorded frame of the trajectory, are automatically named with the name corresponding to the filter configuration. The recorded odometry must be saved with the same name.  

- - - -  

Using the provided evaluation script for "plot_errors_and_save_statistics.py", the statistics regarding the error metrics can be computed and stored in the corresponding statistics folder.  

- - - -  


i.e. if the trajectory was named KITTI_06_vanilla, the command to save the statistics relating to this trajectory is as follows:  

**Please note you need to be in the corresponding localization folder of the relevant results folder to run this script**  

```
./plot_errors_and_save_statistics.py KITTI_06_vanilla
```

- - - -  


Using the provided evaluation script for "plot_statistics.py", the statistics saved for all trajectories can be computed using multiple windows with relevant plots in sevaral tabs, consisting of all metrics discussed in the project report. **Please make sure the correct folder configuration is being followed**.  

i.e. if the trajectories were named KITTI_06_ff_10_20 KITTI_06_ff_20_30 KITTI_06_ff_30_40, the code to plot the statistics comparing these trajectories is as follows:  

**Please note you need to be in the corresponding statistics folder of the localization folder of the relevant results folder to run this script**  

```
./plot_statistics.py KITTI_06_ff_10_20 KITTI_06_ff_20_30 KITTI_06_ff_30_40
```

- - - -  

Similarly to plot the speed vs error and speed vs time relationship for each of these trajectories, this can be done by following:  

**Please note you need to be in the corresponding localization folder of the relevant results folder to run this script**  

```
./plot_speed_statistics.py KITTI_06_ff_10_20
./plot_speed_statistics.py KITTI_06_ff_20_30
./plot_speed_statistics.py KITTI_06_ff_30_40
```
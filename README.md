### Semnatic SLAM
This project provides functionality to build a semantic voxel grid of the environment using a lidar, camera, SLAM system, and semantic segmentation network.

It is forked from a project which provided this functionality for an RGB camera. The SLAM portion was removed from that project and is now assumed to be externally provided.

# Install
Create a `catkin_ws`. Into the `src` folder clone this project and [ros_numpy](https://github.com/eric-wieser/ros_numpy). Install octomap using `sudo apt install ros-<ros-version>-octomap` and `sudo apt install ros-<ros-version>-octomap-msgs`. Alternatively, you can build both from source by cloning them from the following two links: 
 [octomap](https://github.com/OctoMap/octomap), [octomap_msgs](https://github.com/OctoMap/octomap_msgs), 

Run `catkin_make` from the `catkin_ws` folder to build the package and and then source `devel/setup.sh` to make the packages available.

Now we need to set up a conda environment containing the dependencies. The suggested name is `semantic_slam`, but you can change it to whatever.
```
conda create -f mmseg-ros-environment.yml -n semantic_slam 
conda activate semantic_slam 
```
Now we need to install `mmsegmentation` from my fork. Clone the repo from [my fork](git@github.com:russelldj/mmsegmentation.git), which contains a modification to return the confidence values. Now `cd` to that directory and with the conda environment activated, `pip install -e .` to install the project into the conda environment.

Update model paths in `semantic_slam/params/semantic_cloud.yaml` to point to the trained model from [here](https://github.com/fyandun/SafeForest/tree/main/data/models).
Download the [ros bag](https://drive.google.com/file/d/1fvlerB5mmBoTpVeji7noYaOWlfmmDmXf/view?usp=sharing) which contains containing lidar, images, and precomputed pose. Finally, run `roslaunch semantic_slam semantic_mapping.launch`.

# Modifications (David Russell)
`semantic_cloud/include/color_pcl_generator/color_pcl_generator.py` now optionally takes in a point cloud and textures it.
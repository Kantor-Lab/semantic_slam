### Semnatic SLAM
This project provides functionality to build a semantic voxel grid of the environment using a lidar, camera, SLAM system, and semantic segmentation network.

It is forked from a project which provided this functionality for an RGB camera. The SLAM portion was removed from that project and is now assumed to be externally provided.

# Install
Create a `catkin_ws`. Into the `src` folder clone this project and [ros_numpy](https://github.com/eric-wieser/ros_numpy). Install octomap using `sudo apt install ros-<ros-version>-octomap` and `sudo apt install ros-<ros-version>-octomap-msgs`. Alternatively, you can build both from source by cloning them from the following two links: 
 [octomap](https://github.com/OctoMap/octomap), [octomap_msgs](https://github.com/OctoMap/octomap_msgs), 

Run `catkin_make` from the `catkin_ws` folder to build the package and and then source `devel/setup.sh` to make the packages available.

Now we need to set up a conda environment containing the dependencies. The suggested name is `semantic_slam`, but you can change it to whatever.
```
conda env create -f mmseg-ros-environment.yml -n semantic_slam 
conda activate semantic_slam 
```
Now we need to install `mmcv` with pip. I have not been able to successfully install the correct versionusing the conda env file so it must be done manually with the following command.
```
pip install mmcv-full==1.4.0 -f https://download.openmmlab.com/mmcv/dist/cu111/torch1.10.0/index.html
```
This works correctly with the version of CUDA and PyTorch included in the conda env. If you are using different versions, refer to the general `mmcv` [installation guidelines](https://mmcv.readthedocs.io/en/latest/get_started/installation.html). 

Now we need to install `mmsegmentation` from my fork. Clone the repo from [my fork](https://github.com/russelldj/mmsegmentation), which contains a modification to return the confidence values. Now `cd` to that directory and with the conda environment activated, `pip install -e .` to install the project into the conda environment.

Update model paths in `semantic_slam/params/semantic_cloud.yaml` to point to the trained model from [here](https://github.com/fyandun/SafeForest/tree/main/data/models).

Download the [ros bag](https://drive.google.com/file/d/1fvlerB5mmBoTpVeji7noYaOWlfmmDmXf/view?usp=sharing) which contains containing lidar, images, and precomputed pose.

## Docker install
Alternatively, you can install the docker container. You'll need nvidia docker set up. Then you can run `build_docker.sh` and `run_docker.sh`. Then you can place models and bags in the data folder and they will show up mounted in the `~/data` directory of the container. Inside of the container `conda activate semantic_slam` and `cd ~/catkin_ws` and then you can run things as normal.

Note that you may have to use tmux to run two windows at once. Alternatively, you can attach to a running docker container with `docker exec -it <container ID> bash`. This can be found using `docker container ls`.

# Running
Begin by sourcing the `setup.bash` file. Then play your bag with `rosbag play <bag name>`. Finally, run `roslaunch semantic_slam semantic_mapping.launch`.

# Modifications (David Russell)
`semantic_cloud/include/color_pcl_generator/color_pcl_generator.py` now optionally takes in a point cloud and textures it.

# Params
`semantic_slam/semantic_slam/params/octomap_generator.yaml:octomap:frame_id` should be set to your odometry topic
`semantic_slam/semantic_slam/params/octomap_generator.yaml:octomap:frame_id` should be set to your odometry topic

FROM osrf/ros:melodic-desktop-full

# Suggestion taken from here: https://stackoverflow.com/questions/20635472/using-the-run-instruction-in-a-dockerfile-with-source-does-not-work
# Uses bash instead of sh
SHELL ["/bin/bash", "-c"]

# LABEL about the custom image
LABEL maintainer="davidrus@andrew.cmu.edu"
LABEL version="0.1"
LABEL description="This is custom Docker Image for \
running MSCKF_VIO SLAM system."

# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive

# Update Ubuntu Software repository
RUN apt update

RUN apt install git vim tmux -y
RUN apt install ros-melodic-octomap ros-melodic-octomap-msgs 
##ros-kinetic-octomap-mapping ros-kinetic-octomap-ros -y
#
RUN cd ~ && mkdir catkin_ws && cd catkin_ws && mkdir src && source /opt/ros/melodic/setup.bash && catkin_make
RUN cd ~/catkin_ws/src && git clone https://github.com/eric-wieser/ros_numpy.git && git clone https://github.com/Kantor-Lab/semantic_slam.git
RUN cd ~/catkin_ws && source /opt/ros/melodic/setup.bash  && source devel/setup.sh && catkin_make
RUN cd ~/catkin_ws/src && git clone https://github.com/OctoMap/octomap_rviz_plugins.git

RUN apt install wget 
# Install miniconda
ENV CONDA_DIR /opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
     /bin/bash ~/miniconda.sh -b -p /opt/conda


RUN mkdir ~/dev && cd ~/dev && git clone https://github.com/russelldj/mmsegmentation.git
# Put conda in path so we can use conda activate
ENV PATH=$CONDA_DIR/bin:$PATH
RUN cd ~/catkin_ws/src/semantic_slam && conda env create -f mmseg-ros-environment.yml -n semantic_slam 
#RUN conda create -n semantic_slam python=3.8 pytorch=1.10 torchvision torchaudio cudatoolkit=11.1 -c pytorch -c conda-forge
RUN conda init bash
SHELL ["conda", "run", "-n", "semantic_slam", "/bin/bash", "-c"]
RUN pip install mmcv-full==1.4.0 -f https://download.openmmlab.com/mmcv/dist/cu111/torch1.10.0/index.html
RUN cd ~/dev/mmsegmentation &&  pip install -e .
RUN cd ~/catkin_ws && source /opt/ros/melodic/setup.bash  && source devel/setup.sh && catkin_make
#RUN conda activate semantic_slam

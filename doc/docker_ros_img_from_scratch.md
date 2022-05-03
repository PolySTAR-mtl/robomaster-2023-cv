# Creating a Docker Image from scratch to use for Detection/Tracking...

## Generating Docker ROS image

### Download image and Run image

Download base ROS image: `docker pull ros:ros-<ros-distribution>` (e.g. `ros:ros-melodic`). Then `docker run -it ros:ros-<ros-distribution>`

*Note:* If running container from WSL `docker run -it --net=host --env="DISPLAY" ros:ros-<ros-distribution>` after correctly setting up XMing server (https://sourceforge.net/projects/xming/)

### Setup ssh / git conf

1. Update `apt-get update`
2. Install openssh `apt-get openssh-client`
3. Generate sshkey (`cd ~/.ssh && ssh-keygen`). `mkdir ~/.ssh` if the directory doesn't exist.
4. Put key on github account in the `~/.ssh` directory)
5. Setup git configuration `git config --global user.name "username" && git config --global user.email "email"`

### Setup robomaster repostiory & ROS

First, clone the repostiory `git clone https://github.com/PolySTAR-mtl/robomaster-2022-cv.git`. Then, install [submodule](https://github.com/PolySTAR-mtl/robomaster-2022-cv/tree/main/detection#small-note-on-submodules)
and install [Darknet](https://github.com/PolySTAR-mtl/robomaster-2022-cv/tree/main/detection#setting-up-darknet-environment). You can first verify everything works with detection on it's
own by running the inference presented [here](https://github.com/PolySTAR-mtl/robomaster-2022-cv/tree/main/detection#setting-up-darknet-environment).

Then, you need to install the different packages used by ROS:

1. `apt-get install ros-<ros-distribution>-cv-bridge`
2. `apt-get install ros-<ros-distribution>-image-view`
3. `apt-get install ros-<ros-distribution>-vision-opencv`
4. `apt-get install ros-<ros-distribution>-camera-info-manager`
5. `apt-get install ros-<ros-distribution>-video-stream-opencv`

You can save the state of the container `docker commit <container:id>  <image:name>`.

### Detection: modify and run

1. Modify `video_sender.launch`: `<remap from="/video_file/image_raw" to="/detection/image_in"/>` -> `<remap from="/video_file/image_raw" to="/detector/image_in"/>`.
2. Add `image_view.launch` to visualise:
```
<?xml version="1.0"?>
<launch>
    <node name="image_view" pkg="image_view" type="image_view" respawn="false" output="screen">
    <remap from="image" to="/detector/image_in" />
    </node>
</launch>
```

Don't forget to `catkin_make -DDARKNET_PATH=../detection/darknet` and `source devel/setup.bash`.

To run, launch in different terminal:
1. `roscore`
2. `rosrun detection detection _net/datacfg:=<dji.data path> _net/config_path:=<cfg path> _net/weights:=<weights path> _net/labels:=<dji.names path>`
3. `roslaunch video_sender.launch file:=/home/robomaster-2022-cv/detection/exemple/vid_test.mp4` (for instance). **Warning** Put the absolute path or it won't work
4. `roslaunch image_view.launch`
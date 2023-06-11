# Description & Instructions

This package implements the box pushing scenario for the triton robot. This was written from scratch (no starter code). The demo video can be viewed at: https://youtu.be/qCwxBEC8WS4

Note that this requires the triton's docker image and the hector-mapping package. Make sure to also set the paramter "align_depth" to "true" when launching the realsense package in the docker image. 

Side note: The deep learning object detection experiment code, models, and images are in the "DL_obj_det" folder inside the "src" folder.

## Usage

Extract `P4D3.zip` in your `catkin_ws/src` directory:
```bash
cd ~/catkin_ws/src
unzip P4D3.zip
```

To compile: 
```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

Launch the simulation:
```bash
roslaunch triton_box_push triton_box_push_slam.launch
```


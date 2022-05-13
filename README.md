# Dense-point-cloud-scan-reconstruction

Install

Follow the installation steps of LOAM

after installation

Enter the command in the terminal:

sudo gedit /opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py

modifes this sentence in nodeprocess.py:

_TIMEOUT_SIGINT = 15.0 

where 15.0 is 15 seconds
modify 15.0 to an expected value

This is to allow enough time to store files when the node logs out

RUN

roslaunch frame_reconstruction mapping.launch

note that:

The file path of rosbag is formulated in Dense-point-cloud-scan-reconstruction/frame_reconstruction/launch/mapping.launch
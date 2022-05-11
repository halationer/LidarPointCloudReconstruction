# Dense-point-cloud-scan-reconstruction




sudo gedit /opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py

modifes this sentence:

_TIMEOUT_SIGINT = 15.0 

where 15.0 is 15 seconds
modify 15.0 to an expected value
#install docker and nvidia_docker and CUDA in Ubuntu 16.04

On Mac or Windows install Docker for Mac or Windows
The instructions bellow should work on Mac also (at least the CPU mode).

#then execute these commands
	
	mkdir ~/sharefolder
	cd ~/sharefolder/
	git pull https://github.com/gtarobotics/self-driving-car
	cd self-driving-car

#run SDC Docker image

In GPU mode:

	./run_nvidia_docker-sdc-ros-gpu.sh

or CPU mode:

	./run_nvidia_docker-sdc-ros-cpu.sh

#Once in the container try to test the performance

	./run_quick_benchmark.sh

Please post the results on https://nd013.slack.com team in #environment channel.

#open 2 new terminals in host OS

#check docker container_id in the first new terminal
	
	sudo docker ps | grep "gtarobotics/udacity-sdc"

#go to terminal 2 and attach to the container and start roscore
	
	attach-docker-container.sh container_id
	source /opt/ros/indigo/setup.bash
	roscore 
 
#go to terminal 3 

	attach-docker-container.sh container_id

#change dir to where the Udacity SDC challange rosbag sets are, make sure they are under /sharefolder in the container

	cd /sharefolder/sdc-data/600GB-dataset/2016-10-10

#and play all 3 cameras rosbag starting with second 120 (you can change this starting point and it should load pretty fast)
	
	rosbag play -s 120 udacity-dataset_sensor_camera_left_2016-10-11-13-23-02_0.bag udacity-dataset_sensor_camera_center_2016-10-11-13-23-02_0.bag udacity-dataset_sensor_camera_right_2016-10-11-13-23-02_0.bag

or just:

	rosbag play *.bag


#go back to first terminal and run the viewer

	cd /sharefolder/self-driving-car
	python2 sdc_rosbag_viewer.py

You should see the 3 cameras in the view like in the screenshot:

	sdc_rosbag_viewer-in-action.png

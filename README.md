
#Instructions on how to your environment ready for Udacity Self Driving Car (SDC) Challenges

The challenges are available here:

	https://www.udacity.com/self-driving-car 

#Install docker and nvidia_docker and CUDA

The recommended platform is Ubuntu 16.04 host and Docker guest.

On Mac or Windows install Docker for Mac or Windows

The instructions bellow should work on Mac also (at least the CPU mode).

For Amazon AWS EC2 see [Amazon AWS EC2 AMI with gtarobotics/udacity-sdc image installed](#AWS) 

#Get the SDC environment ready

Execute these commands in the host OS (Ubuntu and OSX):
	
	mkdir ~/sharefolder
	cd ~/sharefolder/
	git pull https://github.com/gtarobotics/self-driving-car
	cd self-driving-car
	chmod 755 *.sh

#Run SDC Docker image

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

## AWS
#Amazon AWS EC2 AMI with gtarobotics/udacity-sdc image installed

The image id is: ami-0267c362

Launch at least a p2-xlarge instance (one K80 GPU)

Once in the EC2 instance shell you can run the quick benchmark like this:

	./run_gtarobotics_udacity_sdc_docker_image.sh ./run_quick_benchmark.sh

Or to get into the container shell use this:

	./run_gtarobotics_udacity_sdc_docker_image.sh

I ran the benchmark on a Spot instance with one Nvidia K80, up to $0.90 per hour and I got this performance:

	Step 1000 (epoch 1.16), 12.3 ms

On my Nvidia 980ti based desktop I get:

	Step 1000 (epoch 1.16), 5.7 ms 


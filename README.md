
# Instructions on how to get your development environment ready for Udacity Self Driving Car (SDC) Challenges

The challenges are available here:
	[An Open Source Self-Driving Car](https://www.udacity.com/self-driving-car)

Follow us on Twitter at [@gtarobotics](https://twitter.com/gtarobotics)

# Install docker and nvidia_docker and CUDA

The recommended platform is Ubuntu 16.04 host and [Docker for Linux](https://docs.docker.com/engine/installation/linux/ubuntulinux/).

If you have an NVidia GPU with CUDA compute level >= 3, make sure you install [nvidia_docker](https://github.com/NVIDIA/nvidia-docker) also.

On Mac install [Docker for Mac](https://docs.docker.com/docker-for-mac/) on Windows [Docker for Windows](https://docs.docker.com/docker-for-windows/)

The instructions bellow should work on Mac also (at least the CPU mode).

For Amazon AWS EC2 see [Amazon AWS EC2 AMI with gtarobotics/udacity-sdc image](#amazon-aws-ec2-ami-with-gtaroboticsudacity-sdc-image-installed) 

# Get the SDC host development environment ready

Execute these commands in the host OS (Ubuntu and OSX):
	
	mkdir ~/sharefolder
	cd ~/sharefolder/
	git clone https://github.com/gtarobotics/self-driving-car
	cd self-driving-car
	chmod 755 *.sh

# Run SDC Docker image

Here you can see more details about this docker image:
	[Docker instance with Tensorflow GPU, Keras, Caffe, Torch, Jupyter Notebook, ROS Indigo and Autoware and more](https://hub.docker.com/r/gtarobotics/udacity-sdc/)

In GPU mode:
	./run_nvidia_docker-sdc-ros-gpu.sh

or CPU mode:
	./run_nvidia_docker-sdc-ros-cpu.sh

# Once in the container
First update the scripts to latest version from GitHub

	./update_scripts.sh
	nvcc -V #to check the CUDA version

CUDA version can be switch with:

	switch_to_CUDA-7.5.sh

	switch_to_CUDA-8.0.sh

Then run a performance test, this will also confirm that env (CPU/GPU) is working

	./run_quick_benchmark.sh

Please post the [results like this](#quick-benchmark-results) on [ND013 Slack Team](https://nd013.udacity.com/) in [#benchmarks](https://nd013.slack.com/messages/benchmarks) channel.

See some results from contributors local machines and AWS here [benchmarks_results](./benchmarks_results)

# View datasets, works for now only on local computer, I'll add instructions on how to do it on AWS later

### open 2 new terminals in host OS

### check docker container_id in the first new terminal	
	sudo docker ps | grep "gtarobotics/udacity-sdc"

### go to terminal 2 and attach to the container and start roscore
	attach-docker-container.sh container_id
	source /opt/ros/indigo/setup.bash
	roscore 
 
### go to terminal 3
	attach-docker-container.sh container_id

### change dir to where the Udacity SDC challenges rosbag sets are, make sure they are under /sharefolder/sdc-data in the container
The current datasets can be downloaded from here [Udacity SDC GitHub project udacity/self-driving-car](https://github.com/udacity/self-driving-car)
	cd /sharefolder/sdc-data/600GB-dataset/2016-10-10

### and play all 3 cameras rosbag starting with second 120 (you can change this starting point and it should load pretty fast)	
	rosbag play -s 120 udacity-dataset_sensor_camera_left_2016-10-11-13-23-02_0.bag udacity-dataset_sensor_camera_center_2016-10-11-13-23-02_0.bag udacity-dataset_sensor_camera_right_2016-10-11-13-23-02_0.bag

or just:
	rosbag play *.bag

### go back to first terminal and run the viewer
	cd /sharefolder/self-driving-car
	python2 sdc_rosbag_viewer.py

You should see the 3 cameras in the view like in the screenshot:
	sdc_rosbag_viewer-in-action.png

# Amazon AWS EC2 AMI with gtarobotics/udacity-sdc image installed
The image id is: ami-0267c362 and it is available only in US West (Oregon) region

Launch at least a [p2.xlarge CUDA compute](https://aws.amazon.com/ec2/instance-types/p2/) instance (one K80 GPU)

Once in the EC2 instance shell you can run the quick benchmark like this:

	./run_gtarobotics_udacity_sdc_docker_image.sh ./run_quick_benchmark.sh

Or to start a Docker container shell use this just this:

	./run_gtarobotics_udacity_sdc_docker_image.sh

# Quick benchmark results
I ran the benchmark on a Spot instance with one Nvidia K80, up to $0.90 per hour and I got this performance:

	Step 1000 (epoch 1.16), 12.3 ms

On my Nvidia 980ti based desktop I get:

	Step 1000 (epoch 1.16), 5.7 ms 

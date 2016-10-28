

# Instructions on how to get your development environment ready for Udacity Self Driving Car (SDC) Challenges

The challenges are available here:
	[An Open Source Self-Driving Car](https://www.udacity.com/self-driving-car)

Follow us on Twitter at [@gtarobotics](https://twitter.com/gtarobotics)

# Install docker and nvidia_docker and CUDA

The recommended platform is Ubuntu 16.04 host and [Docker for Linux](https://docs.docker.com/engine/installation/linux/ubuntulinux/).

If you have an NVidia GPU with CUDA compute level >= 3, make sure you install [nvidia_docker](https://github.com/NVIDIA/nvidia-docker) also.

On Mac install [Docker for Mac](https://docs.docker.com/docker-for-mac/) on Windows [Docker for Windows](https://docs.docker.com/docker-for-windows/)

The instructions bellow should work on Mac also (at least the CPU mode, see specific script to start the docker image bellow).
For Mac see also [Docker for Mac and GUI applications](https://fredrikaverpil.github.io/2016/07/31/docker-for-mac-and-gui-applications/).

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

On Mac OSX run this script instead (provided by ND013@tantony):

	./run_docker_sdc_ros_cpu-on-OSX.sh (change en1 with en0 if it doesn't work, also check the ND013 #environment channel for other solutions for multimonitor setups)

# Once in the container
First update the scripts to latest version from GitHub

	./update_scripts.sh
	nvcc -V #to check the CUDA version

CUDA version can be switched with:

	switch_to_CUDA-7.5.sh
	switch_to_CUDA-8.0.sh

Then run a performance test, this will also confirm that env (CPU/GPU) is working

	./run_quick_benchmark.sh

Please post the [results like this](#quick-benchmark-results) on [ND013 Slack Team](https://nd013.udacity.com/) in [#benchmarks](https://nd013.slack.com/messages/benchmarks) channel.

See some results from contributors local machines and AWS here [benchmarks_results](./benchmarks_results)

To test OpenCV do this:

	cd /sharefolder/
	curl https://archive.org/download/NASA_Launchpad_MSL_HD/NASA_Launchpad_MSL_HD.mp4 -o NASA_Launchpad_MSL_HD.mp4 -L
	python3 ~/test_opencv.py -v NASA_Launchpad_MSL_HD.mp4

It should play (fast) the video in grayscale, resized to width 640.

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
The images are available in  available only in US West (Oregon) region, their ids are:
 
 GTA Robotics - Udacity Open Source Self Driving Car Challanges - Docker GPU/CPU image - ami-0267c362 (older docker imgage version, I'll remove this in the future)
 GTA Robotics - Udacity Open Source Self Driving Car Challenges - Docker GPU/CPU Cuda 7.5/8.0 image - ami-7d28e8f1d (latest docker image, also Tensorflow can be uesd in VM directoy)

Aalways check the list above for updated AMIs, when new AMIs are added the old ones will be removed!
The best way to find them on AWS is to search for "gta robotics" in the community AMIs in us-west-2 zone.

Launch at least a [p2.xlarge CUDA compute](https://aws.amazon.com/ec2/instance-types/p2/) instance (one K80 GPU)

Once in the EC2 instance shell you can run the quick benchmark like this:

	./run_gtarobotics_udacity_sdc_docker_image.sh ./run_quick_benchmark.sh

Or to start a Docker container shell use this just this:

	./run_gtarobotics_udacity_sdc_docker_image.sh

# Quick benchmark results
I ran the benchmark on a Spot instance (p2.xlarge) with one Nvidia K80, up to $0.90 per hour and I got this performance:

	Step 1000 (epoch 1.16), 12.3 ms

On my Nvidia 980TI based desktop I get:

	Step 1000 (epoch 1.16), 5.2 ms 

The Nvidia Autopilot test was a bit faster on p2.xlarge vs a 980TI based desktop, both running the same docker (gtarobotics/udacity-sdc) instance:

	gtarobotics/udacity-sdc docker instance on AWS EC2 p2.xlarge VM (Ubuntu 14.04 as host):

		root@f62afc086a85:~/sharefolder/Nvidia-Autopilot-TensorFlow# time python3 train.py

		Model saved in file: ./save/model.ckpt
		step 13610, val loss 0.0172059
		step 13620, val loss 0.00907515

		real	87m15.149s
		user	80m31.441s
		sys		4m4.473s

	gtarobotics/udacity-sdc docker instance on Nvidia 980TI (Ubuntu 16.04 as host):

		root@48ae719d1e3b:~/sharefolder/Nvidia-Autopilot-TensorFlow# time python3 train.py
		
		Model saved in file: ./save/model.ckpt
		step 13610, val loss 0.0117589
		step 13620, val loss 0.0108083
		real    95m36.718s
		user    53m38.324s
		sys     3m10.360s
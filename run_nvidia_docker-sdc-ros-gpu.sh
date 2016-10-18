sudo xhost +
sudo nvidia-docker run --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -env="QT_X11_NO_MITSHM=1" -v /dev/video0:/dev/video0 -v /tmp/.X11-unix:/tmp/.X11-unix:ro -it -p 8888:8888 -p 6007:6006 -v ~/sharefolder:/sharefolder:shared gtarobotics/udacity-sdc bash


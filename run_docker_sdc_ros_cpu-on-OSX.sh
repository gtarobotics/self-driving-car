#!/bin/sh

ip=$(ifconfig en1 | grep inet | awk '$1=="inet" {print $2}')

xhost + $ip

docker run -it --rm\
   -p 2022:22 \
   --name sdcnd_ros \
   --env="DISPLAY=$ip:0" \
   -v /tmp/.X11-unix:/tmp/.X11-unix\
   --volume "$HOME/.Xauthority:/root/.Xauthority:rw" \
   --volume "$HOME/sharefolder:/sharefolder" \
   gtarobotics/udacity-sdc /bin/bash
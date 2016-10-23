#!/usr/bin/env bash

echo "This command will run a quick benchmark. On a system with Nvidia Geforce 980ti - 2816 CUDA Cores you should see these results:"
echo " - aproxmative 5.2 ms per step run  - if GPU is available (when the docker image is started with nvidia-docker command)"
echo " - aproximative 102 ms per step run - if it runs in CPU mode (when the docker image is started with docker command)"
sleep 5

time python3 -m tensorflow.models.image.mnist.convolutional


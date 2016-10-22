#!/usr/bin/env bash

echo "This command will run a quick benchmark. On a system with Nvidia Geforce 980ti - 2816 CUDA Cores you should see these results:"
echo " - 6 ms   - if GPU is available (when the docker image is started with nvidia-docker command)"
echo " - 102 ms - if it runs in CPU mode (when the docker image is started with docker command)"


time python3 -m tensorflow.models.image.mnist.convolutional


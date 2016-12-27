export CUDA_HOME=/usr/local/cuda-7.5
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/cuda-7.5/lib64"
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/mesa/libGL.so.1 ./run_autoware.sh

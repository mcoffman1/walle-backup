#!/usr/bin/env bash
env > /tmp/script_environment.txt

arg1=${1:-"display://0"}  #rtp://10.0.0.50:1234 or display://0 or rtp://10.0.0.177:5000
cd /home/walle/jetson-inference
sudo docker/run.sh --volume /home/walle/ml_python:/ml_python --ros=noetic -r /ml_python/detectnet.sh "$arg1"

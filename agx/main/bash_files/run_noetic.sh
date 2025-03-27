#!/bin/bash

cd /home/walle/jetson-inference/
docker/run.sh --volume /home/walle/ml_python:/ml_python --ros=noetic #-r /ml_python/detectnet.sh

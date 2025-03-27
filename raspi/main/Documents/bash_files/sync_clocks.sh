#!/bin/bash

# Get the current date/time in UTC from the Raspberry Pi
wallemain_time=$(ssh walle@10.0.0.63 'date -u +"%Y-%m-%d %H:%M:%S"')

# Set the system date/time to the retrieved value in UTC
sudo date -u -s "$wallemain_time"

echo "Time synchronized to: $wallemain_time"

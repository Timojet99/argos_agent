#!/bin/bash
OUTFILE="./data/fence.bag"
if [ -f "$OUTFILE" ]; then
	echo "file $OUTFILE exists, delete it first...";
	exit 1;
fi
echo "Will start recording in 10 seconds..." &&
sleep 10 &&
echo "Starting recording..." &&
rosbag record -O "$OUTFILE" /tf /tf_static

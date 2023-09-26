#!/usr/bin/env bash
(trap 'kill 0' SIGINT;\
	ros2 run tello tello &\
	ros2 run tello_control tello_control &\
	ros2 run zbar_ros barcode_reader -r /image:/image_raw &\
	wait)

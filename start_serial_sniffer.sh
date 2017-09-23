#! /bin/sh 
interceptty -s "ospeed 38400 ispeed 38400" /dev/ttyUSB0 | interceptty-nicedump 

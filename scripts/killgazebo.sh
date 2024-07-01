#!/bin/bash
ps -ef | grep 'ign gazebo server' | cut -c 9-15 | xargs kill -s 15
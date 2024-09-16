#!/bin/zsh

# Set user camera follow the robot in ignition-gazebo

ign service -s /gui/follow \
  -r "data: 'red_standard_robot1'" \
  --reqtype ignition.msgs.StringMsg \
  --reptype ignition.msgs.Boolean \
  --timeout 1000

ign service -s /gui/follow/offset \
  -r "x: -0.8, y: 0.0, z: 1.2" \
  --reqtype ignition.msgs.Vector3d \
  --reptype ignition.msgs.Boolean \
  --timeout 1000
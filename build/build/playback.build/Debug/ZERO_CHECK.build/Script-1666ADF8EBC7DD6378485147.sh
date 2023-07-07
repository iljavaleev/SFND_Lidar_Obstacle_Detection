#!/bin/sh
set -e
if test "$CONFIGURATION" = "Debug"; then :
  cd /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/build
  make -f /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "Release"; then :
  cd /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/build
  make -f /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "MinSizeRel"; then :
  cd /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/build
  make -f /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "RelWithDebInfo"; then :
  cd /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/build
  make -f /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/build/CMakeScripts/ReRunCMake.make
fi


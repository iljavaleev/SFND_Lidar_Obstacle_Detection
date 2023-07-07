#!/bin/sh
set -e
if test "$CONFIGURATION" = "Debug"; then :
  cd /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build
  make -f /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "Release"; then :
  cd /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build
  make -f /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "MinSizeRel"; then :
  cd /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build
  make -f /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "RelWithDebInfo"; then :
  cd /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build
  make -f /Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build/CMakeScripts/ReRunCMake.make
fi

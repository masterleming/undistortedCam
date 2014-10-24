undistortedCam
==============

Program utilizing all OpenCV stereo algorithms.

It was developed for my master thesis to test and compare all stereo algorithms available in OpenCV.

Dependancies:
 - OpenCV 2.4.9 -- developed and tested with this version; later versions (3.0 and above) have been redesigned and are known to not work with my program;
 - Program Options -- from Boost, tested with version 1.55 and 1.56;
 - Date Time -- from Boost, tested with version 1.55;
 - Nvidia CUDA -- tested with version 5.5 (onlu because my video card is not supported with newer toolkits); NOTE: current state of the CMake project does not allow to build without CUDA;

Features:
 - deep parametrization of the process;
 - calibrating stereo camera (although it is not working correctly at the time; also I am currently considering to drop this feature in favor of bundeled calibration program);
 - calculating disparity map using any of the following stereo algorithms (names from OpenCV):
   - SBlock Matching
   - Semiglobal Block Matching
   - Var
   - GPU Block Matching
   - GPU Belief Propagation
   - GPU Constant Space Belief Propagation
 - 2 modes of operation:
   - live from 2 cameras connected to the PC
   - from 2 static images
 - "shifted" mode where one image is a copy of the other with small region sligthly displaced (good for testing without cameras or static stereo pair pictures)

This program is a part of a tools bundle (and a master CMake project);
Other tools being:
 - sample program from OpenCV for stereo calibration (not included);
 - program for acquisiting images for stereo calibration (in separate project);

 

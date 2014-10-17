/*
 * main.cpp
 *
 *  Created on: 08-06-2013
 *      Author: Krzysztof Pilch
 */

#include <iostream>
#include "ConfigParser.h"

int main(int argc, char **argv)
{
	calibrationCfg calibInfo;
	programCfg programInfo;
	stereoModeData stereoModeInfo;
	
	ConfigParser cp(argc, argv);
	switch (cp.parse(calibInfo, programInfo, stereoModeInfo))
	{
	case SUCCESS:
		break;
	case SUCCESS_CLOSE:
		return 0;
	case FAILURE:
		return 1;
	}

	//Camera handlers
	VideoCapture capture1(0), capture2(1);
	camData cam1(NULL), cam2(NULL);
	camPoints points1, points2;

	//Setting intrinsics camera parameters (either from file or default (neutral) ones).
	initIntrinsicsCamParams(programInfo.mSwapCameras, cam1, cam2, programInfo.mIntrinsics, capture1, capture2);

	cvNamedWindow("Cam1", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Cam2", CV_WINDOW_AUTOSIZE);
	Mat frame1, frame2, oFrame1, oFrame2, R, T, Q;

	capture1 >> frame1;
	capture2 >> frame2;

	cam1.mSize = frame1.size();
	cam2.mSize = frame2.size();

	Size oldSize(frame2.cols, frame2.rows);

	//Acquiring camera extrinsics parameters either from calibration or restoring from file.
	//Alternatively this step may be skipped or omitted if '--skip' parameter is passed, or [esc]
	//key is pressed before acquiring enough pictures during calibration or if file containing
	//extrinsics parameters could not be opened.
	Mat left, right;
	if (!programInfo.mStatic)
	{
		if (!calibInfo.mSkip)
		{
			if (aquireCalibData(cam1, cam2, points1, points2, calibInfo.mCalibSamples, calibInfo.mBoardSize, calibInfo.mDelay) >= 2)
			{
				if (!calibrateCameras(cam1, cam2, points1, points2, calibInfo.mBoardSize, calibInfo.mSquareSize, R, T, Q))
				{
					std::cerr << "Error calibrating cameras!";
					return 2;
				}
			}
			else
				std::cerr << "Acquired to few samples for calibration, skipping!\n";
		}
		else
		{
			initExtrinsicsCamParams(programInfo.mExtrinsics, calibInfo.mSkip, R, T, Q, cam1, cam2);
		}
	}
	else
	{
		left = imread(programInfo.mLeft, CV_LOAD_IMAGE_GRAYSCALE);
		right = imread(programInfo.mRight, CV_LOAD_IMAGE_GRAYSCALE);
	}

	stereoModeInfo.camera.mCam1 = &cam1;
	stereoModeInfo.camera.mCam2 = &cam2;
	stereoModeInfo.camera.R = R;
	stereoModeInfo.camera.T = T;
	stereoModeInfo.camera.Q = Q;

	if(stereoModeInfo.mMode >= SM_GPU_BM)
	{
		if(!initializeCUDA())
		{
			cerr << "\n\nFailed to load CUDA device!\n";
			return 4;
		}
		else
			cout << "\n\n Loaded CUDA device.\n";
	}

	StereoCam *scCam;
	if (programInfo.mStatic)
		scCam = new StereoCam(stereoModeInfo, "Cam1", "Cam2", "StereoCam", left, right, programInfo.mShifted);
	else
		scCam = new StereoCam(stereoModeInfo, "Cam1", "Cam2", "StereoCam", programInfo.mShifted);

	cvNamedWindow("StereoCam", CV_WINDOW_AUTOSIZE);

	int waitPeriod = (programInfo.mStatic ? 0 : 33);

	// Live display
	while (1)
	{
		scCam->process(calibInfo.mSkip);

		char c = cvWaitKey(waitPeriod);
		if (c == 27)
			break;
	}
	//Closing camera handles and opened windows.
	capture1.release();
	capture2.release();
	cvDestroyWindow("Cam1");
	cvDestroyWindow("Cam2");
	cvDestroyWindow("StereoCam");

	//Writing intrinsics and extrinsics camera parameters to backup files.
	saveLastCameraParams(cam1, cam2, scCam);
	delete scCam;

	return 0;
}

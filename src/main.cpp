/*
 * main.cpp
 *
 *  Created on: 08-06-2013
 *      Author: Krzysztof Pilch
 */

#include "undistorted_cam.h"

int main(int argc, char **argv)
{
	cout<< SHRT_MAX <<endl;
	//Parsing program options from command line.
	po::variables_map varMap;
	po::options_description *conCfg = NULL;
	po::options_description *fileCfg = NULL;
	switch (programOptions(varMap, conCfg, fileCfg, argc, argv))
	{
	case SUCCESS:
		break;
	case SUCCESS_CLOSE:
		return 0;
	case FAILURE:
		return 1;
	}

	//parameters that can be defined using command line options.
//	Size boardSize = Size(9, 6);

	calibrationCfg calibInfo;
	programCfg programInfo;
	stereoModeData stereoModeInfo;

	//Retrieving parsed command line options.
	if (!getRuntimeConfiguration(varMap, calibInfo, programInfo, stereoModeInfo))
	{
		cerr << "\nThis program options may be specified both from the command line and config.cfg file:\n"
				<< "(parameters specified in console override those from file)\n";
		conCfg->print(cerr);
		cerr << "\nThis program options may be specified only in config.cfg file:\n";
		fileCfg->print(cerr);
		cerr << endl;

	}

	//Camera handlers
	VideoCapture capture1(0), capture2(1);
	camData cam1(NULL), cam2(NULL);
	camPoints points1, points2;

	//Setting intrinsics camera parameters (either from file or default (neutral) ones.
	initIntrinsicsCamParams(programInfo.mSwapCameras, cam1, cam2, programInfo.mIntrinsics, capture1, capture2);

	cvNamedWindow("Cam1", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Cam2", CV_WINDOW_AUTOSIZE);
	Mat frame1, frame2, oFrame1, oFrame2, R, T, Q;

	capture1 >> frame1;
	capture2 >> frame2;

	cam1.mSize = frame1.size();
	cam2.mSize = frame2.size();

	Size newSize(2 * frame1.cols, 2 * frame1.rows), oldSize(frame2.cols, frame2.rows);

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

	StereoCam *scCam;
	if (programInfo.mStatic)
		scCam = new StereoCam(stereoModeInfo, "Cam1", "Cam2", "StereoCam", left, right);
	else
		scCam = new StereoCam(stereoModeInfo, "Cam1", "Cam2", "StereoCam", programInfo.mShifted);

	cvNamedWindow("StereoCam", CV_WINDOW_AUTOSIZE);

	Mat gray1(oldSize, CV_8UC1), gray2(oldSize, CV_8UC1);

	// Live display
	while (1)
	{
		scCam->process(calibInfo.mSkip);

		char c = cvWaitKey(33);
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

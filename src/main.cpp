/*
 * main.cpp
 *
 *  Created on: 08-06-2013
 *      Author: Krzysztof Pilch
 */

#include "undistorted_cam.h"

int main(int argc, char **argv)
{
	//Parsing program options from command line.
	po::variables_map varMap;
	po::options_description *optDesc = NULL;
	switch (programOptions(varMap, optDesc, argc, argv))
	{
	case SUCCESS:
		break;
	case SUCCESS_CLOSE:
		return 0;
	case FAILURE:
		return 1;
	}

	//parameters that can be defined using command line options.
	bool staticImages = false, skipCalib = false, swapCameras = false;
	string leftImg, rightImg, intrParamsFile, extrParamsFile;
	int disparitiesCnt = 16;
	int sadWindowSize = 21;
	int sampleNums = 25;
	int delay = 1000;
	float squareSize = 25.5;
	Size boardSize = Size(9, 6);
	StereoCam::SHIFT_CAM shiftedCam = StereoCam::NONE;

	//Retrieving parsed command line options.
	getRuntimeConfiguration(varMap, staticImages, leftImg, rightImg, skipCalib, disparitiesCnt, sadWindowSize,
			sampleNums, delay, squareSize, boardSize, swapCameras, intrParamsFile, extrParamsFile, shiftedCam);

	//Camera handlers
	VideoCapture capture1(0), capture2(1);
	camData cam1(NULL), cam2(NULL);
	camPoints points1, points2;

	//Setting intrinsics camera parameters (either from file or default (neutral) ones.
	initIntrinsicsCamParams(swapCameras, cam1, cam2, intrParamsFile, capture1, capture2);

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
	if (!staticImages)
	{
		if (!skipCalib)
		{
			if (aquireCalibData(cam1, cam2, points1, points2, sampleNums, boardSize, delay) >= 2)
			{
				if (!calibrateCameras(cam1, cam2, points1, points2, boardSize, squareSize, R, T, Q))
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
			initExtrinsicsCamParams(extrParamsFile, skipCalib, R, T, Q, cam1, cam2);
		}
	}
	else
	{
		left = imread(leftImg, CV_LOAD_IMAGE_GRAYSCALE);
		right = imread(rightImg, CV_LOAD_IMAGE_GRAYSCALE);
	}

	//Creating stereo camera.
	StereoCam *scCam;
	if (staticImages)
		scCam = new StereoCam(disparitiesCnt, sadWindowSize, "Cam1", "Cam2", "StereoCam", left, right);
	else
		scCam = new StereoCam(disparitiesCnt, sadWindowSize, "Cam1", "Cam2", "StereoCam", &cam1, &cam2, R, T, Q,
				shiftedCam);

	cvNamedWindow("StereoCam", CV_WINDOW_AUTOSIZE);

	Mat gray1(oldSize, CV_8UC1), gray2(oldSize, CV_8UC1);

	// Live display
	while (1)
	{
		scCam->process(skipCalib);

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

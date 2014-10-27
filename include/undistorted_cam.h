/*
 * undistorted_cam.h
 *
 *  Created on: 07-07-2013
 *      Author: Krzysztof Pilch
 */

#ifndef UNDISTORTED_CAM_H_
#define UNDISTORTED_CAM_H_

#include "StereoCam.h"
#include <boost/program_options.hpp>

using namespace std;
namespace po = boost::program_options;

enum optionReadStatus
{
	SUCCESS = 0, SUCCESS_CLOSE, FAILURE
};

struct camPoints
{
	vector<vector<Point2f> > mPoints;
};

struct calibrationCfg
{
	bool mSkip;
	int mCalibSamples;
	int mDelay;
	float mSquareSize;
	int mBoardWidth;
	int mBoardHeight;
	Size mBoardSize;

public:
	calibrationCfg()
	{
		mSkip = false;
		mCalibSamples = 25;
		mDelay = 1000;
		mSquareSize = 25.5;
		mBoardWidth = 9;
		mBoardHeight = 6;
	}
};

struct programCfg
{
	string mAlgorithm;
	string mLeft;
	string mRight;
	bool mStatic;
	string mReferenceDisparity;
	bool mSilent;
	StereoCam::SHIFT_CAM mShifted;
	string mIntrinsics;
	string mExtrinsics;
	bool mSwapCameras;

public:
	programCfg()
	{
		mAlgorithm = "bm";
		mLeft = "";
		mRight = "";
		mStatic = false;
		mReferenceDisparity = "";
		mSilent = false;
		mShifted = StereoCam::NONE;
		mIntrinsics = "";
		mExtrinsics = "";
		mSwapCameras = false;
	}
};

void initIntrinsicsCamParams(bool swapCameras, camData &cam1, camData &cam2, const string& intrParamsFile, VideoCapture& capture1,
		VideoCapture& capture2);

void rectifyCameras(camData &cam1, camData &cam2, Mat& R, Mat &T, Mat &Q);

void initExtrinsicsCamParams(const string& extrParamsFile, bool& skipCalib, Mat& R, Mat& T, Mat& Q, camData& cam1, camData& cam2);

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
{
	corners.clear();

	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			corners.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
}

unsigned aquireCalibData(camData &cam1, camData &cam2, camPoints &points1, camPoints &points2, unsigned sampleNums, Size boardSize, int delay);

bool calibrateCameras(camData &cam1, camData &cam2, camPoints &points1, camPoints &points2, Size boardSize, float squareSize, Mat &R, Mat &T, Mat &Q);

void saveLastCameraParams(const camData& cam1, const camData& cam2, StereoCam* scCam);

bool initializeCUDA(int chosen_cuda_card = 0);

#endif //UNDISTORTED_CAM_H_

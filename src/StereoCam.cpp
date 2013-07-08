/*
 * StereoCam.cpp
 *
 *  Created on: 07-07-2013
 *      Author: Krzysztof Pilch
 */

#include "StereoCam.h"
#include <iostream>

using namespace std;

StereoCam::StereoCam()
{
	mMode = invalidMode;
	mShiftMode = NONE;
	mCam1 = NULL;
	mCam2 = NULL;
}

StereoCam::StereoCam(int disparitiesCnt, int sadWindowSize, std::string window1, std::string window2,
						std::string disparityWindow, camData *cam1, camData *cam2, Mat R, Mat T, Mat Q, SHIFT_CAM mode) :
		mCam1(cam1), mCam2(cam2), mWindow1(window1), mWindow2(window2), mDisparityWindow(disparityWindow), mR(R), mT(T),
				mQ(Q), mStereoCam(StereoBM::BASIC_PRESET, disparitiesCnt, sadWindowSize)
{
	if (mode == NONE)
	{
		mMode = normalCam;
		mShiftMode = NONE;
	}
	else
	{
		mMode = shiftedCam;
		mShiftMode = mode;
	}
}

StereoCam::StereoCam(int disparitiesCnt, int sadWindowSize, std::string window1, std::string window2,
						std::string disparityWindow, Mat &left, Mat &right) :
		mCam1(NULL), mCam2(NULL), mWindow1(window1), mWindow2(window2), mDisparityWindow(disparityWindow),
				mStereoCam(StereoBM::BASIC_PRESET, disparitiesCnt, sadWindowSize), mMode(StereoCam::staticImg),
				mShiftMode(NONE)
{
	left.copyTo(mLeft);
	right.copyTo(mRight);
//	cvtColor(left, mLeft, CV_RGB2GRAY);
//	cvtColor(right, mRight, CV_RGB2GRAY);
}

void StereoCam::process(bool skipRemap)
{
	Mat frame1, frame2, oFrame1, oFrame2, gray1, gray2, dispFrame;

//only for shifted fragment
	int offset = 50;
	if (mShiftMode == RIGHT)
		offset = -50;
	Rect initialFrame(150, 150, 300, 300);
	Rect finalFrame(initialFrame.x + offset, initialFrame.y, initialFrame.width, initialFrame.height);
	Mat push, cleared, pushed;
//end shifted variables

	switch (mMode)
	{
	case normalCam:
		*mCam1->mCap >> frame1;
		*mCam2->mCap >> frame2;

		if (frame1.empty() || frame2.empty())
			return;

		if (skipRemap)
		{
			oFrame1 = frame1;
			oFrame2 = frame2;
		}
		else
		{
			remap(frame1, oFrame1, mCam1->mMap1, mCam1->mMap2, INTER_LANCZOS4);
			remap(frame2, oFrame2, mCam2->mMap1, mCam2->mMap2, INTER_LANCZOS4);
		}

		cvtColor(oFrame1, gray1, CV_RGB2GRAY);
		cvtColor(oFrame2, gray2, CV_RGB2GRAY);

		break;
	case shiftedCam:
		*mCam1->mCap >> frame1;
		*mCam2->mCap >> frame2;

		if (frame1.empty() || frame2.empty())
			return;

		if (skipRemap)
		{
			oFrame1 = frame1;
			oFrame2 = frame2;
		}
		else
		{
			remap(frame1, oFrame1, mCam1->mMap1, mCam1->mMap2, INTER_LANCZOS4);
			remap(frame2, oFrame2, mCam2->mMap1, mCam2->mMap2, INTER_LANCZOS4);
		}

		cvtColor(oFrame1, gray1, CV_RGB2GRAY);
		cvtColor(oFrame2, gray2, CV_RGB2GRAY);

		switch (mShiftMode)
		{
		case LEFT:
			push = Mat(gray1, initialFrame);
			cleared = Mat(gray2, initialFrame);
			pushed = Mat(gray2, finalFrame);

			gray1.copyTo(gray2);
			break;
		case RIGHT:
			push = Mat(gray2, initialFrame);
			cleared = Mat(gray1, initialFrame);
			pushed = Mat(gray1, finalFrame);

			gray2.copyTo(gray1);
			break;
		case NONE:
		default:
			throw "Shift can not go with NONE mode! Make sure to pass \"left\" or \"right\" to \"--shifted\" parameter!.";
		}

		cleared = Scalar(0, 0, 0);
		push.copyTo(pushed);

		break;
	case staticImg:
		mLeft.copyTo(gray1);
		mRight.copyTo(gray2);
		break;
	case invalidMode:
	default:
		return;
	}
	mStereoCam(gray1, gray2, dispFrame, CV_32F);

	double min, max;
	minMaxIdx(dispFrame, &min, &max);

//	cout << "min: " << min << "\tmax: " << max << endl;
	dispFrame = (dispFrame + 1) / (max + 1);

	imshow(mWindow1, gray1);
	imshow(mWindow2, gray2);
	imshow(mDisparityWindow, dispFrame);
}

const Mat& StereoCam::getR()
{
	return mR;
}

const Mat& StereoCam::getT()
{
	return mT;
}

const Mat& StereoCam::getQ()
{
	return mQ;
}

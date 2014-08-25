/*
 * StereoCam.cpp
 *
 *  Created on: 07-07-2013
 *      Author: Krzysztof Pilch
 */

#include "StereoCam.h"
#include <opencv2/gpu/gpu.hpp>
#include <iostream>

using namespace std;

stereoModeData::stereoModeData()
{
	mMode = SM_BLOCK_MACHING;

	camera.mCam1 = NULL;
	camera.mCam2 = NULL;

	common.mMaxDisp = -1;
	common.mMinDisp = -1;
	common.mLevels = -1;

	gpuCommon.mNdisp = -1;
	gpuCommon.mIters = -1;
	gpuCommon.mMsgType = -1;
	gpuCommon.mMaxDataTerm = -1;
	gpuCommon.mDataWeight = -1;
	gpuCommon.mMaxDiscTerm = -1;
	gpuCommon.mDiscSingleJump = -1;

	blockMaching.mSadWindowSize = -1;

	var.mPyrScale = -1;
	var.mnIt = -1;
	var.mPolyN = -1;
	var.mPolySigma = -1;
	var.mFi = -1;
	var.mLambda = -1;
	var.mPenalization = -1;
	var.mCycle = -1;
	var.mFlags = -1;

	//Belief Propagation

	constantSpaceBP.mNrPlane = -1;
}

StereoCam::StereoCam()
		: mData()
{
	mMode = invalidMode;
	mShiftMode = NONE;
	mStereoDevice = NULL;
}

StereoCam::StereoCam(stereoModeData &data, std::string window1, std::string window2, std::string disparityWindow, SHIFT_CAM mode)
		: mData(data), mWindow1(window1), mWindow2(window2), mDisparityWindow(disparityWindow), mStereoDevice(NULL)
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

	mStereoDevice = createStereoDevice(mData);
}

StereoCam::StereoCam(stereoModeData &data, std::string window1, std::string window2, std::string disparityWindow, Mat &left, Mat &right)
		: mData(data), mWindow1(window1), mWindow2(window2), mDisparityWindow(disparityWindow), mStereoDevice(NULL), mMode(StereoCam::staticImg), mShiftMode(
				NONE)
{
	left.copyTo(mLeft);
	right.copyTo(mRight);
//	cvtColor(left, mLeft, CV_RGB2GRAY);
//	cvtColor(right, mRight, CV_RGB2GRAY);

	mStereoDevice = createStereoDevice(mData);
}

StereoCam::~StereoCam()
{
	if(mStereoDevice != NULL)
	{
		delete mStereoDevice;
		mStereoDevice = NULL;
	}
}

StereoCam::iStereoDevice* StereoCam::createStereoDevice(stereoModeData &data)
{
	iStereoDevice *device = NULL;
	switch (data.mMode)
	{
	case SM_BLOCK_MACHING:
		device = new StereoContainer<StereoBM>(new StereoBM(StereoBM::BASIC_PRESET, data.common.mMaxDisp, data.blockMaching.mSadWindowSize));
		break;
	case SM_VAR:
		device = new StereoContainer<StereoVar>(
				new StereoVar(data.common.mLevels, data.var.mPyrScale, data.var.mnIt, data.common.mMinDisp, data.common.mMaxDisp, data.var.mPolyN,
						data.var.mPolySigma, data.var.mFi, data.var.mLambda, data.var.mPenalization, data.var.mCycle, data.var.mFlags));
		break;
	case SM_BELIEF_PROPAGATION:
		device = new StereoContainer<gpu::StereoBeliefPropagation>(
				new gpu::StereoBeliefPropagation(data.gpuCommon.mNdisp, data.gpuCommon.mIters, data.common.mLevels, data.gpuCommon.mMaxDataTerm,
						data.gpuCommon.mDataWeight, data.gpuCommon.mMaxDiscTerm, data.gpuCommon.mDiscSingleJump, data.gpuCommon.mMsgType));
		break;
	case SM_CONSTANT_SPACE_BP:
		device = new StereoContainer<gpu::StereoConstantSpaceBP>(
				new gpu::StereoConstantSpaceBP(data.gpuCommon.mNdisp, data.gpuCommon.mIters, data.common.mLevels, data.constantSpaceBP.mNrPlane,
						data.gpuCommon.mMaxDataTerm, data.gpuCommon.mDataWeight, data.gpuCommon.mMaxDiscTerm, data.gpuCommon.mDiscSingleJump,
						data.common.mMinDisp, data.gpuCommon.mMsgType));
		break;
	default:
		break;
	}

	return device;
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
		*mData.camera.mCam1->mCap >> frame1;
		*mData.camera.mCam2->mCap >> frame2;

		if (frame1.empty() || frame2.empty())
			return;

		if (skipRemap)
		{
			oFrame1 = frame1;
			oFrame2 = frame2;
		}
		else
		{
			remap(frame1, oFrame1, mData.camera.mCam1->mMap1, mData.camera.mCam1->mMap2, INTER_LANCZOS4);
			remap(frame2, oFrame2, mData.camera.mCam2->mMap1, mData.camera.mCam2->mMap2, INTER_LANCZOS4);
		}

		cvtColor(oFrame1, gray1, CV_RGB2GRAY);
		cvtColor(oFrame2, gray2, CV_RGB2GRAY);

		break;
	case shiftedCam:
		*mData.camera.mCam1->mCap >> frame1;
		*mData.camera.mCam2->mCap >> frame2;

		if (frame1.empty() || frame2.empty())
			return;

		if (skipRemap)
		{
			oFrame1 = frame1;
			oFrame2 = frame2;
		}
		else
		{
			remap(frame1, oFrame1, mData.camera.mCam1->mMap1, mData.camera.mCam1->mMap2, INTER_LANCZOS4);
			remap(frame2, oFrame2, mData.camera.mCam2->mMap1, mData.camera.mCam2->mMap2, INTER_LANCZOS4);
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
	(*mStereoDevice)(gray1, gray2, dispFrame, CV_32F);

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
	return mData.camera.R;
}

const Mat& StereoCam::getT()
{
	return mData.camera.T;
}

const Mat& StereoCam::getQ()
{
	return mData.camera.Q;
}

// --- StereoCam::StereoContainer ---
template<>
void StereoCam::StereoContainer<StereoBM>::operator ()(const Mat &left, const Mat &right, Mat &disparity, int disptype)
{
	mStereoDevice->operator ()(left, right, disparity, disptype);
}

template<>
void StereoCam::StereoContainer<StereoVar>::operator ()(const Mat &left, const Mat &right, Mat &disparity, int disptype)
{
	mStereoDevice->operator ()(left, right, disparity);
}

template<>
void StereoCam::StereoContainer<gpu::StereoBeliefPropagation>::operator ()(const Mat &left, const Mat &right, Mat &disparity, int disptype)
{
	gpu::GpuMat gpuLeft, gpuRight, gpuDisp;
	gpuLeft.upload(left);
	gpuRight.upload(right);
	mStereoDevice->operator ()(gpuLeft, gpuRight, gpuDisp);
	gpuDisp.download(disparity);
}

template<>
void StereoCam::StereoContainer<gpu::StereoConstantSpaceBP>::operator ()(const Mat &left, const Mat &right, Mat &disparity, int disptype)
{
	gpu::GpuMat gpuLeft, gpuRight, gpuDisp;
	gpuLeft.upload(left);
	gpuRight.upload(right);
	mStereoDevice->operator ()(gpuLeft, gpuRight, gpuDisp);
	gpuDisp.download(disparity);
}

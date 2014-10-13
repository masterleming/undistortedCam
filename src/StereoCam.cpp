/*
 * StereoCam.cpp
 *
 *  Created on: 07-07-2013
 *      Author: Krzysztof Pilch
 */

#include "StereoCam.h"
#include <iostream>

using namespace std;

stereoModeData::stereoModeData()
{
	mMode = SM_INVALID;

	camera.mCam1 = NULL;
	camera.mCam2 = NULL;
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

	mStereoDevice = StereoFactory::createStereoDevice(mData);
}

StereoCam::StereoCam(stereoModeData &data, std::string window1, std::string window2, std::string disparityWindow, Mat &left, Mat &right)
		: mData(data), mWindow1(window1), mWindow2(window2), mDisparityWindow(disparityWindow), mStereoDevice(NULL), mMode(StereoCam::staticImg), mShiftMode(
				NONE)
{
	left.copyTo(mLeft);
	right.copyTo(mRight);
//	cvtColor(left, mLeft, CV_RGB2GRAY);
//	cvtColor(right, mRight, CV_RGB2GRAY);

	mStereoDevice = StereoFactory::createStereoDevice(mData);
}

StereoCam::~StereoCam()
{
	if(mStereoDevice != NULL)
	{
		delete mStereoDevice;
		mStereoDevice = NULL;
	}
}

StereoCam::iStereoDevice* StereoCam::StereoFactory::createStereoDevice(stereoModeData &data)
{
	iStereoDevice *device = NULL;
	StereoBM *dev = NULL;
	switch (data.mMode)
	{
	case SM_BLOCK_MACHING:
		device = new StereoContainer<StereoBM>(data);
		break;
	case SM_SEMI_GLOBAL_BM:
		device = new StereoContainer<StereoSGBM>(data);
		break;
	case SM_VAR:
		device = new StereoContainer<StereoVar>(data);
		break;
	case SM_GPU_BM:
		device = new StereoContainer<gpu::StereoBM_GPU>(data);
		break;
	case SM_BELIEF_PROPAGATION:
		device = new StereoContainer<gpu::StereoBeliefPropagation>(data);
		break;
	case SM_CONSTANT_SPACE_BP:
		device = new StereoContainer<gpu::StereoConstantSpaceBP>(data);
		break;
	default:
		throw "Unrecognised algorithm";
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

	int64 t = getTickCount();
	(*mStereoDevice)(gray1, gray2, dispFrame, CV_16S); //CV_32F);
    t = getTickCount() - t;
//	cout << "Time elapsed: " << t*1000/getTickFrequency() << endl;

	double min, max;

	Mat dispFrame8;

	minMaxIdx(dispFrame, &min, &max);
//	cout << "dispFrame\t" << "min: " << min << "\tmax: " << max << "\ttype: " << dispFrame.type() << "\tempty? " << (dispFrame.empty() ? "YES" : "NO") << endl;

	dispFrame = (255 * (dispFrame - min)) / (max - min);
//	minMaxIdx(dispFrame, &min, &max);
//	cout << "dispFrame\t" << "min: " << min << "\tmax: " << max << "\ttype: " << dispFrame8.type() << "\tempty? " << (dispFrame.empty() ? "YES" : "NO") << endl;


	dispFrame.convertTo(dispFrame8, CV_8U);
//	minMaxIdx(dispFrame8, &min, &max);
//	cout << "dispFrame8\t" << "min: " << min << "\tmax: " << max << "\ttype: " << dispFrame8.type() << "\tempty? " << (dispFrame8.empty() ? "YES" : "NO") << endl;
//
//	dispFrame8 = (255 * (dispFrame8 - min)) / (max - min);
//	
//	minMaxIdx(dispFrame8, &min, &max);
//	cout << "dispFrame8\t" << "min: " << min << "\tmax: " << max << "\ttype: " << dispFrame8.type() << "\tempty? " << (dispFrame8.empty() ? "YES" : "NO") << endl;

	imshow(mWindow1, gray1);
	imshow(mWindow2, gray2);
	imshow(mDisparityWindow, dispFrame8);
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
//StereoBM
template<>
StereoCam::StereoContainer<StereoBM>::StereoContainer(stereoModeData &data)
{
		mStereoDevice = StereoBM(data.mAlgorithmData.blockMatching.mPreset,
							data.mAlgorithmData.blockMatching.mDisparities,
							data.mAlgorithmData.blockMatching.mSadWindowSize);
		
		if(data.mAlgorithmData.blockMatching.mSpeckleWindowSize != -1)
			mStereoDevice.state->speckleWindowSize = data.mAlgorithmData.blockMatching.mSpeckleWindowSize;

		if(data.mAlgorithmData.blockMatching.mSpeckleRange != -1)
			mStereoDevice.state->speckleRange = data.mAlgorithmData.blockMatching.mSpeckleRange;

		if(data.mAlgorithmData.blockMatching.mDisp12MaxDiff != -1)
			mStereoDevice.state->disp12MaxDiff = data.mAlgorithmData.blockMatching.mDisp12MaxDiff;
}

template<>
void StereoCam::StereoContainer<StereoBM>::operator ()(const Mat &left, const Mat &right, Mat &disparity, int disptype)
{
	mStereoDevice(left, right, disparity, disptype);
}

//StereoSGBM
template<>
StereoCam::StereoContainer<StereoSGBM>::StereoContainer(stereoModeData &data)
{
	mStereoDevice = StereoSGBM(data.mAlgorithmData.semiGlobalBM.mMinDisp,
								data.mAlgorithmData.semiGlobalBM.mDisparities,
								data.mAlgorithmData.semiGlobalBM.mSadWindowSize,
								data.mAlgorithmData.semiGlobalBM.mP1,
								data.mAlgorithmData.semiGlobalBM.mP2,
								data.mAlgorithmData.semiGlobalBM.mDisp12MaxDiff,
								data.mAlgorithmData.semiGlobalBM.mPreFilterCap,
								data.mAlgorithmData.semiGlobalBM.mUniquenessRatio,
								data.mAlgorithmData.semiGlobalBM.mSadWindowSize,
								data.mAlgorithmData.semiGlobalBM.mSpeckleRange,
								data.mAlgorithmData.semiGlobalBM.mFullDP);
}

//StereoVAR
template<>
StereoCam::StereoContainer<StereoVar>::StereoContainer(stereoModeData &data)
{
	mStereoDevice = StereoVar(data.mAlgorithmData.var.mLevels,
								data.mAlgorithmData.var.mPyrScale,
								data.mAlgorithmData.var.mIteratnions,
								data.mAlgorithmData.var.mMinDisp,
								data.mAlgorithmData.var.mMaxDisp,
								data.mAlgorithmData.var.mPolyN,
								data.mAlgorithmData.var.mPolySigma,
								data.mAlgorithmData.var.mFi,
								data.mAlgorithmData.var.mLambda,
								data.mAlgorithmData.var.mPenalization,
								data.mAlgorithmData.var.mCycle,
								data.mAlgorithmData.var.mFlags);
}

//Stereo_BM_GPU
template<>
StereoCam::StereoContainer<gpu::StereoBM_GPU>::StereoContainer(stereoModeData &data)
{
	mStereoDevice = gpu::StereoBM_GPU(data.mAlgorithmData.gpuBlockMatching.mPreset,
										data.mAlgorithmData.gpuBlockMatching.mDisparities,
										data.mAlgorithmData.gpuBlockMatching.mWindowSize);
}

template<>
void StereoCam::StereoContainer<gpu::StereoBM_GPU>::operator ()(const Mat &left, const Mat &right, Mat &disparity, int disptype)
{
	gpu::GpuMat gpuLeft, gpuRight, gpuDisp(left.rows, left.cols, CV_8U);
	gpuLeft.upload(left);
	gpuRight.upload(right);
	mStereoDevice(gpuLeft, gpuRight, gpuDisp);
	gpuDisp.download(disparity);
}

//StereoBeliefPropagation
template<>
StereoCam::StereoContainer<gpu::StereoBeliefPropagation>::StereoContainer(stereoModeData &data)
{
	mStereoDevice = gpu::StereoBeliefPropagation(data.mAlgorithmData.beliefPropagation.mDisparities,
												data.mAlgorithmData.beliefPropagation.mIterations,
												data.mAlgorithmData.beliefPropagation.mLevels,
												data.mAlgorithmData.beliefPropagation.mMaxDataTerm,
												data.mAlgorithmData.beliefPropagation.mDataWeight,
												data.mAlgorithmData.beliefPropagation.mMaxDiscTerm,
												data.mAlgorithmData.beliefPropagation.mDiscSingleJump,
												data.mAlgorithmData.beliefPropagation.mMsgType);
}

template<>
void StereoCam::StereoContainer<gpu::StereoBeliefPropagation>::operator ()(const Mat &left, const Mat &right, Mat &disparity, int disptype)
{
	gpu::GpuMat gpuLeft, gpuRight, gpuDisp(left.rows, left.cols, CV_8U);
	gpuLeft.upload(left);
	gpuRight.upload(right);
	mStereoDevice(gpuLeft, gpuRight, gpuDisp);
	gpuDisp.download(disparity);
}

//StereoConstantSpace
template<>
StereoCam::StereoContainer<gpu::StereoConstantSpaceBP>::StereoContainer(stereoModeData &data)
{
	mStereoDevice = gpu::StereoConstantSpaceBP(data.mAlgorithmData.beliefPropagation.mDisparities,
												data.mAlgorithmData.beliefPropagation.mIterations,
												data.mAlgorithmData.beliefPropagation.mLevels,
												data.mAlgorithmData.beliefPropagation.mNrPlane,
												data.mAlgorithmData.beliefPropagation.mMaxDataTerm,
												data.mAlgorithmData.beliefPropagation.mDataWeight,
												data.mAlgorithmData.beliefPropagation.mMaxDiscTerm,
												data.mAlgorithmData.beliefPropagation.mDiscSingleJump,
												data.mAlgorithmData.beliefPropagation.mMinDispTh,
												data.mAlgorithmData.beliefPropagation.mMsgType);
}

template<>
void StereoCam::StereoContainer<gpu::StereoConstantSpaceBP>::operator ()(const Mat &left, const Mat &right, Mat &disparity, int disptype)
{
	gpu::GpuMat gpuLeft, gpuRight, gpuDisp;
	gpuLeft.upload(left);
	gpuRight.upload(right);
	mStereoDevice(gpuLeft, gpuRight, gpuDisp);
	gpuDisp.download(disparity);
}

//general
template<class T>
StereoCam::StereoContainer<T>::StereoContainer(stereoModeData &data)
{
	throw "Unsupported class";
}

template<class T>
void StereoCam::StereoContainer<T>::operator() (const Mat &left, const Mat &right, Mat &disparity, int disptype)
{
	mStereoDevice(left, right, disparity);
}
/*
 * StereoCam.h
 *
 *  Created on: 07-07-2013
 *      Author: Krzysztof Pilch
 */

#ifndef STEREOCAM_H_
#define STEREOCAM_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
//#include <opencv2/>
#include <opencv2/opencv_modules.hpp>

using namespace cv;

enum StereoMode
{
	SM_BLOCK_MACHING = 0, SM_VAR, SM_BELIEF_PROPAGATION, SM_CONSTANT_SPACE_BP
};

struct camData
{
	//camera source
	VideoCapture *mCap;
	//preset camera parameters
	Mat mCamMat;
	Mat mDistMat;
	//calculated camera parameters
	Mat mMap1;
	Mat mMap2;
	Mat mR;
	Mat mP;
	Rect mValidRoi;
	Size mSize;

	//methods
public:
	camData(VideoCapture *cap)
			: mCap(cap)
	{
	}
};

struct stereoModeData
{
	StereoMode mMode;
	struct
	{
		camData *mCam1;
		camData *mCam2;
		Mat R;
		Mat T;
		Mat Q;
	} camera;

	struct
	{
		int mMaxDisp;
		int mMinDisp;
		int mLevels;
	} common;

	struct
	{
		int mNdisp;
		int mIters;
		int mMsgType;
		float mMaxDataTerm;
		float mDataWeight;
		float mMaxDiscTerm;
		float mDiscSingleJump;
	} gpuCommon;

	struct
	{
		int mSadWindowSize;
	} blockMaching;

	struct
	{
		double mPyrScale;
		int mnIt;
		int mPolyN;
		double mPolySigma;
		float mFi;
		float mLambda;
		int mPenalization;
		int mCycle;
		int mFlags;
	} var;

	struct
	{
	} beliefPropagation;

	struct
	{
		int mNrPlane;
	} constantSpaceBP;

public:
	stereoModeData();
};

class StereoCam
{
private:
	enum SC_MODE
	{
		invalidMode = -1, normalCam = 0, shiftedCam, staticImg
	};

public:
	enum SHIFT_CAM
	{
		LEFT = -1, NONE, RIGHT
	};

private:
	class iStereoDevice
	{
	protected:
		iStereoDevice()
		{
		}

	public:
		virtual void operator ()(const Mat &left, const Mat &right, Mat &disparity, int disptype) = 0;
		virtual ~iStereoDevice()
		{
		}
	};

	template<class T>
	class StereoContainer: public iStereoDevice
	{
	private:
		T* mStereoDevice;

		StereoContainer()
		{
			mStereoDevice = NULL;
		}

	public:
		StereoContainer(T* device)
				: iStereoDevice(), mStereoDevice(device)
		{
		}
		~StereoContainer()
		{
			if (mStereoDevice != NULL)
			{
				delete mStereoDevice;
				mStereoDevice = NULL;
			}
		}

		void operator ()(const Mat &left, const Mat &right, Mat &disparity, int disptype);
	};

	static iStereoDevice* createStereoDevice(stereoModeData &data);

private:
	stereoModeData mData;

	Mat mLeft;
	Mat mRight;

	std::string mWindow1;
	std::string mWindow2;
	std::string mDisparityWindow;

	iStereoDevice *mStereoDevice;

	SC_MODE mMode;
	SHIFT_CAM mShiftMode;

	StereoCam();

public:
	StereoCam(stereoModeData &data, std::string window1, std::string window2, std::string disparityWindow, SHIFT_CAM mode = NONE);
	StereoCam(stereoModeData &data, std::string window1, std::string window2, std::string disparityWindow, Mat &left, Mat &right);
	~StereoCam();

	void process(bool skipRemap);

	const Mat& getR();
	const Mat& getT();
	const Mat& getQ();
};

#endif /* STEREOCAM_H_ */

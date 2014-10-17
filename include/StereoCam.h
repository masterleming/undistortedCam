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
#include <opencv2/gpu/gpu.hpp>

using namespace cv;

enum StereoMode
{
	SM_INVALID = -1,
	SM_BLOCK_MACHING = 0,
	SM_SEMI_GLOBAL_BM,
	SM_VAR,
	SM_GPU_BM,
	SM_BELIEF_PROPAGATION,
	SM_CONSTANT_SPACE_BP
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
	struct blockMaching_data
	{
		int mSadWindowSize;
		int mDisparities;
		int mPreset;
		int mSpeckleWindowSize;
		int mSpeckleRange;
		int mDisp12MaxDiff;
	};

	struct semiGlobalBM_data
	{
		int mMinDisp;
		int mDisparities;
		int mSadWindowSize;
		int mP1;
		int mP2;
		int mDisp12MaxDiff;
		int mPreFilterCap;
		int mUniquenessRatio;
		int mSpeckleWindowsSize;
		int mSpeckleRange;
		bool mFullDP;
	};

	struct var_data
	{
		int mLevels;
		double mPyrScale;
		int mIteratnions;
		int mMinDisp;
		int mMaxDisp;
		int mPolyN;
		double mPolySigma;
		float mFi;
		float mLambda;
		int mPenalization;
		int mCycle;
		int mFlags;
	};

	struct gpuBlockMatching_data
	{
		int mPreset;
		int mDisparities;
		int mWindowSize;
	};

	struct beliefPropagation_data
	{
		int mDisparities;
		int mIterations;
		int mLevels;
		int mNrPlane;
		float mMaxDataTerm;
		float mDataWeight;
		float mMaxDiscTerm;
		float mDiscSingleJump;
		int mMinDispTh;
		int mMsgType;
	};

	StereoMode mMode;
	struct cameraConstants
	{
		camData *mCam1;
		camData *mCam2;
		Mat R;
		Mat T;
		Mat Q;
	};

	union algorithmData
	{
		blockMaching_data blockMatching;
		semiGlobalBM_data semiGlobalBM;
		var_data var;
		gpuBlockMatching_data gpuBlockMatching;
		beliefPropagation_data beliefPropagation;
	};

	cameraConstants camera;
	algorithmData mAlgorithmData;

public:
	stereoModeData();
};

class StereoCam
{
private:
	enum SC_MODE
	{
		invalidMode = -1, normalCam = 0, staticImg
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
		T mStereoDevice;

		StereoContainer()
		{
			mStereoDevice = NULL;
		}

	public:
		StereoContainer(stereoModeData &data);

		~StereoContainer()
		{
		}

		void operator ()(const Mat &left, const Mat &right, Mat &disparity, int disptype);
	};

	class StereoFactory
	{
	private:
		StereoFactory()
		{
		};
	public:
		static iStereoDevice* createStereoDevice(stereoModeData &data);
		static void saveOutput(stereoModeData &data, Mat &img);
	};

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
	StereoCam(stereoModeData &data, std::string window1, std::string window2, std::string disparityWindow, Mat &left, Mat &right, SHIFT_CAM mode = NONE);
	~StereoCam();

	void process(bool skipRemap);

	const Mat& getR();
	const Mat& getT();
	const Mat& getQ();
};

#endif /* STEREOCAM_H_ */

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

using namespace cv;

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
	camData(VideoCapture *cap) :
			mCap(cap)
	{
	}
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
	camData *mCam1;
	camData *mCam2;

	Mat mLeft;
	Mat mRight;

	std::string mWindow1;
	std::string mWindow2;
	std::string mDisparityWindow;

	Mat mR, mT, mQ;

	StereoBM mStereoCam;

	SC_MODE mMode;
	SHIFT_CAM mShiftMode;

	StereoCam();
public:
	StereoCam(int disparitiesCnt, int sadWindowSize, std::string window1, std::string window2,
				std::string disparityWindow, camData *cam1, camData *cam2, Mat R, Mat T, Mat Q,
				SHIFT_CAM mode = NONE);
	StereoCam(int disparitiesCnt, int sadWindowSize, std::string window1, std::string window2,
				std::string disparityWindow, Mat &left, Mat &right);

	void process(bool skipRemap);

	const Mat& getR();
	const Mat& getT();
	const Mat& getQ();
};

#endif /* STEREOCAM_H_ */

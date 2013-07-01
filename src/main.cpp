/*
 * main.cpp
 *
 *  Created on: 08-06-2013
 *      Author: Krzysztof Pilch
 */

#include <iostream>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

static void calcBoardCornerPositions(Size boardSize, float squareSize,
		vector<Point3f>& corners);

struct camData
{
	//camera source
	VideoCapture &mCap;
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
	camData(VideoCapture &cap) :
			mCap(cap)
	{
	}
};

struct camPoints
{
	vector<vector<Point2f> > mPoints;
};

class StereoCam
{
public:
	enum SC_MODE
	{
		invalidMode = -1, normalCam = 0, shiftedCam, staticImg
	};

private:
	camData *mCam1;
	camData *mCam2;

	Mat mLeft;
	Mat mRight;

	std::string mWindow1;
	std::string mWindow2;
	std::string mDisparityWindow;

	StereoBM mStereoCam;

	SC_MODE mMode;

	StereoCam();
public:
	StereoCam(int disparitiesCnt, int sadWindowSize, std::string window1,
			std::string window2, std::string disparityWindow, camData *cam1,
			camData *cam2, SC_MODE mode = normalCam);
	StereoCam(int disparitiesCnt, int sadWindowSize, std::string window1,
			std::string window2, std::string disparityWindow, Mat &left,
			Mat &right);

	void process();
};

unsigned aquireCalibData(camData &cam1, camData &cam2, camPoints &points1,
		camPoints &points2, unsigned sampleNums = 25,
		Size boardSize = Size(9, 6), int delay = 100);

bool calibrateCameras(camData &cam1, camData &cam2, camPoints &points1,
		camPoints &points2, Size boardSize, float squareSize);

void runInfo()
{
	std::cout
			<< "Computes disparity matrix for either live cam feed or static images.\nFor live cam feed program may perform calibration. To skip calibration press [esc].\nTo calculate disparity matrix for static images run program with parameter \"static\" and put two images \"tsukuba_l.png\" and \"tsukuba_r.png\" images in root directory of the program.\n\nTo close the application press [esc].\n\n";
}

int main(int argc, char **argv)
{
	runInfo();
	bool staticImages = false;
	if (argc > 1)
	{
		for (int i = 0; i < argc; i++)
		{
			std::string s(argv[i]);
			if (s == "static")
			{
				staticImages = true;
				break;
			}
		}
	}

	double dm1[3][3] =
	{
	{ 7.6756952371690159e+002, 0., 3.1950000000000000e+002 },
	{ 0., 7.6756952371690159e+002, 2.3950000000000000e+002 },
	{ 0., 0., 1. } };
	double dm2[3][3] =
	{
	{ 6.7745616052311675e+002, 0., 3.1950000000000000e+002 },
	{ 0, 6.7745616052311675e+002, 2.3950000000000000e+002 },
	{ 0., 0., 1. } };

	double dist1[] =
	{ -1.3884904706805279e-001, 1.2125564850471653e+000, 0., 0.,
			-5.0642441596711691e+000 };
	double dist2[] =
	{ 1.8715030647753067e-001, -1.3071922928154918e+000, 0., 0.,
			1.9894553723712307e+000 };

	double transl[] =
	{ 15.5, 0, 0.5 };

	VideoCapture capture1(0), capture2(1);
	camData cam1(capture1), cam2(capture2);
	camPoints points1, points2;
	int sampleNums = 25;
	Size boardSize = Size(9, 6);
	int delay = 1000;
	float squareSize = 25.5;

	cam1.mCamMat = Mat(3, 3, CV_64FC1, dm1);
	cam1.mDistMat = Mat(1, 5, CV_64FC1, dist1);
	cam2.mCamMat = Mat(3, 3, CV_64FC1, dm2);
	cam2.mDistMat = Mat(1, 5, CV_64FC1, dist2);

//	Mat emptyR = Mat::eye(3, 3, CV_64FC1);
//	Mat translationT(1, 3, CV_64FC1, transl);

	cvNamedWindow("Cam1", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Cam2", CV_WINDOW_AUTOSIZE);
//	cvNamedWindow("Cam1Orig", CV_WINDOW_AUTOSIZE);
//	cvNamedWindow("Cam2Orig", CV_WINDOW_AUTOSIZE);
	Mat frame1, frame2, oFrame1, oFrame2;
	capture1 >> frame1;
	capture2 >> frame2;

	Size newSize(2 * frame1.cols, 2 * frame1.rows), oldSize(frame2.cols,
			frame2.rows);

	if (!staticImages)
		if (aquireCalibData(cam1, cam2, points1, points2, sampleNums, boardSize,
				delay) >= 2)
		{
			if (!calibrateCameras(cam1, cam2, points1, points2, boardSize,
					squareSize))
			{
				std::cerr << "Error calibrating cameras!";
				return 2;
			}
		}
		else
			std::cerr << "Acquired to few samples for calibration, skipping!\n";

// --- stereo ze zdjêc ---
	Mat left = imread("tsukuba_l.png", CV_LOAD_IMAGE_GRAYSCALE), right = imread(
			"tsukuba_r.png", CV_LOAD_IMAGE_GRAYSCALE);
// --- koniec sterea ze zdjec ---

	int disparitiesCnt = 16;
	int sadWindowSize = 21;
	StereoCam *scCam;

	if (staticImages)
		scCam = new StereoCam(disparitiesCnt, sadWindowSize, "Cam1", "Cam2",
				"StereoCam", left, right);
	else
		scCam = new StereoCam(disparitiesCnt, sadWindowSize, "Cam1", "Cam2",
				"StereoCam", &cam1, &cam2);

	cvNamedWindow("StereoCam", CV_WINDOW_AUTOSIZE);

	Mat gray1(oldSize, CV_8UC1), gray2(oldSize, CV_8UC1);

// Live display
	while (1)
	{
		scCam->process();

		char c = cvWaitKey(33);
		if (c == 27)
			break;
	}
	delete scCam;
	capture1.release();
	capture2.release();
	cvDestroyWindow("Cam1");
	cvDestroyWindow("Cam2");
	cvDestroyWindow("StereoCam");
//	cvDestroyWindow("Cam1Orig");
//	cvDestroyWindow("Cam2Orig");

	return 0;
}

static void calcBoardCornerPositions(Size boardSize, float squareSize,
		vector<Point3f>& corners)
{
	corners.clear();

	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			corners.push_back(
					Point3f(float(j * squareSize), float(i * squareSize), 0));
}

unsigned aquireCalibData(camData &cam1, camData &cam2, camPoints &points1,
		camPoints &points2, unsigned sampleNums, Size boardSize, int delay)
{
	Mat frame1, frame2;
	cam1.mCap >> frame1;
	cam2.mCap >> frame2;

	cam1.mSize = frame1.size();
	cam2.mSize = frame2.size();

	if (cam1.mSize != cam2.mSize)
	{
		std::cerr << "Input cameras has different resolutions!\n";
		return 0;
	}
	Size oldSize = cam1.mSize;

	Mat cam1NewMat = getOptimalNewCameraMatrix(cam1.mCamMat, cam1.mDistMat,
			oldSize, 1.), cam2NewMat = getOptimalNewCameraMatrix(cam2.mCamMat,
			cam2.mDistMat, oldSize, 1.);

	cam1.mValidRoi = Rect(0, 0, frame1.cols, frame1.rows);
	cam2.mValidRoi = Rect(0, 0, frame1.cols, frame1.rows);

//	stereoRectify(cam1mat, diff1mat, cam2mat, diff2mat, oldSize, emptyR, translationT, R1, R2, P1, P2, Q);

	initUndistortRectifyMap(cam1.mCamMat, cam1.mDistMat, cam1.mR, cam1NewMat,
			oldSize, CV_32FC1, cam1.mMap1, cam1.mMap2);
	initUndistortRectifyMap(cam2.mCamMat, cam2.mDistMat, cam2.mR, cam2NewMat,
			oldSize, CV_32FC1, cam2.mMap1, cam2.mMap2);

	// Variables for Stereo Calibration
	const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
	clock_t prevTimestamp = 0;

	Mat oFrame1, oFrame2;

	// Acquisition of data for Stereo Calibration
	while (points1.mPoints.size() < sampleNums)
	{
		cam1.mCap >> frame1;
		cam2.mCap >> frame2;

		vector<Point2f> buff1, buff2;
		bool found1 = false, found2 = false;

//// wykomentowac gdy remap'y zostan¹ odkomentowane:
		oFrame1 = frame1;
		oFrame2 = frame2;
//		remap(view1, oFrame1, map11, map12, INTER_LANCZOS4);
//		remap(view2, oFrame2, map21, map22, INTER_LANCZOS4);

		found1 = findChessboardCorners(oFrame1, boardSize, buff1,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK
						| CV_CALIB_CB_NORMALIZE_IMAGE);
		found2 = findChessboardCorners(oFrame2, boardSize, buff2,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK
						| CV_CALIB_CB_NORMALIZE_IMAGE);

		if (found1 && found2)
		{
			Mat viewGray1, viewGray2;
			cvtColor(oFrame1, viewGray1, CV_BGR2GRAY);
			cvtColor(oFrame2, viewGray2, CV_BGR2GRAY);
			cornerSubPix(viewGray1, buff1, Size(11, 11), Size(-1, -1),
					TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			cornerSubPix(viewGray2, buff2, Size(11, 11), Size(-1, -1),
					TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			if (clock() - prevTimestamp > delay * 0.001 * CLOCKS_PER_SEC )
			{
				points1.mPoints.push_back(buff1);
				points2.mPoints.push_back(buff2);
				prevTimestamp = clock();
			}

			drawChessboardCorners(oFrame1, boardSize, Mat(buff1), found1);
			drawChessboardCorners(oFrame2, boardSize, Mat(buff2), found2);
		}

//		undistort(view1, oFrame1, cam1mat, dist1mat);
//		undistort(view2, oFrame2, cam2mat, dist2mat);

		string msg = format("%d/%d", (int) points1.mPoints.size(), sampleNums);
		int baseLine = 0;
		Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
		Point textOrigin(oFrame1.cols - 2 * textSize.width - 10,
				oFrame1.rows - 2 * baseLine - 10);
		putText(oFrame1, msg, textOrigin, 1, 1, RED);

		imshow("Cam1", oFrame1);
		imshow("Cam2", oFrame2);

		char c = cvWaitKey(33);
		if (c == 27)
			break;
	}

	return points1.mPoints.size();
}

bool calibrateCameras(camData &cam1, camData &cam2, camPoints &points1,
		camPoints &points2, Size boardSize, float squareSize)
{
	if (points1.mPoints.size() == points2.mPoints.size())
	{
// Stereo Calibration
		vector<vector<Point3f> > objectPoints(1);
		calcBoardCornerPositions(boardSize, squareSize, objectPoints[0]);
		objectPoints.resize(points1.mPoints.size(), objectPoints[0]);

		if (cam1.mSize != cam2.mSize)
		{
			std::cerr << "Iput cameras has different resolution!\n";
			return false;
		}
		Size oldSize = cam1.mSize;
		Mat R, T, E, F, Q;

		double projError = stereoCalibrate(objectPoints, points1.mPoints,
				points2.mPoints, cam1.mCamMat, cam1.mDistMat, cam2.mCamMat,
				cam2.mDistMat, oldSize, R, T, E, F,
				TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 1e-5),
				CV_CALIB_FIX_ASPECT_RATIO + CV_CALIB_ZERO_TANGENT_DIST
						+ CV_CALIB_RATIONAL_MODEL + CV_CALIB_FIX_K3
						+ CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);

		cout << "\nCalibrated stereo camera with projection error at: "
				<< projError << endl;

		stereoRectify(cam1.mCamMat, cam1.mDistMat, cam2.mCamMat, cam2.mDistMat,
				oldSize, R, T, cam1.mR, cam2.mR, cam1.mP, cam2.mP, Q,
				CV_CALIB_ZERO_DISPARITY, 1, oldSize, &cam1.mValidRoi,
				&cam2.mValidRoi);

		initUndistortRectifyMap(cam1.mCamMat, cam1.mDistMat, cam1.mR, cam1.mP,
				oldSize, CV_32FC1, cam1.mMap1, cam1.mMap2);
		initUndistortRectifyMap(cam2.mCamMat, cam2.mDistMat, cam2.mR, cam2.mP,
				oldSize, CV_32FC1, cam2.mMap1, cam2.mMap2);
	}
	else
	{
		std::cerr << "Point buffers of different sizes! Calibration aborted";
		return false;
	}
	return true;
}

StereoCam::StereoCam()
{
	mMode = invalidMode;
	mCam1 = NULL;
	mCam2 = NULL;
}

//camData *mCam1;
//camData *mCam2;
//
//Mat mLeft;
//Mat mRight;
//
//std::string mWindow1;
//std::string mWindow2;
//std::string mDisparityWindow;
//
//StereoBM mStereoCam;
//
//SC_MODE mMode;

StereoCam::StereoCam(int disparitiesCnt, int sadWindowSize, std::string window1,
		std::string window2, std::string disparityWindow, camData *cam1,
		camData *cam2, SC_MODE mode) :
		mCam1(cam1), mCam2(cam2), mWindow1(window1), mWindow2(window2), mDisparityWindow(
				disparityWindow), mStereoCam(StereoBM::BASIC_PRESET,
				disparitiesCnt, sadWindowSize), mMode(mode)
{
}

StereoCam::StereoCam(int disparitiesCnt, int sadWindowSize, std::string window1,
		std::string window2, std::string disparityWindow, Mat &left, Mat &right) :
		mCam1(NULL), mCam2(NULL), mWindow1(window1), mWindow2(window2), mDisparityWindow(
				disparityWindow), mStereoCam(StereoBM::BASIC_PRESET,
				disparitiesCnt, sadWindowSize), mMode(StereoCam::staticImg)
{
	left.copyTo(mLeft);
	right.copyTo(mRight);
//	cvtColor(left, mLeft, CV_RGB2GRAY);
//	cvtColor(right, mRight, CV_RGB2GRAY);
}

void StereoCam::process()
{
	Mat frame1, frame2, oFrame1, oFrame2, gray1, gray2, dispFrame;

	//only for shifted fragment
	int offset = 50;
	Rect initialFrame(150, 150, 300, 300);
	Rect finalFrame(initialFrame.x + offset, initialFrame.y, initialFrame.width,
			initialFrame.height);
	Mat push, cleared, pushed;
	//end shifted variables

	switch (mMode)
	{
	case normalCam:
		mCam1->mCap >> frame1;
		mCam2->mCap >> frame2;

		if (frame1.empty() || frame2.empty())
			return;

		remap(frame1, oFrame1, mCam1->mMap1, mCam1->mMap2, INTER_LANCZOS4);
		remap(frame2, oFrame2, mCam2->mMap1, mCam2->mMap2, INTER_LANCZOS4);

		cvtColor(oFrame1, gray1, CV_RGB2GRAY);
		cvtColor(oFrame2, gray2, CV_RGB2GRAY);

		break;
	case shiftedCam:
		mCam1->mCap >> frame1;
		mCam2->mCap >> frame2;

		if (frame1.empty() || frame2.empty())
			return;

		remap(frame1, oFrame1, mCam1->mMap1, mCam1->mMap2, INTER_LANCZOS4);
		remap(frame2, oFrame2, mCam2->mMap1, mCam2->mMap2, INTER_LANCZOS4);

		cvtColor(oFrame1, gray1, CV_RGB2GRAY);
		cvtColor(oFrame2, gray2, CV_RGB2GRAY);

		push = Mat(gray1, initialFrame);
		cleared = Mat(gray2, initialFrame);
		pushed = Mat(gray2, finalFrame);

		gray1.copyTo(gray2);

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

	imshow(mWindow1, gray1);
	imshow(mWindow2, gray2);
	imshow(mDisparityWindow, dispFrame);
}


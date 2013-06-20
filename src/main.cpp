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

int main(int argc, char **argv)
{
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

	Mat cam1mat(3, 3, CV_64FC1, dm1), cam2mat(3, 3, CV_64FC1, dm2);
	Mat dist1mat(1, 5, CV_64FC1, dist1), dist2mat(1, 5, CV_64FC1, dist2);
	Mat emptyR = Mat::eye(3, 3, CV_64FC1);
	Mat translationT(1, 3, CV_64FC1, transl);

	cvNamedWindow("Cam1", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Cam2", CV_WINDOW_AUTOSIZE);
//	cvNamedWindow("Cam1Orig", CV_WINDOW_AUTOSIZE);
//	cvNamedWindow("Cam2Orig", CV_WINDOW_AUTOSIZE);
	VideoCapture capture1(0), capture2(1);
	Mat frame1, frame2, oFrame1, oFrame2;
	capture1 >> frame1;
	capture2 >> frame2;

	Size newSize(2 * frame1.cols, 2 * frame1.rows), oldSize(frame2.cols,
			frame2.rows);

	Mat cam1NewMat = getOptimalNewCameraMatrix(cam1mat, dist1mat, oldSize, 1.),
			cam2NewMat = getOptimalNewCameraMatrix(cam2mat, dist2mat, oldSize,
					1.);

	Mat map11, map12, map21, map22;
	Mat R1, R2, P1, P2, R, T, E, F, Q;
	Rect validRoi1, validRoi2;

//	stereoRectify(cam1mat, diff1mat, cam2mat, diff2mat, oldSize, emptyR, translationT, R1, R2, P1, P2, Q);

	initUndistortRectifyMap(cam1mat, dist1mat, R1, cam1NewMat, oldSize,
			CV_32FC1, map11, map12);
	initUndistortRectifyMap(cam2mat, dist2mat, R2, cam1NewMat, oldSize,
			CV_32FC1, map21, map22);

	// Variables for Sterei Calibration
	vector<vector<Point2f> > imagePoints1, imagePoints2;
	int sampleNums = 25;
	const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
	clock_t prevTimestamp = 0;
	int delay = 1000;
	Size boardSize(9, 6);
	float squareSize = 25.5;

	// Acquisition of data for Stereo Calibration
	while (imagePoints1.size() < sampleNums)
	{
		Mat view1, view2;
		capture1 >> view1;
		capture2 >> view2;

		vector<Point2f> buff1, buff2;
		bool found1 = false, found2 = false;

		remap(view1, oFrame1, map11, map12, INTER_LANCZOS4);
		remap(view2, oFrame2, map21, map22, INTER_LANCZOS4);

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
				imagePoints1.push_back(buff1);
				imagePoints2.push_back(buff2);
				prevTimestamp = clock();
			}

			drawChessboardCorners(oFrame1, boardSize, Mat(buff1), found1);
			drawChessboardCorners(oFrame2, boardSize, Mat(buff2), found2);
		}

//		undistort(view1, oFrame1, cam1mat, dist1mat);
//		undistort(view2, oFrame2, cam2mat, dist2mat);

		string msg = format("%d/%d", (int) imagePoints1.size(), sampleNums);
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

	if (imagePoints1.size() == sampleNums)
	{
		// Stereo Calibration
		vector<vector<Point3f> > objectPoints(1);
		calcBoardCornerPositions(boardSize, squareSize, objectPoints[0]);
		objectPoints.resize(imagePoints1.size(), objectPoints[0]);

		double projError = stereoCalibrate(objectPoints, imagePoints1,
				imagePoints2, cam1mat, dist1mat, cam2mat, dist2mat, oldSize, R,
				T, E, F,
				TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 1e-5),
				CV_CALIB_FIX_ASPECT_RATIO + CV_CALIB_ZERO_TANGENT_DIST
						+ CV_CALIB_RATIONAL_MODEL + CV_CALIB_FIX_K3
						+ CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);

		cout << "\nCalibrated stereo camera with projection error at: "
				<< projError << endl;

		stereoRectify(cam1mat, dist1mat, cam2mat, dist2mat, oldSize, R, T, R1,
				R2, P1, P2, Q, CV_CALIB_ZERO_DISPARITY, 1, oldSize, &validRoi1,
				&validRoi2);

		initUndistortRectifyMap(cam1mat, dist1mat, R1, P1, oldSize, CV_32FC1,
				map11, map12);
		initUndistortRectifyMap(cam2mat, dist2mat, R2, P2, oldSize, CV_32FC1,
				map21, map22);
	}

	int disparitiesCnt = 16;
	int sadWindowSize = 11;
	StereoBM stereoCam(StereoBM::BASIC_PRESET, disparitiesCnt, sadWindowSize);

// --- stereo ze zdjêc ---
	Mat left = imread("tsukuba_l.png", CV_LOAD_IMAGE_GRAYSCALE), right = imread(
			"tsukuba_r.png", CV_LOAD_IMAGE_GRAYSCALE);
// --- koniec sterea ze zdjec ---

	Mat dispFrame;
	cvNamedWindow("StereoCam", CV_WINDOW_AUTOSIZE);

	Mat gray1(oldSize, CV_8UC1), gray2(oldSize, CV_8UC1);

// Live display
	while (1)
	{
		capture1 >> frame1;
		capture2 >> frame2;
		if (frame1.empty() || frame2.empty())
			break;

		remap(frame1, oFrame1, map11, map12, INTER_LANCZOS4);
		remap(frame2, oFrame2, map21, map22, INTER_LANCZOS4);

		cvtColor(oFrame1, gray1, CV_RGB2GRAY);
		cvtColor(oFrame2, gray2, CV_RGB2GRAY);

//		stereoCam(left, right, dispFrame, CV_32F);
		stereoCam(gray1, gray2, dispFrame, CV_32F);

		imshow("Cam1", oFrame1);
		imshow("Cam2", oFrame2);
//		imshow("Cam1", left);
//		imshow("Cam2", right);
		imshow("StereoCam", dispFrame);
//		imshow("Cam1Orig", frame1);
//		imshow("Cam2Orig", frame2);
		char c = cvWaitKey(33);
		if (c == 27)
			break;
	}
	capture1.release();
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


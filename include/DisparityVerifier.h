/*
 * DisparityVerifier.h
 *
 *  Created on: 25-10-2014
 *      Author: Krzysztof Pilch
 */

#ifndef DISPARITY_VERIFIER_H_
#define DISPARITY_VERIFIER_H_

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class DisparityVerifier
{
private:
	Mat mGroundTruth;
	Mat mResult;
	Mat mDiff;

	unsigned long mAbs;
	unsigned long mSquare;

public:
	DisparityVerifier(const Mat& truth);
	DisparityVerifier();
	~DisparityVerifier();

	void setGroundTruth(const Mat& truth);
	const Mat& getGroundTruth();

	void setResult(const Mat& result);
	const Mat& getResult();
	
	const Mat& diff(const Mat& result);
	const Mat& diff();

	unsigned long getAbs();
	unsigned long getSquare();

	void save(string fileName);
};

#endif //DISPARITY_VERIFIER_H_

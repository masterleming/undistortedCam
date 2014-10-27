#include "DisparityVerifier.h"
#include <fstream>

DisparityVerifier::DisparityVerifier(const Mat& truth)
	:mGroundTruth(truth), mResult(), mDiff(), mAbs(0), mSquare(0)
{
}

DisparityVerifier::DisparityVerifier()
	:mGroundTruth(), mResult(), mDiff(), mAbs(0), mSquare(0)
{
}

DisparityVerifier::~DisparityVerifier()
{
}

void DisparityVerifier::setGroundTruth(const Mat& truth)
{
	mGroundTruth = truth;
}

const Mat& DisparityVerifier::getGroundTruth()
{
	return mGroundTruth;
}

void DisparityVerifier::setResult(const Mat& result)
{
	mResult = result;
}

const Mat& DisparityVerifier::getResult()
{
	return mResult;
}
	
const Mat& DisparityVerifier::diff(const Mat& result)
{
	mResult = result;
	return diff();
}

const Mat& DisparityVerifier::diff()
{
	cout << "mResult\n\ttype: " << mResult.type() << "\tempty? " << (mResult.empty() ? "YES" : "NO") << endl;
	cout << "mGroundTruth\n\ttype: " << mGroundTruth.type() << "\tempty? " << (mGroundTruth.empty() ? "YES" : "NO") << endl;
//	mDiff = mResult - mGroundTruth;
	mDiff = Mat::zeros(mResult.rows, mResult.cols, 0);
	mAbs = 0;
	mSquare = 0;
	for(int i = 0; i < mResult.rows; i++)
		for(int j = 0; j < mResult.cols; j++)
		{
			unsigned long diff = abs(mResult.at<unsigned char>(i, j) - mGroundTruth.at<unsigned char>(i, j));
			mDiff.at<unsigned char>(i, j) = diff;

			mAbs += diff;
			mSquare += diff * diff;
		}
	return mDiff;
}

unsigned long DisparityVerifier::getAbs()
{
	return mAbs;
}

unsigned long DisparityVerifier::getSquare()
{
	return mSquare;
}

void DisparityVerifier::save(string fileName)
{
	imwrite(fileName + "-diff.png", mDiff);
	FileStorage fs(fileName + "-diff.yml", FileStorage::WRITE);
	fs << "mAbs" << (double) mAbs << "mSquare" << (double) mSquare;
	fs.release();

	ofstream file("output/index.csv", ios::app);
	if(file.good())
		file << fileName << ',' << mAbs << ',' << mSquare << endl;
	else
		cerr << "Can not open output/index.csv!";

	file.close();
}

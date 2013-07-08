#include "StereoCam.h"
#include <boost/program_options.hpp>

using namespace std;
namespace po = boost::program_options;

enum optionReadStatus
{
	SUCCESS = 0, SUCCESS_CLOSE,

	FAILURE
};

struct camPoints
{
	vector<vector<Point2f> > mPoints;
};

optionReadStatus programOptions(po::variables_map &vm, po::options_description* desc, int argc, char **argv)
{
	desc = new po::options_description("Supported options");
	desc->add_options()("help,h", "Prints this help message.")("left,l", po::value<string>(),
			"Sets left image for static image stereo-vision.")("right,r", po::value<string>(),
			"Sets right image for static image stereo-vision.")("static,s", "Enables static image stereo-vision.")(
			"shifted", po::value<string>(),
			"Applies algorithm to two copies of the same image, one of which has a rectangular piece of image shifted in an arbitrary direction. Pass \"left\" to copy left picture or \"right\" to copy right picture")(
			"skip", "Skips camera calibration.")("intrinsics,i", po::value<string>(),
			"Sets file containing intrinsics calibration parameters.")("extrinsics,e", po::value<string>(),
			"Sets file containing extrinsics calibration parameters.")("swap-cameras,S",
			"Swaps cameras input so there is no need to move already positioned cameras.")("disparities,d",
			po::value<int>()->default_value(0),
			"How many disparities shall be searched for. Allowed values are 0 and any integer multiplication of 16.")(
			"sad-window-size,w", po::value<int>()->default_value(21),
			"Size of the square to search for matching features. Must be an ODD number.")("calibration-samples,c",
			po::value<int>()->default_value(25), "How many pictures shall be acquired for calibration.")("delay,D",
			po::value<int>()->default_value(1000), "Delay between samples measured in milliseconds.")("square-size",
			po::value<float>()->default_value(25.5),
			"Distance between square corners. May be any arbitrary value or physical length of the square edge in millimeters or inches or pixels.")(
			"board-width,W", po::value<int>()->default_value(9), "Width of the calibrating pattern.")("board-height,H",
			po::value<int>()->default_value(6), "Height of the calibrating pattern.");

	po::store(po::parse_command_line(argc, argv, *desc), vm);
	po::notify(vm);

	if (vm.count("help"))
	{
		cout << *desc << endl;
		return SUCCESS_CLOSE;
	}

	if (vm.count("static"))
	{
		if (!(vm.count("left") && vm.count("right")))
		{
			cerr
					<< "Too few arguments passed to start static image stereo-vision. It requires both left and right images to be specified!";
			cout << *desc << endl;
			return FAILURE;
		}
	}
	return SUCCESS;
}

void getRuntimeConfiguration(const po::variables_map& varMap, bool& staticImages, string& leftImg, string& rightImg,
								bool& skipCalib, int& disparitiesCnt, int& sadWindowSize, int& sampleNums, int& delay,
								float& squareSize, Size& boardSize, bool& swapCameras, string& intrParamsFile,
								string& extrParamsFile, StereoCam::SHIFT_CAM &dir)
{
	if (varMap.count("static"))
	{
		staticImages = true;
		leftImg = varMap["left"].as<string>();
		rightImg = varMap["right"].as<string>();
	}
	if (varMap.count("skip"))
		skipCalib = true;

	if (varMap.count("disparities"))
		disparitiesCnt = varMap["disparities"].as<int>();

	if (varMap.count("sad-window-size"))
		sadWindowSize = varMap["sad-window-size"].as<int>();

	if (varMap.count("calibration-samples"))
		sampleNums = varMap["calibration-samples"].as<int>();

	if (varMap.count("delay"))
		delay = varMap["delay"].as<int>();

	if (varMap.count("square-size"))
		squareSize = varMap["square-size"].as<float>();

	if (varMap.count("board-width") && varMap.count("board-height"))
	{
		int boardW = varMap["board-width"].as<int>(), boardH = varMap["board-height"].as<int>();
		boardSize = Size(boardW, boardH);
	}
	if (varMap.count("swap-cameras"))
		swapCameras = true;

	if (varMap.count("intrinsics"))
		intrParamsFile = varMap["intrinsics"].as<string>();

	if (varMap.count("extrinsics"))
	{
		extrParamsFile = varMap["extrinsics"].as<string>();
		skipCalib = true;
	}

	if (varMap.count("shifted"))
	{
		string sDir = varMap["shifted"].as<string>();
		if (sDir == "left")
			dir = StereoCam::LEFT;
		else if (sDir == "right")
			dir = StereoCam::RIGHT;
		else
			dir = StereoCam::NONE;
	}
}

void initIntrinsicsCamParams(bool swapCameras, camData &cam1, camData &cam2, const string& intrParamsFile,
								VideoCapture& capture1, VideoCapture& capture2)
{
	if (swapCameras)
	{
		cam1.mCap = &capture2;
		cam2.mCap = &capture1;
	}
	else
	{
		cam1.mCap = &capture1;
		cam2.mCap = &capture2;
	}
	FileStorage intrStorage;
	if (intrParamsFile.size() > 0)
		intrStorage.open(intrParamsFile, FileStorage::READ);

	if (intrStorage.isOpened())
	{
		intrStorage["M1"] >> cam1.mCamMat;
		intrStorage["D1"] >> cam1.mDistMat;
		intrStorage["M2"] >> cam2.mCamMat;
		intrStorage["D2"] >> cam2.mDistMat;
		intrStorage.release();
	}
	else
	{
		cam1.mCamMat = Mat::eye(Size(3, 3), CV_64FC1);
		cam1.mDistMat = Mat::zeros(Size(1, 8), CV_64FC1);
		cam2.mCamMat = Mat::eye(Size(3, 3), CV_64FC1);
		cam2.mDistMat = Mat::zeros(Size(1, 8), CV_64FC1);
	}
}

void rectifyCameras(camData &cam1, camData &cam2, Mat& R, Mat &T, Mat &Q)
{
	Size oldSize = cam1.mSize;

//	stereoRectify(cam1.mCamMat, cam1.mDistMat, cam2.mCamMat, cam2.mDistMat, oldSize, R, T, cam1.mR, cam2.mR, cam1.mP,
//			cam2.mP, Q, CV_CALIB_ZERO_DISPARITY, 1, oldSize, &cam1.mValidRoi, &cam2.mValidRoi);

	initUndistortRectifyMap(cam1.mCamMat, cam1.mDistMat, cam1.mR, cam1.mP, oldSize, CV_32FC1, cam1.mMap1, cam1.mMap2);
	initUndistortRectifyMap(cam2.mCamMat, cam2.mDistMat, cam2.mR, cam2.mP, oldSize, CV_32FC1, cam2.mMap1, cam2.mMap2);
}

void initExtrinsicsCamParams(const string& extrParamsFile, bool& skipCalib, Mat& R, Mat& T, Mat& Q, camData& cam1,
								camData& cam2)
{
	FileStorage extrStorage;
	if (extrParamsFile.size() > 0)
	{
		extrStorage.open(extrParamsFile, FileStorage::READ);
		if (extrStorage.isOpened())
		{
			extrStorage["R"] >> R;
			extrStorage["T"] >> T;
			extrStorage["Q"] >> Q;
			extrStorage["R1"] >> cam1.mR;
			extrStorage["P1"] >> cam1.mP;
			extrStorage["R2"] >> cam2.mR;
			extrStorage["P2"] >> cam2.mP;
			extrStorage.release();
			skipCalib = false;
			rectifyCameras(cam1, cam2, R, T, Q);
		}
	}
}

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
{
	corners.clear();

	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			corners.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
}

unsigned aquireCalibData(camData &cam1, camData &cam2, camPoints &points1, camPoints &points2, unsigned sampleNums = 25,
							Size boardSize = Size(9, 6), int delay = 100)
{
	Mat frame1, frame2;
	*cam1.mCap >> frame1;
	*cam2.mCap >> frame2;

	cam1.mSize = frame1.size();
	cam2.mSize = frame2.size();

	if (cam1.mSize != cam2.mSize)
	{
		std::cerr << "Input cameras has different resolutions!\n";
		return 0;
	}
	Size oldSize = cam1.mSize;

	Mat cam1NewMat = getOptimalNewCameraMatrix(cam1.mCamMat, cam1.mDistMat, oldSize, 1.), cam2NewMat =
			getOptimalNewCameraMatrix(cam2.mCamMat, cam2.mDistMat, oldSize, 1.);

	cam1.mValidRoi = Rect(0, 0, frame1.cols, frame1.rows);
	cam2.mValidRoi = Rect(0, 0, frame1.cols, frame1.rows);

//	stereoRectify(cam1mat, diff1mat, cam2mat, diff2mat, oldSize, emptyR, translationT, R1, R2, P1, P2, Q);

	initUndistortRectifyMap(cam1.mCamMat, cam1.mDistMat, cam1.mR, cam1NewMat, oldSize, CV_32FC1, cam1.mMap1,
			cam1.mMap2);
	initUndistortRectifyMap(cam2.mCamMat, cam2.mDistMat, cam2.mR, cam2NewMat, oldSize, CV_32FC1, cam2.mMap1,
			cam2.mMap2);

// Variables for Stereo Calibration
	const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
	clock_t prevTimestamp = 0;

	Mat oFrame1, oFrame2;

// Acquisition of data for Stereo Calibration
	while (points1.mPoints.size() < sampleNums)
	{
		*cam1.mCap >> frame1;
		*cam2.mCap >> frame2;

		vector<Point2f> buff1, buff2;
		bool found1 = false, found2 = false;

//// wykomentowac gdy remap'y zostan¹ odkomentowane:
		oFrame1 = frame1;
		oFrame2 = frame2;
//		remap(view1, oFrame1, map11, map12, INTER_LANCZOS4);
//		remap(view2, oFrame2, map21, map22, INTER_LANCZOS4);

		found1 = findChessboardCorners(oFrame1, boardSize, buff1,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		found2 = findChessboardCorners(oFrame2, boardSize, buff2,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

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
		Point textOrigin(oFrame1.cols - 2 * textSize.width - 10, oFrame1.rows - 2 * baseLine - 10);
		putText(oFrame1, msg, textOrigin, 1, 1, RED);

		imshow("Cam1", oFrame1);
		imshow("Cam2", oFrame2);

		char c = cvWaitKey(33);
		if (c == 27)
			break;
	}

	return points1.mPoints.size();
}

bool calibrateCameras(camData &cam1, camData &cam2, camPoints &points1, camPoints &points2, Size boardSize,
						float squareSize, Mat &R, Mat &T, Mat &Q)
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
		Mat E, F;

		double projError = stereoCalibrate(objectPoints, points1.mPoints, points2.mPoints, cam1.mCamMat, cam1.mDistMat,
				cam2.mCamMat, cam2.mDistMat, oldSize, R, T, E, F,
				TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 1e-5),
				CV_CALIB_FIX_ASPECT_RATIO + CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_RATIONAL_MODEL + CV_CALIB_FIX_K3
						+ CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);

		cout << "\nCalibrated stereo camera with projection error at: " << projError << endl;

		rectifyCameras(cam1, cam2, R, T, Q);
	}
	else
	{
		std::cerr << "Point buffers of different sizes! Calibration aborted";
		return false;
	}
	return true;
}

void saveLastCameraParams(const camData& cam1, const camData& cam2, StereoCam* scCam)
{
	FileStorage fs("last_intrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cam1.mCamMat << "D1" << cam1.mDistMat << "M2" << cam2.mCamMat << "D2" << cam2.mDistMat;
		fs.release();
	}
	else
		cerr << "Writing last intrinsics parameters has failed!\n";

	fs.open("last_extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << scCam->getR() << "T" << scCam->getT() << "R1" << cam1.mR << "R2" << cam2.mR << "P1" << cam1.mP
				<< "P2" << cam2.mP << "Q" << scCam->getQ();
		fs.release();
	}
	else
		cerr << "Writing last extrinsics parameters has failed!\n";
}

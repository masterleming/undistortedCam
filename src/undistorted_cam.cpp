/*
 * undistorted_cam.cpp
 *
 *  Created on: 28 paź 2013
 *      Author: Krzysztof Pilch
 */

#include "undistorted_cam.h"
#include "StereoCam.h"

optionReadStatus programOptions(po::variables_map &vm, po::options_description *&conCfg, po::options_description *&fileCfg, int argc, char **argv)
{
	conCfg = new po::options_description("Command line options");
	conCfg->add_options()("mode,m", po::value<string>()->default_value("bm"), "Stereo algorithm to be used. Supported values are:"
					"\n bm  - block maching"
					"\n var - var"
					"\n bp  - belief propagation"
					"\n cs  - constant space")
			("help,h", "Prints this help message.")
			("left,l", po::value<string>(), "Sets left image for static image stereo-vision.")
			("right,r", po::value<string>(), "Sets right image for static image stereo-vision.")
			("static,s", "Enables static image stereo-vision.")
			("shifted", po::value<string>(),"Applies algorithm to two copies of the same image, one of which has a rectangular piece of image shifted in an arbitrary direction. Pass \"left\" to copy left picture or \"right\" to copy right picture")
			("skip", "Skips camera calibration.")
			("intrinsics,i", po::value<string>(), "Sets file containing intrinsics calibration parameters.")
			("extrinsics,e", po::value<string>(), "Sets file containing extrinsics calibration parameters.")
			("swap-cameras,S","Swaps cameras input so there is no need to move already positioned cameras.")
			("calibration-samples,c", po::value<int>()->default_value(25), "How many pictures shall be acquired for calibration.")
			("delay,D",po::value<int>()->default_value(1000), "Delay between samples measured in milliseconds.")
			("square-size",po::value<float>()->default_value(25.5),"Distance between square corners. May be any arbitrary value or physical length of the square edge in millimeters or inches or pixels.")
			("board-width,W", po::value<int>()->default_value(9), "Width of the calibrating pattern.")
			("board-height,H",po::value<int>()->default_value(6), "Height of the calibrating pattern.")
			;

	fileCfg = new po::options_description("Config file options");
	fileCfg->add_options()
			//common options
			("max_disp", po::value<int>(), "How many disparities shall be searched for. Allowed values are 0 and any integer multiplication of 16.")
			("min_disp", po::value<int>(), "Minimal number to be searched for (only some algorithms support this feature.")
			("levels", po::value<int>(), "The number of pyramid layers, including the initial image.")
			//GPU common options
			("n_disp", po::value<int>(), "Number of disparities.")
			("iterations", po::value<int>(), "Number of iterations of algorithm.")
			("msg_type", po::value<int>(), "Type of message returned by algorithm.")
			("max_data_term", po::value<float>(), "Truncation of data cost.")
			("data_weight", po::value<float>(), "Data weight.")
			("max_disc_term", po::value<float>(), "Truncation of discontinuity.")
			("disc_single_jump", po::value<float>(), "Discontinuity single jump.")
			//StereoBM options
			("sad-window-size,w",po::value<int>()->default_value(21), "Size of the square to search for matching features. Must be an ODD number.")
			//Var
			("pyr_scale", po::value<double>(), "Specifies the image scale (<1) to build the pyramids for each image.")
			("n_it", po::value<int>(), "The number of iterations the algorithm does at each pyramid level.")
			("poly_n", po::value<int>(), "Size of the pixel neighbourhood used to find polynomial expansion in each pixel.")
			("poly_sigma", po::value<double>(), "Standard deviation of the Gaussian that is used to smooth derivatives that are used as a basis for the polynomial expansion.")
			("fi", po::value<float>(), "The smoothness parameter, or the weight coefficient for the smoothness term.")
			("lambda", po::value<float>(), "The threshold parameter for edge-preserving smoothness.")
			("penalization", po::value<int>(), "Possible values: PENALIZATION_TICHONOV - linear smoothness; PENALIZATION_CHARBONNIER - non-linear edge preserving smoothness; PENALIZATION_PERONA_MALIK - non-linear edge-enhancing smoothness. (check StereoVar documentation for actual values!)")
			("cycle", po::value<int>(), "Type of the multigrid cycle. Possible values: CYCLE_O and CYCLE_V for null- and v-cycles respectively.")
			("flags",po::value<int>(), "The operation flags; can be a combination of the following:"
					"\nUSE_INITIAL_DISPARITY: Use the input flow as the initial flow approximation."
					"\nUSE_EQUALIZE_HIST: Use the histogram equalization in the pre-processing phase."
					"\nUSE_SMART_ID: Use the smart iteration distribution (SID)."
					"\nUSE_AUTO_PARAMS: Allow the method to initialize the main parameters."
					"\nUSE_MEDIAN_FILTERING: Use the median filer of the solution in the post processing phase."
					"\nPASS SUM OF DESIRED FLAGS. For actual values check StereoVar.")
			//belief propagation does not need specific options
			//constant space options
			("nr_plane", po::value<int>(),"Number of disparity levels on the first level.")
			;
	po::options_description fileOptions("File options");
	fileOptions.add(*conCfg).add(*fileCfg);

	string cfgFile = "config.cfg";
	po::store(po::parse_command_line(argc, argv, *conCfg), vm);
	po::store(po::parse_config_file<char>(cfgFile.c_str(), fileOptions, false), vm);
	po::notify(vm);

	if (vm.count("help"))
	{
		cout << *conCfg << endl;
		return SUCCESS_CLOSE;
	}

	return SUCCESS;
}

//void getRuntimeConfiguration(const po::variables_map& varMap, bool& staticImages, string& leftImg, string& rightImg, bool& skipCalib,
//		int& disparitiesCnt, int& sadWindowSize, int& sampleNums, int& delay, float& squareSize, Size& boardSize, bool& swapCameras,
//		string& intrParamsFile, string& extrParamsFile, StereoCam::SHIFT_CAM &dir)
//{
//	if (varMap.count("static"))
//	{
//		staticImages = true;
//		leftImg = varMap["left"].as<string>();
//		rightImg = varMap["right"].as<string>();
//	}
//	if (varMap.count("skip"))
//		skipCalib = true;
//
//	if (varMap.count("disparities"))
//		disparitiesCnt = varMap["disparities"].as<int>();
//
//	if (varMap.count("sad-window-size"))
//		sadWindowSize = varMap["sad-window-size"].as<int>();
//
//	if (varMap.count("calibration-samples"))
//		sampleNums = varMap["calibration-samples"].as<int>();
//
//	if (varMap.count("delay"))
//		delay = varMap["delay"].as<int>();
//
//	if (varMap.count("square-size"))
//		squareSize = varMap["square-size"].as<float>();
//
//	if (varMap.count("board-width") && varMap.count("board-height"))
//	{
//		int boardW = varMap["board-width"].as<int>(), boardH = varMap["board-height"].as<int>();
//		boardSize = Size(boardW, boardH);
//	}
//	if (varMap.count("swap-cameras"))
//		swapCameras = true;
//
//	if (varMap.count("intrinsics"))
//		intrParamsFile = varMap["intrinsics"].as<string>();
//
//	if (varMap.count("extrinsics"))
//	{
//		extrParamsFile = varMap["extrinsics"].as<string>();
//		skipCalib = true;
//	}
//
//	if (varMap.count("shifted"))
//	{
//		string sDir = varMap["shifted"].as<string>();
//		if (sDir == "left")
//			dir = StereoCam::LEFT;
//		else if (sDir == "right")
//			dir = StereoCam::RIGHT;
//		else
//			dir = StereoCam::NONE;
//	}
//}

bool getRuntimeConfiguration(const po::variables_map& varMap, calibrationCfg &calibCfg, programCfg &prgCfg, stereoModeData &camData)
{
	if (varMap.count("mode"))
	{
		prgCfg.mAlgorithm = varMap["mode"].as<string>();
		if (prgCfg.mAlgorithm != "bm" && prgCfg.mAlgorithm != "var" && prgCfg.mAlgorithm != "bp" && prgCfg.mAlgorithm != "cs")
		{
			cerr << "Invalid algorithm specified: " << prgCfg.mAlgorithm;
			return false;
		}
	}
	else
	{
		cerr << "No algorithm specified!";
		return false;
	}

	if (varMap.count("static"))
	{
		if (!(varMap.count("left") && varMap.count("right")))
		{
			cerr << "Too few arguments passed to start static image stereo-vision. It requires both left and right images to be specified!";
			return false;
		}
		prgCfg.mStatic = true;
		prgCfg.mLeft = varMap["left"].as<string>();
		prgCfg.mRight = varMap["right"].as<string>();
	}

	if (varMap.count("shifted"))
	{
		string sDir = varMap["shifted"].as<string>();
		if (sDir == "left")
			prgCfg.mShifted = StereoCam::LEFT;
		else if (sDir == "right")
			prgCfg.mShifted = StereoCam::RIGHT;
		else
		{
			cerr << "Specified wrong shift direction: " << sDir << "!";
			return false;
		}
	}

	if (varMap.count("intrinsics"))
		prgCfg.mIntrinsics = varMap["intrinsics"].as<string>();

	if (varMap.count("extrinsics"))
		prgCfg.mExtrinsics = varMap["extrinsics"].as<string>();

	if (varMap.count("swap-cameras"))
		prgCfg.mSwapCameras = true;
	//////////////////////////////////////////////////////////////////
	if (varMap.count("skip"))
		calibCfg.mSkip = true;

	if (varMap.count("calibration-samples"))
		calibCfg.mCalibSamples = varMap["calibration-samples"].as<int>();

	if (varMap.count("delay"))
		calibCfg.mDelay = varMap["delay"].as<int>();

	if (varMap.count("square-size"))
		calibCfg.mSquareSize = varMap["square-size"].as<float>();

	if (varMap.count("board-width"))
		calibCfg.mBoardWidth = varMap["board-width"].as<int>();
	else if (!calibCfg.mSkip)
	{
		cerr << "board-width MUST be specified to use calibration!";
		return false;
	}

	if (varMap.count("board-height"))
		calibCfg.mBoardHeight = varMap["board-height"].as<int>();
	else if (!calibCfg.mSkip)
	{
		cerr << "board-height MUST be specified to use calibration!";
		return false;
	}

	calibCfg.mBoardSize = Size(calibCfg.mBoardWidth, calibCfg.mBoardHeight);
	/////////////////////////////////////////////////////////////////
	if (varMap.count("max_disp"))
		camData.common.mMaxDisp = varMap["max_disp"].as<int>();

	if (varMap.count("min_disp"))
		camData.common.mMinDisp = varMap["min_disp"].as<int>();

	if (varMap.count("levels"))
		camData.common.mLevels = varMap["levels"].as<int>();
	/////////////////////////////////////////////////////////////////
	if (varMap.count("n_disp"))
		camData.gpuCommon.mNdisp = varMap["n_disp"].as<int>();

	if (varMap.count("iterations"))
		camData.gpuCommon.mIters = varMap["iterations"].as<int>();

	if (varMap.count("msg_type"))
		camData.gpuCommon.mMsgType = varMap["msg_type"].as<int>();

	if (varMap.count("max_data_term"))
		camData.gpuCommon.mMaxDataTerm = varMap["max_data_term"].as<float>();

	if (varMap.count("data_weight"))
		camData.gpuCommon.mDataWeight = varMap["data_weight"].as<float>();

	if (varMap.count("max_disc_term"))
		camData.gpuCommon.mMaxDiscTerm = varMap["max_disc_term"].as<float>();

	if (varMap.count("disc_single_jump"))
		camData.gpuCommon.mDiscSingleJump = varMap["disc_single_jump"].as<float>();
	///////////////////////////////////////////////////////////////////
	if (varMap.count("sad-window-size"))
		camData.blockMaching.mSadWindowSize = varMap["sad-window-size"].as<int>();
	///////////////////////////////////////////////////////////////////
	if (varMap.count("pyr_scale"))
		camData.var.mPyrScale = varMap["pyr_scale"].as<float>();

	if (varMap.count("n_it"))
		camData.var.mnIt = varMap["n_it"].as<int>();

	if (varMap.count("poly_n"))
		camData.var.mPolyN = varMap["poly_n"].as<int>();

	if (varMap.count("poly_sigma"))
		camData.var.mPolySigma = varMap["poly_sigma"].as<float>();

	if (varMap.count("fi"))
		camData.var.mFi = varMap["fi"].as<float>();

	if (varMap.count("lambda"))
		camData.var.mLambda = varMap["lambda"].as<float>();

	if (varMap.count("penalization"))
		camData.var.mPenalization = varMap["penalization"].as<int>();

	if (varMap.count("cycle"))
		camData.var.mCycle = varMap["cycle"].as<int>();

	if (varMap.count("flags"))
		camData.var.mFlags = varMap["flags"].as<int>();
	//////////////////////////////////////////////////////////////////////
	if (varMap.count("nr_plane"))
		camData.constantSpaceBP.mNrPlane = varMap["nr_plane"].as<int>();

	return true;
}

void initIntrinsicsCamParams(bool swapCameras, camData &cam1, camData &cam2, const string& intrParamsFile, VideoCapture& capture1,
		VideoCapture& capture2)
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

void initExtrinsicsCamParams(const string& extrParamsFile, bool& skipCalib, Mat& R, Mat& T, Mat& Q, camData& cam1, camData& cam2)
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

unsigned aquireCalibData(camData &cam1, camData &cam2, camPoints &points1, camPoints &points2, unsigned sampleNums = 25, Size boardSize = Size(9, 6),
		int delay = 100)
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

	Mat cam1NewMat = getOptimalNewCameraMatrix(cam1.mCamMat, cam1.mDistMat, oldSize, 1.), cam2NewMat = getOptimalNewCameraMatrix(cam2.mCamMat,
			cam2.mDistMat, oldSize, 1.);

	cam1.mValidRoi = Rect(0, 0, frame1.cols, frame1.rows);
	cam2.mValidRoi = Rect(0, 0, frame1.cols, frame1.rows);

//	stereoRectify(cam1mat, diff1mat, cam2mat, diff2mat, oldSize, emptyR, translationT, R1, R2, P1, P2, Q);

	initUndistortRectifyMap(cam1.mCamMat, cam1.mDistMat, cam1.mR, cam1NewMat, oldSize, CV_32FC1, cam1.mMap1, cam1.mMap2);
	initUndistortRectifyMap(cam2.mCamMat, cam2.mDistMat, cam2.mR, cam2NewMat, oldSize, CV_32FC1, cam2.mMap1, cam2.mMap2);

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

//// wykomentowac gdy remap'y zostan� odkomentowane:
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
			cornerSubPix(viewGray1, buff1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			cornerSubPix(viewGray2, buff2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			if (clock() - prevTimestamp > delay * 0.001 * CLOCKS_PER_SEC)
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

bool calibrateCameras(camData &cam1, camData &cam2, camPoints &points1, camPoints &points2, Size boardSize, float squareSize, Mat &R, Mat &T, Mat &Q)
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

		double projError =
				stereoCalibrate(objectPoints, points1.mPoints, points2.mPoints, cam1.mCamMat, cam1.mDistMat, cam2.mCamMat, cam2.mDistMat, oldSize, R,
						T, E, F, TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 1e-5),
						CV_CALIB_FIX_ASPECT_RATIO + CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_RATIONAL_MODEL + CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4
								+ CV_CALIB_FIX_K5);

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
		fs << "R" << scCam->getR() << "T" << scCam->getT() << "R1" << cam1.mR << "R2" << cam2.mR << "P1" << cam1.mP << "P2" << cam2.mP << "Q"
				<< scCam->getQ();
		fs.release();
	}
	else
		cerr << "Writing last extrinsics parameters has failed!\n";
}

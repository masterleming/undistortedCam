/*
 * undistorted_cam.cpp
 *
 *  Created on: 28 paź 2013
 *      Author: Krzysztof Pilch
 */

#include <iostream>
#include "undistorted_cam.h"
#include "StereoCam.h"

optionReadStatus readProgramOptions(po::variables_map &vm, po::options_description *&po_description, int argc, char **argv)
{
	po_description = new po::options_description("Program options");

	po_description->add_options()("config-file",po::value<string>(), "File containing configuration information to be used.")
			("help,h", po::value<string>()->implicit_value("all"), "Prints help message. It may be specified to print:\n   \tall, basic, calib, modes, bm, sgbm, var, gpu-bm, bp, cs.\nIf no argument is specified 'all' is presumed.")
			;

	po::options_description basic_options("Basic configuration");
	basic_options.add_options()("mode,m", po::value<string>()->default_value("bm"), "Stereo algorithm to be used. Supported values are:"
					"\n bm     - block matching"
					"\n sgbm   - semi global bm"
					"\n var    - var"
					"\n gpu-bm - gpu block matching"
					"\n bp     - belief propagation"
					"\n cs     - constant space")
			("left,l", po::value<string>(), "Sets left image for static image stereo-vision.")
			("right,r", po::value<string>(), "Sets right image for static image stereo-vision.")
			("static,s", "Enables static image stereo-vision.")
			("shifted", po::value<string>(),"Applies algorithm to two copies of the same image, one of which has a rectangular piece of image shifted in an arbitrary direction. Pass \"left\" to copy left picture or \"right\" to copy right picture")
			("swap-cameras,S","Swaps cameras input so there is no need to move already positioned cameras.")
			("skip", "Skips camera calibration.")
			;

	po::options_description calibration_options("Calibraton configuration");
	calibration_options.add_options()("intrinsics,i", po::value<string>(), "Sets file containing intrinsics calibration parameters.")
			("extrinsics,e", po::value<string>(), "Sets file containing extrinsics calibration parameters.")
			("calibration-samples,c", po::value<int>()->default_value(25), "How many pictures shall be acquired for calibration.")
			("delay,D",po::value<int>()->default_value(1000), "Delay between samples measured in milliseconds.")
			("square-size",po::value<float>()->default_value(25.5),"Distance between square corners. May be any arbitrary value or physical length of the square edge in millimeters or inches or pixels.")
			("board-width,W", po::value<int>()->default_value(9), "Width of the calibrating pattern.")
			("board-height,H",po::value<int>()->default_value(6), "Height of the calibrating pattern.")
			;

	po::options_description algo_all("Stereovision algorithms configuration");

	po::options_description algo_bm("Block Matching options");
	algo_bm.add_options()("sad-window-size,w",po::value<int>()->default_value(21), "Size of the square to search for matching features. Must be an ODD number.")
			("disparities,d", po::value<int>()->default_value(48))
			("preset", po::value<string>()->default_value("basic"), "Preset for options for different types of lenses, does not affect 'disparities' or 'sad-window-size'; available options are:\n   \tbasic, fish-eye, narrow.")
			("speckle-window-size", po::value<int>()->default_value(-1), "Maximum size of smooth disparity regions to consider their noise speckles and invalidate. Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.")
			("speckle-range", po::value<int>()->default_value(-1), "Maximum disparity variation within each connected component. If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16")
			("disp-12-max-diff", po::value<int>()->default_value(-1), "Maxumum allowed difference (in integer pixel units) in the left-right disparity check. Set it to a non-positiv value to disable the check.")
			;

	po::options_description algo_sgbm("Semi-Global Block Matching options");
	algo_sgbm.add_options()("min-disp", po::value<int>()->default_value(0), "Minimum posible disparity value.")
			("disparities,d", po::value<int>()->default_value(48), "Number of disparity levels to be separated. *MUST be dvivisible by 16.*")
			("sad-window-size,w", po::value<int>()->default_value(11), "Size of a match block. *MUST be an odd number.*")
			("p1,1", po::value<int>()->default_value(0), "Disparity change penalisation parameter; setting it to non-zero value causes disparities to be smoother. *It is required that p1 < p2*")
			("p2,2", po::value<int>()->default_value(0), "Another disparity change penalisation parameter; setting it to non-zero value causes disparities to be smoother. *It is required that p1 < p2*")
			("disp-12-max-diff", po::value<int>()->default_value(0), "Maxumum allowed difference (in integer pixel units) in the left-right disparity check. Set it to a non-positiv value to disable the check.")
			("pre-filter-cap", po::value<int>()->default_value(0), "Truncation value for prefiltered image pixels. See OpenCV documentation for details.")
			("uniqueness-ratio", po::value<int>()->default_value(0), "Marigin in percentage by which the best (minimum) cimputed cost function value should \"win\" the second best value to consider the found match correct.")
			("speckle-window-size", po::value<int>()->default_value(0), "Maximum size of smooth disparity regions to consider their noise speckles and invalidate. Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.")
			("speckle-range", po::value<int>()->default_value(0), "Maximum disparity variation within each connected component. If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16")
			("full-dp", po::value<bool>()->default_value(false), "Set it to true to run the full-scale two-pass dynamic programming algorithm. It will consume O(W*H*numDisparities) bytes, which is large for 640x480 stereo and huge for HD-size pictures.")
			;

	po::options_description algo_var("Var options");
	algo_var.add_options()("levels,L", po::value<int>()->default_value(4), "The number of pyramid layers, including the initial image.")
			("pyr-scale", po::value<double>()->default_value(1.0), "Specifies the image scale (<1) to build the pyramids for each image.")
			("iterations,I", po::value<int>()->default_value(4), "The number of iterations the algorithm does at each pyramid level.")
			("min-disp", po::value<int>()->default_value(0), "Minimal possible value for disparity, may be negative one.")
			("max-disp", po::value<int>()->default_value(48), "Maximal possible value for disparity.")
			("poly-n", po::value<int>()->default_value(3), "Size of the pixel neighbourhood used to find polynomial expansion in each pixel.")
			("poly-sigma", po::value<double>()->default_value(1), "Standard deviation of the Gaussian that is used to smooth derivatives that are used as a basis for the polynomial expansion.")
			("fi", po::value<float>()->default_value(1), "The smoothness parameter, or the weight coefficient for the smoothness term.")
			("lambda", po::value<float>()->default_value(0), "The threshold parameter for edge-preserving smoothness.")
			("penalization", po::value<string>()->default_value("tichinov"), "Possible values:\n   \ttichinov, charbonnier,perona-malik")
			("cycle", po::value<string>()->default_value("cycle-o"), "Type of the multigrid cycle. Possible values:\n   \tcycle-o and cycle-v.")
			("flags",po::value<vector<string> >()->multitoken(), "The operation flags; can be a combination of the following:"
					"\n   USE_INITIAL_DISPARITY: Use the input flow as the initial flow approximation."
					"\n   USE_EQUALIZE_HIST: Use the histogram equalization in the pre-processing phase."
					"\n   USE_SMART_ID: Use the smart iteration distribution (SID)."
					"\n   USE_AUTO_PARAMS: Allow the method to initialize the main parameters."
					"\n   USE_MEDIAN_FILTERING: Use the median filer of the solution in the post processing phase.")
			;

	po::options_description algo_gpu_bm("GPU Block Matching options");
	algo_gpu_bm.add_options()("preset", po::value<string>()->default_value("basic"), "Preset to be used, allowed values are:\n   \tbasic, xsobel.")
			("disparities,d", po::value<int>()->default_value(48), "Number of disparities. *MUST be multiple of 8 and less or equal to 256.*")
			("win-size", po::value<int>()->default_value(19), "Match block size.")
			;

	po::options_description algo_gpu_bp("GPU Belief Propagation options");
	algo_gpu_bp.add_options()("disparities,d", po::value<int>()->default_value(48), "Number of disparities")
			("iterations,I", po::value<int>()->default_value(5), "Number of algorithm iterations on each level.")
			("levels,L", po::value<int>()->default_value(4), "Number of levels.")
			("nr-plane", po::value<int>()->default_value(4), "Number of disparity levels on the first level.")
			("max-data-term", po::value<float>()->default_value(0), "Truncation of data cost.")
			("data-weight", po::value<float>()->default_value(0), "Data weight.")
			("max-disc-term", po::value<float>()->default_value(0), "Truncation of discontinuity.")
			("disc-single-jump", po::value<float>()->default_value(0), "Discontinuity single jump.")
			("msg-type", po::value<int>()->default_value(CV_32FC1), "Type of message returned by algorithm. *Only CV_16SC1 and CV_32FC1 types are supported.*")
			;

	po::options_description algo_gpu_cs("GPU Constant Space Belief Propagation options");
	algo_gpu_cs.add_options()("disparities,d", po::value<int>()->default_value(128), "Number of disparities.")
			("iterations,I", po::value<int>()->default_value(8), "Number of algorithm iterations on each level.")
			("levels,L", po::value<int>()->default_value(4), "Number of levels.")
			("nr-plane", po::value<int>()->default_value(4), "Number of disparity levels on the first level.")
			("max-data-term", po::value<float>()->default_value(0), "Truncation of data cost.")
			("data-weight", po::value<float>()->default_value(0), "Data weight.")
			("max-disc-term", po::value<float>()->default_value(0), "Truncation of discontinuity.")
			("disc-single-jump", po::value<float>()->default_value(0), "Discontinuity single jump.")
			("min-disp-th", po::value<int>()->default_value(1), "Minimal disparity threshold.")
			("msg-type", po::value<int>()->default_value(CV_32FC1), "Type of message returned by algorithm. *Only CV_16SC1 and CV_32FC1 types are supported.*")
			;

	algo_all.add(algo_bm)
		.add(algo_sgbm)
		.add(algo_var)
		.add(algo_gpu_bm)
		.add(algo_gpu_bp)
		.add(algo_gpu_cs);

	string cfgFile;
	po::parsed_options first_run = po::command_line_parser(argc, argv).options(*po_description).allow_unregistered().run();
	po::store(first_run, vm);

	vector<string> unrecognised = po::collect_unrecognized(first_run.options, po::include_positional);
	cout << "unrecognised:\n";
	for(vector<string>::iterator i = unrecognised.begin(); i != unrecognised.end(); i++)
		cout << "\t" << *i << endl;

	//Checking "global" options
	//--checks for help message
	if(vm.count("help"))
	{
		string topic = vm["help"].as<string>();
		if(topic == "all")
		{
			//todo: add USAGE information
			po_description->add(basic_options)
				.add(calibration_options)
				.add(algo_all);
			cout << *po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "basic")
		{
			po_description->add(basic_options);
			cout << *po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "calib")
		{
			po_description->add(calibration_options);

			cout << *po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "modes")
		{
			po_description->add(algo_all);

			cout << *po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "bm")
		{
			po_description->add(algo_bm);

			cout << *po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "sgbm")
		{
			po_description->add(algo_sgbm);

			cout << *po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "var")
		{
			po_description->add(algo_var);

			cout << *po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "gpu-bm")
		{
			po_description->add(algo_gpu_bm);

			cout << *po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "bp")
		{
			po_description->add(algo_gpu_bp);

			cout << *po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "cs")
		{
			po_description->add(algo_gpu_cs);

			cout << *po_description << endl;
			return SUCCESS_CLOSE;
		}
		else
		{
			cerr << "Unknown help topic: '" << topic <<"'!\n";
			return FAILURE;
		}
	}
	po_description->add(basic_options);
	po::store(po::command_line_parser(argc, argv).options(*po_description).allow_unregistered()/*.positional(pos_opt)*/.run(), vm);

	//--checking for config file
	if(vm.count("config-file"))
	try
	{
		cfgFile = vm["config-file"].as<string>();
		po::store(po::parse_config_file<char>(cfgFile.c_str(), *po_description, false), vm);
	}
	catch (boost::program_options::reading_file &e)
	{
		//No configuration file -- no biggie
		cerr << "Can not read configuration file '" << cfgFile << "'\n";
		return FAILURE;
	}
	vm.notify();

	//Checking operation MODE
	//--checking for calibration/static mode
	if(!vm.count("static") && !vm.count("skip"))
	{
//		po_description->add(calibration_options);
		po::store(po::command_line_parser(argc, argv).options(calibration_options).allow_unregistered().run(), vm);
		if(cfgFile != "")
			po::store(po::parse_config_file<char>(cfgFile.c_str(), *po_description, false), vm);
		vm.notify();
	}


	string opMode = vm["mode"].as<string>();
	if(opMode == "bm")
	{
		po::store(po::command_line_parser(argc, argv).options(algo_bm).allow_unregistered().run(), vm);
		if(cfgFile != "")
			po::store(po::parse_config_file<char>(cfgFile.c_str(), *po_description, false), vm);
	}
	else if(opMode == "sgbm")
	{
		po::store(po::command_line_parser(argc, argv).options(algo_sgbm).allow_unregistered().run(), vm);
		if(cfgFile != "")
			po::store(po::parse_config_file<char>(cfgFile.c_str(), *po_description, false), vm);
	}
	else if(opMode == "var")
	{
		po::store(po::command_line_parser(argc, argv).options(algo_var).allow_unregistered().run(), vm);
		if(cfgFile != "")
			po::store(po::parse_config_file<char>(cfgFile.c_str(), *po_description, false), vm);
	}
	else if(opMode == "gpu-bm")
	{
		po::store(po::command_line_parser(argc, argv).options(algo_gpu_bm).allow_unregistered().run(), vm);
		if(cfgFile != "")
			po::store(po::parse_config_file<char>(cfgFile.c_str(), *po_description, false), vm);
	}
	else if(opMode == "bp")
	{
		po::store(po::command_line_parser(argc, argv).options(algo_gpu_bp).allow_unregistered().run(), vm);
		if(cfgFile != "")
			po::store(po::parse_config_file<char>(cfgFile.c_str(), *po_description, false), vm);
	}
	else if(opMode == "cs")
	{
		po::store(po::command_line_parser(argc, argv).options(algo_gpu_cs).allow_unregistered().run(), vm);
		if(cfgFile != "")
			po::store(po::parse_config_file<char>(cfgFile.c_str(), *po_description, false), vm);
	}
	else
	{
		cerr << "Unrecognised stereo algorithm: '" << opMode << "'!\n";
		return FAILURE;
	}
	po::notify(vm);
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
		if (prgCfg.mAlgorithm == "bm")
		{
			camData.mMode = SM_BLOCK_MACHING;
			camData.algorithmData.blockMatching.mSadWindowSize = varMap["sad-window-size"].as<int>();
			camData.algorithmData.blockMatching.mDisparities = varMap["disparities"].as<int>();
			
			string preset = varMap["preset"].as<string>();
			if(preset == "basic")
				camData.algorithmData.blockMatching.mPreset = StereoBM::BASIC_PRESET;
			else if(preset == "fish-eye")
				camData.algorithmData.blockMatching.mPreset = StereoBM::FISH_EYE_PRESET;
			else if(preset == "narrow")
				camData.algorithmData.blockMatching.mPreset = StereoBM::NARROW_PRESET;
			else
			{
				cerr << "\nUnrecognised preset was specified: '" << preset << "'!\n";
				return false;
			}
		
			camData.algorithmData.blockMatching.mSpeckleWindowSize = varMap["speckle-window-size"].as<int>();
			camData.algorithmData.blockMatching.mSpeckleRange = varMap["speckle-range"].as<int>();
			camData.algorithmData.blockMatching.mDisp12MaxDiff = varMap["disp-12-max-diff"].as<int>();
		}
		else if (prgCfg.mAlgorithm == "sgbm")
		{
			camData.mMode = SM_SEMI_GLOBAL_BM;
			camData.algorithmData.semiGlobalBM.mMinDisp = varMap["min-disp"].as<int>();
			camData.algorithmData.semiGlobalBM.mDisparities = varMap["disparities"].as<int>();
			camData.algorithmData.semiGlobalBM.mSadWindowSize = varMap["sad-window-size"].as<int>();
			camData.algorithmData.semiGlobalBM.mP1 = varMap["p1"].as<int>();
			camData.algorithmData.semiGlobalBM.mP2 = varMap["p2"].as<int>();
			camData.algorithmData.semiGlobalBM.mDisp12MaxDiff = varMap["disp-12-max-diff"].as<int>();
			camData.algorithmData.semiGlobalBM.mPreFilterCap = varMap["pre-filter-cap"].as<int>();
			camData.algorithmData.semiGlobalBM.mUniquenessRatio = varMap["uniqueness-ratio"].as<int>();
			camData.algorithmData.semiGlobalBM.mSpeckleWindowsSize = varMap["speckle-window-size"].as<int>();
			camData.algorithmData.semiGlobalBM.mSpeckleRange = varMap["speckle-range"].as<int>();
			camData.algorithmData.semiGlobalBM.mFullDP = varMap["full-dp"].as<bool>();
		}
		else if (prgCfg.mAlgorithm == "var")
		{
			camData.mMode = SM_VAR;
			camData.algorithmData.var.mLevels = varMap["levels"].as<int>();
			camData.algorithmData.var.mPyrScale = varMap["pyr-scale"].as<double>();
			camData.algorithmData.var.mIteratnions = varMap["iterations"].as<int>();
			camData.algorithmData.var.mMinDisp = varMap["min-disp"].as<int>();
			camData.algorithmData.var.mMaxDisp = varMap["max-disp"].as<int>();
			camData.algorithmData.var.mPolyN = varMap["poly-n"].as<int>();
			camData.algorithmData.var.mPolySigma = varMap["poly-sigma"].as<double>();
			camData.algorithmData.var.mFi = varMap["fi"].as<float>();
			camData.algorithmData.var.mLambda = varMap["lambda"].as<float>();

			string penalization = varMap["penalization"].as<string>();
			if(penalization == "tichinov")
				camData.algorithmData.var.mPenalization = StereoVar::PENALIZATION_TICHONOV;
			else if(penalization == "charbonnier")
				camData.algorithmData.var.mPenalization = StereoVar::PENALIZATION_CHARBONNIER;
			else if(penalization == "perona-malik")
				camData.algorithmData.var.mPenalization = StereoVar::PENALIZATION_PERONA_MALIK;
			else
			{
				cerr << "\nUnrecognised penalisation mode: '" << penalization << "'!\n";
				return false;
			}

			string cycle = varMap["cycle"].as<string>();
			if(cycle == "cycle-o")
				camData.algorithmData.var.mCycle = StereoVar::CYCLE_O;
			else if (cycle == "cycle-v")
				camData.algorithmData.var.mCycle = StereoVar::CYCLE_V;
			else
			{
				cerr << "\nUnrecognised cycle found: '" << cycle << "'!\n";
				return false;
			}

			camData.algorithmData.var.mFlags = 0;
			if(varMap.count("flags"))
			{
				vector<string> flags = varMap["flags"].as<vector<string> >();
			
				for(vector<string>::iterator i = flags.begin(); i != flags.end(); i++)
				{
					if(*i == "USE_INITIAL_DISPARITY")
						camData.algorithmData.var.mFlags |= StereoVar::USE_INITIAL_DISPARITY;
					else if(*i == "USE_EQUALIZE_HIST")
						camData.algorithmData.var.mFlags |= StereoVar::USE_EQUALIZE_HIST;
					else if(*i == "USE_SMART_ID")
						camData.algorithmData.var.mFlags |= StereoVar::USE_SMART_ID;
					else if(*i == "USE_AUTO_PARAMS")
						camData.algorithmData.var.mFlags |= StereoVar::USE_AUTO_PARAMS;
					else if(*i == "USE_MEDIAN_FILTERING")
						camData.algorithmData.var.mFlags |= StereoVar::USE_MEDIAN_FILTERING;
					else
					{
						cerr << "\nUnrecognised flag found: '" << *i << "'!\n";
						return false;
					}
				}
			}
		}
		else if (prgCfg.mAlgorithm == "gpu-bm")
		{
			camData.mMode = SM_GPU_BM;

			string preset = varMap["preset"].as<string>();
			if(preset == "basic")
				camData.algorithmData.gpuBlockMatching.mPreset = gpu::StereoBM_GPU::BASIC_PRESET;
			else if(preset == "xsobel")
				camData.algorithmData.gpuBlockMatching.mPreset = gpu::StereoBM_GPU::PREFILTER_XSOBEL;
			else
			{
				cerr << "\nUnrecognised preset found: '" << preset << "'!\n";
				return false;
			}

			camData.algorithmData.gpuBlockMatching.mDisparities = varMap["disparities"].as<int>();
			camData.algorithmData.gpuBlockMatching.mWindowSize = varMap["win-size"].as<int>();
		}
		else if (prgCfg.mAlgorithm == "bp")
		{
			camData.mMode = SM_BELIEF_PROPAGATION;
			camData.algorithmData.beliefPropagation.mDisparities = varMap["disparities"].as<int>();
			camData.algorithmData.beliefPropagation.mIterations = varMap["iterations"].as<int>();
			camData.algorithmData.beliefPropagation.mLevels = varMap["levels"].as<int>();
			camData.algorithmData.beliefPropagation.mNrPlane = varMap["nr-plane"].as<int>();
			camData.algorithmData.beliefPropagation.mMaxDataTerm = varMap["max-data-term"].as<float>();
			camData.algorithmData.beliefPropagation.mDataWeight = varMap["data-weight"].as<float>();
			camData.algorithmData.beliefPropagation.mMaxDiscTerm = varMap["max-disc-term"].as<float>();
			camData.algorithmData.beliefPropagation.mDiscSingleJump = varMap["disc-single-jump"].as<float>();
			camData.algorithmData.beliefPropagation.mMsgType = varMap["msg-type"].as<int>();

		}
		else if (prgCfg.mAlgorithm == "cs")
		{
			camData.mMode = SM_CONSTANT_SPACE_BP;
			camData.mMode = SM_BELIEF_PROPAGATION;
			camData.algorithmData.beliefPropagation.mDisparities = varMap["disparities"].as<int>();
			camData.algorithmData.beliefPropagation.mIterations = varMap["iterations"].as<int>();
			camData.algorithmData.beliefPropagation.mLevels = varMap["levels"].as<int>();
			camData.algorithmData.beliefPropagation.mNrPlane = varMap["nr-plane"].as<int>();
			camData.algorithmData.beliefPropagation.mMaxDataTerm = varMap["max-data-term"].as<float>();
			camData.algorithmData.beliefPropagation.mDataWeight = varMap["data-weight"].as<float>();
			camData.algorithmData.beliefPropagation.mMaxDiscTerm = varMap["max-disc-term"].as<float>();
			camData.algorithmData.beliefPropagation.mDiscSingleJump = varMap["disc-single-jump"].as<float>();
			camData.algorithmData.beliefPropagation.mMinDispTh = varMap["min-disp-th"].as<int>();
			camData.algorithmData.beliefPropagation.mMsgType = varMap["msg-type"].as<int>();
		}
		else
		{
			cerr << "\nInvalid algorithm specified: '" << prgCfg.mAlgorithm << "'!\n";
			return false;
		}
	}
	else
	{
		cerr << "\nNo algorithm specified!\n";
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
		calibCfg.mSkip = true;
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

bool initializeCUDA(int chosen_cuda_card)
{
	cout << "\nStarting Nvidia CUDA\n";
	int cuda_count = gpu::getCudaEnabledDeviceCount();
	cout << "---\nFound " << cuda_count << " compatible devices\n";

	if(cuda_count == 0)
		return false;

	int did_chosen = (chosen_cuda_card >= cuda_count ? 0 : chosen_cuda_card);

	gpu::setDevice(did_chosen);
	cout << "Set " << did_chosen << " CUDA card for use in this program.\n";

	gpu::DeviceInfo cudaInfo(did_chosen);

	cout << "***********\nChosen card DeviceInfo:"
		"\n\tName:\t\t" << cudaInfo.name() <<
		"\n\tVersion:\t" << cudaInfo.majorVersion() << '.' << cudaInfo.minorVersion() <<
		"\n\tMultiprocesors:\t" << cudaInfo.multiProcessorCount() <<
		"\n\tTotal memory:\t" << cudaInfo.totalMemory() <<
		"\n\tFree memory:\t" << cudaInfo.freeMemory() <<
		"\n\n\tIs compatible?\t" << (cudaInfo.isCompatible() ? "YES" : "NO") <<
		"\n\tSupported features:" <<
		"\n\t\tFeature set compute 10:\t" << (cudaInfo.supports(gpu::FEATURE_SET_COMPUTE_10) ? "YES" : "NO") <<
		"\n\t\tFeature set compute 11:\t" << (cudaInfo.supports(gpu::FEATURE_SET_COMPUTE_11) ? "YES" : "NO") <<
		"\n\t\tFeature set compute 12:\t" << (cudaInfo.supports(gpu::FEATURE_SET_COMPUTE_12) ? "YES" : "NO") <<
		"\n\t\tFeature set compute 13:\t" << (cudaInfo.supports(gpu::FEATURE_SET_COMPUTE_13) ? "YES" : "NO") <<
		"\n\t\tFeature set compute 20:\t" << (cudaInfo.supports(gpu::FEATURE_SET_COMPUTE_20) ? "YES" : "NO") <<
		"\n\t\tFeature set compute 21:\t" << (cudaInfo.supports(gpu::FEATURE_SET_COMPUTE_21) ? "YES" : "NO") <<
		"\n\t\tFeature set compute 30:\t" << (cudaInfo.supports(gpu::FEATURE_SET_COMPUTE_30) ? "YES" : "NO") <<
		"\n\t\tFeature set compute 35:\t" << (cudaInfo.supports(gpu::FEATURE_SET_COMPUTE_35) ? "YES" : "NO") <<
		"\n\t\tGlobal Atomics:\t\t" << (cudaInfo.supports(gpu::GLOBAL_ATOMICS) ? "YES" : "NO") <<
		"\n\t\tShared Atomics:\t\t" << (cudaInfo.supports(gpu::SHARED_ATOMICS) ? "YES" : "NO") <<
		"\n\t\tNative Double:\t\t" << (cudaInfo.supports(gpu::NATIVE_DOUBLE) ? "YES" : "NO") <<
		"\n\t\tWarp shuffle functions:\t" << (cudaInfo.supports(gpu::WARP_SHUFFLE_FUNCTIONS) ? "YES" : "NO") <<
		"\n\t\tDynamic pararellism:\t" << (cudaInfo.supports(gpu::DYNAMIC_PARALLELISM) ? "YES" : "NO") <<
		endl;

	return cudaInfo.isCompatible();
}
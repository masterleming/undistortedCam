/*
 * ConfigParser.cpp
 *
 *  Created on: 31-08-2014
 *      Author: Krzysztof Pilch
 */

#include "ConfigParser.h"

ConfigParser::ConfigParser()
	: mArgc(0), mArgv(NULL)
{
}

ConfigParser::ConfigParser(int argc, char** argv)
	: mArgc(argc), mArgv(argv)
{
}
	
optionReadStatus ConfigParser::parse(calibrationCfg &calibCfg, programCfg &prgCfg, stereoModeData &camData)
{
	try
	{
		switch(readProgramOptions())
		{
		case SUCCESS:
			break;
		case SUCCESS_CLOSE:
			return SUCCESS_CLOSE;
		case FAILURE:
			return FAILURE;
		}
		
		if(!getRuntimeConfiguration(calibCfg, prgCfg, camData))
		{
			help(cerr);
			return FAILURE;
		}
	}
	catch(char *e)
	{
		cerr << e << endl;
		help(cerr);
		return FAILURE;
	}
	
	return SUCCESS;
}

optionReadStatus ConfigParser::readProgramOptions()
{
	po::options_description po_description("Program options");

	po_description.add_options()("config-file",po::value<string>(), "File containing configuration information to be used.")
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
			("penalization", po::value<string>()->default_value("tichinov"), "Possible values:\n   \ttichinov, charbonnier, perona-malik")
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

	const vector< boost::shared_ptr< po::option_description > > opts = po_description.options();
	for(unsigned i = 0; i < opts.size(); i++)
	//for(vector<shared_ptr<po:: option_description> >::const_iterator i = po_description.options().begin(); i != po_description.options().end(); i++)
		mConfig.add(opts[i]);

	mConfig.add(basic_options)
		.add(calibration_options)
		.add(algo_all);
	
	string cfgFile;
	po::parsed_options first_run = po::command_line_parser(mArgc, mArgv).options(po_description).allow_unregistered().run();
	po::store(first_run, mVarMap);

	vector<string> unrecognised = po::collect_unrecognized(first_run.options, po::include_positional);
	cout << "unrecognised:\n";
	for(vector<string>::iterator i = unrecognised.begin(); i != unrecognised.end(); i++)
		cout << "\t" << *i << endl;

	//Checking "global" options
	//--checks for help message
	if(mVarMap.count("help"))
	{
		string topic = mVarMap["help"].as<string>();
		if(topic == "all")
		{
			//todo: add USAGE information
			po_description.add(basic_options)
				.add(calibration_options)
				.add(algo_all);
			cout << po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "basic")
		{
			po_description.add(basic_options);
			cout << po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "calib")
		{
			po_description.add(calibration_options);

			cout << po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "modes")
		{
			po_description.add(algo_all);

			cout << po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "bm")
		{
			po_description.add(algo_bm);

			cout << po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "sgbm")
		{
			po_description.add(algo_sgbm);

			cout << po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "var")
		{
			po_description.add(algo_var);

			cout << po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "gpu-bm")
		{
			po_description.add(algo_gpu_bm);

			cout << po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "bp")
		{
			po_description.add(algo_gpu_bp);

			cout << po_description << endl;
			return SUCCESS_CLOSE;
		}
		else if(topic == "cs")
		{
			po_description.add(algo_gpu_cs);

			cout << po_description << endl;
			return SUCCESS_CLOSE;
		}
		else
		{
			cerr << "Unknown help topic: '" << topic <<"'!\n";
			return FAILURE;
		}
	}
	po_description.add(basic_options);
	po::store(po::command_line_parser(mArgc, mArgv).options(po_description).allow_unregistered()/*.positional(pos_opt)*/.run(), mVarMap);

	//--checking for config file
	if(mVarMap.count("config-file"))
	try
	{
		cfgFile = mVarMap["config-file"].as<string>();
		po::store(po::parse_config_file<char>(cfgFile.c_str(), po_description, true), mVarMap);
	}
	catch (boost::program_options::reading_file &e)
	{
		//No configuration file -- no biggie
		cerr << "Can not read configuration file '" << cfgFile << "'\n";
		return FAILURE;
	}
	mVarMap.notify();

	//Checking operation MODE
	//--checking for calibration/static mode
	if(!mVarMap.count("static") && !mVarMap.count("skip"))
	{
//		po_description.add(calibration_options);
		po::store(po::command_line_parser(mArgc, mArgv).options(calibration_options).allow_unregistered().run(), mVarMap);
		if(cfgFile != "")
			po::store(po::parse_config_file<char>(cfgFile.c_str(), po_description, true), mVarMap);
		mVarMap.notify();
	}


	string opMode = mVarMap["mode"].as<string>();
	if(opMode == "bm")
	{
		po::store(po::command_line_parser(mArgc, mArgv).options(algo_bm).allow_unregistered().run(), mVarMap);
		if(cfgFile != "")
			po::store(po::parse_config_file<char>(cfgFile.c_str(), algo_bm, true), mVarMap);
	}
	else if(opMode == "sgbm")
	{
		po::store(po::command_line_parser(mArgc, mArgv).options(algo_sgbm).allow_unregistered().run(), mVarMap);
		if(cfgFile != "")
			po::store(po::parse_config_file<char>(cfgFile.c_str(), algo_sgbm, true), mVarMap);
	}
	else if(opMode == "var")
	{
		po::store(po::command_line_parser(mArgc, mArgv).options(algo_var).allow_unregistered().run(), mVarMap);
		if(cfgFile != "")
			po::store(po::parse_config_file<char>(cfgFile.c_str(), algo_var, true), mVarMap);
	}
	else if(opMode == "gpu-bm")
	{
		po::store(po::command_line_parser(mArgc, mArgv).options(algo_gpu_bm).allow_unregistered().run(), mVarMap);
		if(cfgFile != "")
			po::store(po::parse_config_file<char>(cfgFile.c_str(), algo_gpu_bm, true), mVarMap);
	}
	else if(opMode == "bp")
	{
		po::store(po::command_line_parser(mArgc, mArgv).options(algo_gpu_bp).allow_unregistered().run(), mVarMap);
		if(cfgFile != "")
			po::store(po::parse_config_file<char>(cfgFile.c_str(), algo_gpu_bp, true), mVarMap);
	}
	else if(opMode == "cs")
	{
		po::store(po::command_line_parser(mArgc, mArgv).options(algo_gpu_cs).allow_unregistered().run(), mVarMap);
		if(cfgFile != "")
			po::store(po::parse_config_file<char>(cfgFile.c_str(), algo_gpu_cs, true), mVarMap);
	}
	else
	{
		cerr << "Unrecognised stereo algorithm: '" << opMode << "'!\n";
		return FAILURE;
	}
	po::notify(mVarMap);
	return SUCCESS;
}

bool ConfigParser::getRuntimeConfiguration(calibrationCfg &calibCfg, programCfg &prgCfg, stereoModeData &camData)
{
	if (mVarMap.count("mode"))
	{
		prgCfg.mAlgorithm = mVarMap["mode"].as<string>();
		if (prgCfg.mAlgorithm == "bm")
		{
			camData.mMode = SM_BLOCK_MACHING;
			camData.mAlgorithmData.blockMatching.mSadWindowSize = mVarMap["sad-window-size"].as<int>();
			camData.mAlgorithmData.blockMatching.mDisparities = mVarMap["disparities"].as<int>();
			
			string preset = mVarMap["preset"].as<string>();
			if(preset == "basic")
				camData.mAlgorithmData.blockMatching.mPreset = StereoBM::BASIC_PRESET;
			else if(preset == "fish-eye")
				camData.mAlgorithmData.blockMatching.mPreset = StereoBM::FISH_EYE_PRESET;
			else if(preset == "narrow")
				camData.mAlgorithmData.blockMatching.mPreset = StereoBM::NARROW_PRESET;
			else
			{
				cerr << "\nUnrecognised preset was specified: '" << preset << "'!\n";
				return false;
			}
		
			camData.mAlgorithmData.blockMatching.mSpeckleWindowSize = mVarMap["speckle-window-size"].as<int>();
			camData.mAlgorithmData.blockMatching.mSpeckleRange = mVarMap["speckle-range"].as<int>();
			camData.mAlgorithmData.blockMatching.mDisp12MaxDiff = mVarMap["disp-12-max-diff"].as<int>();
		}
		else if (prgCfg.mAlgorithm == "sgbm")
		{
			camData.mMode = SM_SEMI_GLOBAL_BM;
			camData.mAlgorithmData.semiGlobalBM.mMinDisp = mVarMap["min-disp"].as<int>();
			camData.mAlgorithmData.semiGlobalBM.mDisparities = mVarMap["disparities"].as<int>();
			camData.mAlgorithmData.semiGlobalBM.mSadWindowSize = mVarMap["sad-window-size"].as<int>();
			camData.mAlgorithmData.semiGlobalBM.mP1 = mVarMap["p1"].as<int>();
			camData.mAlgorithmData.semiGlobalBM.mP2 = mVarMap["p2"].as<int>();
			camData.mAlgorithmData.semiGlobalBM.mDisp12MaxDiff = mVarMap["disp-12-max-diff"].as<int>();
			camData.mAlgorithmData.semiGlobalBM.mPreFilterCap = mVarMap["pre-filter-cap"].as<int>();
			camData.mAlgorithmData.semiGlobalBM.mUniquenessRatio = mVarMap["uniqueness-ratio"].as<int>();
			camData.mAlgorithmData.semiGlobalBM.mSpeckleWindowsSize = mVarMap["speckle-window-size"].as<int>();
			camData.mAlgorithmData.semiGlobalBM.mSpeckleRange = mVarMap["speckle-range"].as<int>();
			camData.mAlgorithmData.semiGlobalBM.mFullDP = mVarMap["full-dp"].as<bool>();
		}
		else if (prgCfg.mAlgorithm == "var")
		{
			camData.mMode = SM_VAR;
			camData.mAlgorithmData.var.mLevels = mVarMap["levels"].as<int>();
			camData.mAlgorithmData.var.mPyrScale = mVarMap["pyr-scale"].as<double>();
			camData.mAlgorithmData.var.mIteratnions = mVarMap["iterations"].as<int>();
			camData.mAlgorithmData.var.mMinDisp = mVarMap["min-disp"].as<int>();
			camData.mAlgorithmData.var.mMaxDisp = mVarMap["max-disp"].as<int>();
			camData.mAlgorithmData.var.mPolyN = mVarMap["poly-n"].as<int>();
			camData.mAlgorithmData.var.mPolySigma = mVarMap["poly-sigma"].as<double>();
			camData.mAlgorithmData.var.mFi = mVarMap["fi"].as<float>();
			camData.mAlgorithmData.var.mLambda = mVarMap["lambda"].as<float>();

			string penalization = mVarMap["penalization"].as<string>();
			if(penalization == "tichinov")
				camData.mAlgorithmData.var.mPenalization = StereoVar::PENALIZATION_TICHONOV;
			else if(penalization == "charbonnier")
				camData.mAlgorithmData.var.mPenalization = StereoVar::PENALIZATION_CHARBONNIER;
			else if(penalization == "perona-malik")
				camData.mAlgorithmData.var.mPenalization = StereoVar::PENALIZATION_PERONA_MALIK;
			else
			{
				cerr << "\nUnrecognised penalisation mode: '" << penalization << "'!\n";
				return false;
			}

			string cycle = mVarMap["cycle"].as<string>();
			if(cycle == "cycle-o")
				camData.mAlgorithmData.var.mCycle = StereoVar::CYCLE_O;
			else if (cycle == "cycle-v")
				camData.mAlgorithmData.var.mCycle = StereoVar::CYCLE_V;
			else
			{
				cerr << "\nUnrecognised cycle found: '" << cycle << "'!\n";
				return false;
			}

			camData.mAlgorithmData.var.mFlags = 0;
			if(mVarMap.count("flags"))
			{
				vector<string> flags = mVarMap["flags"].as<vector<string> >();
			
				for(vector<string>::iterator i = flags.begin(); i != flags.end(); i++)
				{
					if(*i == "USE_INITIAL_DISPARITY")
						camData.mAlgorithmData.var.mFlags |= StereoVar::USE_INITIAL_DISPARITY;
					else if(*i == "USE_EQUALIZE_HIST")
						camData.mAlgorithmData.var.mFlags |= StereoVar::USE_EQUALIZE_HIST;
					else if(*i == "USE_SMART_ID")
						camData.mAlgorithmData.var.mFlags |= StereoVar::USE_SMART_ID;
					else if(*i == "USE_AUTO_PARAMS")
						camData.mAlgorithmData.var.mFlags |= StereoVar::USE_AUTO_PARAMS;
					else if(*i == "USE_MEDIAN_FILTERING")
						camData.mAlgorithmData.var.mFlags |= StereoVar::USE_MEDIAN_FILTERING;
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

			string preset = mVarMap["preset"].as<string>();
			if(preset == "basic")
				camData.mAlgorithmData.gpuBlockMatching.mPreset = gpu::StereoBM_GPU::BASIC_PRESET;
			else if(preset == "xsobel")
				camData.mAlgorithmData.gpuBlockMatching.mPreset = gpu::StereoBM_GPU::PREFILTER_XSOBEL;
			else
			{
				cerr << "\nUnrecognised preset found: '" << preset << "'!\n";
				return false;
			}

			camData.mAlgorithmData.gpuBlockMatching.mDisparities = mVarMap["disparities"].as<int>();
			camData.mAlgorithmData.gpuBlockMatching.mWindowSize = mVarMap["win-size"].as<int>();
		}
		else if (prgCfg.mAlgorithm == "bp")
		{
			camData.mMode = SM_BELIEF_PROPAGATION;
			camData.mAlgorithmData.beliefPropagation.mDisparities = mVarMap["disparities"].as<int>();
			camData.mAlgorithmData.beliefPropagation.mIterations = mVarMap["iterations"].as<int>();
			camData.mAlgorithmData.beliefPropagation.mLevels = mVarMap["levels"].as<int>();
			camData.mAlgorithmData.beliefPropagation.mNrPlane = mVarMap["nr-plane"].as<int>();
			camData.mAlgorithmData.beliefPropagation.mMaxDataTerm = mVarMap["max-data-term"].as<float>();
			camData.mAlgorithmData.beliefPropagation.mDataWeight = mVarMap["data-weight"].as<float>();
			camData.mAlgorithmData.beliefPropagation.mMaxDiscTerm = mVarMap["max-disc-term"].as<float>();
			camData.mAlgorithmData.beliefPropagation.mDiscSingleJump = mVarMap["disc-single-jump"].as<float>();
			camData.mAlgorithmData.beliefPropagation.mMsgType = mVarMap["msg-type"].as<int>();

		}
		else if (prgCfg.mAlgorithm == "cs")
		{
			camData.mMode = SM_CONSTANT_SPACE_BP;
			camData.mMode = SM_BELIEF_PROPAGATION;
			camData.mAlgorithmData.beliefPropagation.mDisparities = mVarMap["disparities"].as<int>();
			camData.mAlgorithmData.beliefPropagation.mIterations = mVarMap["iterations"].as<int>();
			camData.mAlgorithmData.beliefPropagation.mLevels = mVarMap["levels"].as<int>();
			camData.mAlgorithmData.beliefPropagation.mNrPlane = mVarMap["nr-plane"].as<int>();
			camData.mAlgorithmData.beliefPropagation.mMaxDataTerm = mVarMap["max-data-term"].as<float>();
			camData.mAlgorithmData.beliefPropagation.mDataWeight = mVarMap["data-weight"].as<float>();
			camData.mAlgorithmData.beliefPropagation.mMaxDiscTerm = mVarMap["max-disc-term"].as<float>();
			camData.mAlgorithmData.beliefPropagation.mDiscSingleJump = mVarMap["disc-single-jump"].as<float>();
			camData.mAlgorithmData.beliefPropagation.mMinDispTh = mVarMap["min-disp-th"].as<int>();
			camData.mAlgorithmData.beliefPropagation.mMsgType = mVarMap["msg-type"].as<int>();
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

	if (mVarMap.count("static"))
	{
		if (!(mVarMap.count("left") && mVarMap.count("right")))
		{
			cerr << "Too few arguments passed to start static image stereo-vision. It requires both left and right images to be specified!";
			return false;
		}
		prgCfg.mStatic = true;
		prgCfg.mLeft = mVarMap["left"].as<string>();
		prgCfg.mRight = mVarMap["right"].as<string>();
		calibCfg.mSkip = true;
	}

	if (mVarMap.count("shifted"))
	{
		string sDir = mVarMap["shifted"].as<string>();
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

	unsigned isCamParametrized = 0;

	if (mVarMap.count("intrinsics"))
	{
		prgCfg.mIntrinsics = mVarMap["intrinsics"].as<string>();
		isCamParametrized++;
	}

	if (mVarMap.count("extrinsics"))
	{
		prgCfg.mExtrinsics = mVarMap["extrinsics"].as<string>();
		isCamParametrized++;
	}

	if (mVarMap.count("swap-cameras"))
		prgCfg.mSwapCameras = true;
	//////////////////////////////////////////////////////////////////
	if (mVarMap.count("skip") || isCamParametrized == 2)
		calibCfg.mSkip = true;

	if (mVarMap.count("calibration-samples"))
		calibCfg.mCalibSamples = mVarMap["calibration-samples"].as<int>();

	if (mVarMap.count("delay"))
		calibCfg.mDelay = mVarMap["delay"].as<int>();

	if (mVarMap.count("square-size"))
		calibCfg.mSquareSize = mVarMap["square-size"].as<float>();

	if (mVarMap.count("board-width"))
		calibCfg.mBoardWidth = mVarMap["board-width"].as<int>();
	else if (!calibCfg.mSkip)
	{
		cerr << "board-width MUST be specified to use calibration!";
		return false;
	}

	if (mVarMap.count("board-height"))
		calibCfg.mBoardHeight = mVarMap["board-height"].as<int>();
	else if (!calibCfg.mSkip)
	{
		cerr << "board-height MUST be specified to use calibration!";
		return false;
	}

	calibCfg.mBoardSize = Size(calibCfg.mBoardWidth, calibCfg.mBoardHeight);

	return true;
}

void ConfigParser::help(ostream &stream)
{
	stream << mConfig << endl;
}

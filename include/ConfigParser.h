/*
 * ConfigParser.h
 *
 *  Created on: 31-08-2014
 *      Author: Krzysztof Pilch
 */

#ifndef CONFIG_PARSER_H_
#define CONFIG_PARSER_H_

#include <iostream>
#include "undistorted_cam.h"

class ConfigParser
{
private:
	int mArgc;
	char **mArgv;
	
	po::variables_map mVarMap;
	po::options_description mConfig;
	
private:
	ConfigParser();
	optionReadStatus readProgramOptions();
	bool getRuntimeConfiguration(calibrationCfg &calibCfg, programCfg &prgCfg, stereoModeData &camData);

public:
	ConfigParser(int argc, char** argv);
	
	optionReadStatus parse(calibrationCfg &calibCfg, programCfg &prgCfg, stereoModeData &camData);
	void help(ostream &stream);
};

#endif //CONFIG_PARSER_H_

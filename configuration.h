/*
 * configuration.h
 *
 *  Created on: Jan 18, 2020
 *      Author: richard
 */

#pragma once

#include <string>

struct Configuration
{
	std::string elevationDir;
	std::string username;
	std::string password;
	std::string host;
	std::string eleUsername;
	std::string elePassword;
};

Configuration readConfigFile (const std::string &configurationDir);

std::string getEnvVar (const std::string &name, const std::string &defaultValue);

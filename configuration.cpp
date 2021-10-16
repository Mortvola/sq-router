/*
 * configuration.cpp
 *
 *  Created on: Jan 18, 2020
 *      Author: richard
 */


#include "configuration.h"
#include <fstream>
#include <regex>
#include <iostream>


std::string getEnvVar (const std::string &name, const std::string &defaultValue)
{
	auto value = std::getenv(name.c_str ());

	if (value)
	{
		return value;
	}

	return defaultValue;
}


Configuration readConfigFile (const std::string &configurationDir)
{
	Configuration config;

	config.elevationDir = "./elevations";
	config.username = ".";
	config.password = ".";
	config.host = "localhost";
	config.eleUsername = ".";
	config.elePassword = ".";

	std::string configurationFilename {configurationDir + "/routeFind.config"};

	std::ifstream configFile (configurationFilename);

	if (configFile.is_open())
	{
		std::regex regex ("^[ \\t]*([A-Za-z_]+)[ \\t]*=[ \\t]*(.+)[ \\t]*$");
		std::regex comment ("^[ \\t]*#.*$");
		std::regex empty ("^[ \\t]*$");

		while (!configFile.eof ())
		{
			std::string line;

			std::getline(configFile, line);

			if (!std::regex_search(line, empty) && !std::regex_search(line, comment))
			{
				std::smatch match;

				if (std::regex_match (line, match, regex))
				{
					if (match.size () == 3)
					{
						if (match[1].str() == "ELEVATION_DIRECTORY")
						{
							config.elevationDir = match[2].str ();
						}
						else if (match[1].str() == "USERNAME")
						{
							config.username = match[2].str ();
						}
						else if (match[1].str() == "PASSWORD")
						{
							config.password = match[2].str ();
						}
						else if (match[1].str() == "HOST")
						{
							config.host = match[2].str ();
						}
						else if (match[1].str() == "ELE_USERNAME")
						{
							config.eleUsername = match[2].str ();
						}
						else if (match[1].str() == "ELE_PASSWORD")
						{
							config.elePassword = match[2].str ();
						}
						else
						{
							std::cerr << "Unknown setting on line: " << match[1].str() << std::endl;
						}
					}
					else
					{
						std::cerr << "Not enough elements on line: " << line << std::endl;
					}
				}
				else
				{
					std::cerr << "Could not parse configuration line: " << line << std::endl;
				}
			}
		}
	}
	else
	{
		std::cerr << "Could not read configuration file: " << configurationFilename << std::endl;
	}

	return config;
}



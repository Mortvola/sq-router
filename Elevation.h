/*
 * Elevation.h
 *
 *  Created on: Oct 26, 2019
 *      Author: richard
 */

#pragma once

#include "LatLng.h"
#include "ElevationFile.h"
#include <string>
#include <tuple>
#include <fstream>
#include <cmath>

class TextureCoord
{
public:

  double s;
  double t;

	operator Json::Value () const
	{
		Json::Value value;

		value["s"] = s;
		value["t"] = t;

		return value;
	}

	friend std::ostream& operator<<(std::ostream &os, const TextureCoord &coord)
	{
		os << std::setprecision(15) <<
			" (" << coord.s <<
			", " << coord.t <<
			") ";

		return os;
	}

private:
};

struct Terrain
{
  LatLng sw;
  LatLng ne;
  TextureCoord textureSW;
  TextureCoord textureNE;
  std::vector<std::vector<uint16_t>> points;
  std::vector<std::vector<double>> centers;
};


class Elevation
{
public:

	static Elevation *getInstance ();

	double getElevation (const LatLng &point);
	
  Terrain getElevationArea (const LatLng &point, int dimensions);

  Terrain getElevationTile (int x, int y, int z);

private:
  std::vector<Json::Value> getRoutes(const LatLng &sw, const LatLng &ne);
  const ElevationFile &loadFile(const LatLng &point);

  std::map<std::string, ElevationFile> m_files;
  uint64_t m_accessCounter {0};
};

void elevationPathSet (const std::string &path);

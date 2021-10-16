#include "DBConnection.h"
#include "configuration.h"
#include <tuple>
#include <iostream>
#include <unistd.h>
#include <iomanip>


std::tuple<int, int, int, int> getMapExtents ()
{
	DBConnection::prepare ("queryExtents",
		"select "
		"	floor(min(ST_XMin(Box))) LngMin, "
		"	floor(min(ST_YMin(Box))) LatMin, "
		"	ceil(max(ST_XMax(Box))) LngMax, "
		"	ceil(max(ST_YMax(Box))) LatMax "
		"from ( "
		"	select ST_Transform(ST_SetSRID(ST_Extent(way),3857), 4326) Box "
		"	from planet_osm_route) AS extents ");

	auto extents = DBConnection::execPrepared1 ("queryExtents");

	return std::tuple<int, int, int, int>(
			extents["LatMin"].as<int>(), extents["LngMin"].as<int>(),
			extents["LatMax"].as<int>(), extents["LngMax"].as<int>());
}


int getCount (
	int lat,
	int lng)
{
	DBConnection::prepare ("queryCounts",
		"select osm_id "
		"	from planet_osm_route l1 "
		"	where l1.way && ST_SetSRID(ST_MakeBox2D(ST_Transform(ST_SetSRID(ST_MakePoint($2, $1), 4326), 3857), "
		"				ST_Transform(ST_SetSRID(ST_MakePoint($4, $3), 4326), 3857)), 3857) "
		"LIMIT 1");

	auto count = DBConnection::execPrepared ("queryCounts", lat - 1, lng, lat, lng + 1);

	return count.size ();
}

int main(int argc, char *argv[])
{
	if (argc != 1)
	{
		std::cerr << "Usage: elevationBuilder\n";
		return -1;
	}

	try
	{
		auto configurationDir = getEnvVar("CONFIGURATION_DIRECTORY", "/etc/routeFind/");

		auto config = readConfigFile (configurationDir);

		DBConnection::connect (config.username, config.password);

		int result = chdir(config.elevationDir.c_str ());
		if (result != 0)
		{
			throw std::runtime_error("elevation folder does not exist: " + config.elevationDir);
		}

		int latMin, lngMin;
		int latMax, lngMax;

		std::tie(latMin, lngMin, latMax, lngMax) = getMapExtents ();

		latMax = std::min(latMax, 59);
		latMin = std::max(latMin, 0);

		for (int lat = latMax; lat > latMin; lat--)
		{
			for (int lng = lngMin; lng < lngMax; lng++)
			{
				auto count = getCount(lat, lng);

				if (count > 0)
				{
					std::stringstream filename;
					filename << std::setfill('0');

					if (lat < 0)
					{
						filename << std::setw(0) << "S";
						filename << std::setw(2) << -lat;
					}
					else
					{
						filename << std::setw(0) << "N";
						filename << std::setw(2) << lat;
					}

					if (lng < 0)
					{
						filename << std::setw(0) << "W";
						filename << std::setw(3) << -lng;
					}
					else
					{
						filename << std::setw(0) << "E";
						filename << std::setw(3) << lng;
					}

					system ((std::string("./getElevation.sh ") + filename.str () + " " + config.elePassword).c_str ());
				}
				std::cerr << lat << ", " << lng << ": " << count << std::endl;
			}
		}
	}
	catch (const pqxx::pqxx_exception &e)
	{
		std::cerr << e.base().what() << std::endl;
		const pqxx::sql_error *s = dynamic_cast<const pqxx::sql_error*>(&e.base());
		if (s) std::cerr << "Query was: " << s->query() << std::endl;
	}
	catch (...)
	{
		std::cerr << "failed to connect\n";
	}

	return 0;
}

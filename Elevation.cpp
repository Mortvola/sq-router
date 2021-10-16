#include "Elevation.h"
#include "DBConnection.h"
#include "Profiler.h"
#include <cinttypes>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

static std::string g_basepath{"./elevations"};

Elevation *Elevation::getInstance() {
  static Elevation elevation;

  return &elevation;
}

void elevationPathSet(const std::string &path) { g_basepath = path + "/"; }

std::string elevationPathGet() { return g_basepath; }

// Returns the row and col of the lower left corner relative to the point
std::tuple<int, int> getFilePosition(const LatLng &point) {
  // Determine file name
  int latInt = std::floor(point.m_lat);
  int lngInt = std::floor(point.m_lng);
  int row{};
  int col{};

  row = 3600 - std::floor((point.m_lat - latInt) * 3600);
  col = std::floor((point.m_lng - lngInt) * 3600);

  return std::tuple<int, int>(row, col);
}

std::tuple<std::string, LatLng> getBaseFileName(const LatLng &point) {
  // Determine file name
  int latInt = std::floor(point.m_lat);
  int lngInt = std::floor(point.m_lng);

  std::stringstream filename;
  filename << std::setfill('0');

  if (point.m_lat >= 0) {
    filename << "N";
    filename << std::setw(2) << latInt;
  } else {
    filename << "S";
    filename << std::setw(2) << -latInt;
  }

  filename << std::setfill('0');

  if (point.m_lng >= 0) {
    filename << "E";
    filename << std::setw(3) << lngInt;
  } else {
    filename << "W";
    filename << std::setw(3) << -lngInt;
  }

  return std::tuple<std::string, LatLng>(filename.str(),
                                         LatLng(latInt, lngInt));
}

const ElevationFile &Elevation::loadFile(const LatLng &point) {
  std::string filename;
  LatLng latLng;

  std::tie(filename, latLng) = getBaseFileName(point);

  auto fullFileName = elevationPathGet() + filename + ".hgt";

  // See if the file has already been loaded.
  auto iter = m_files.find(fullFileName);

  if (iter == m_files.end()) {
    // No, it hasn't been loaded. Load it now.
    auto file = std::ifstream(fullFileName, std::ios_base::binary);

    if (file.is_open()) {
      file.seekg(0, std::ios_base::end);
      auto size = file.tellg();
      file.seekg(0);

      std::cerr << "loading elevation file " << fullFileName << " of size "
                << size << " bytes" << std::endl;
      std::tie(iter, std::ignore) = m_files.emplace(
          std::pair<std::string, ElevationFile>(fullFileName, {}));

      iter->second.m_latLng = latLng;
      iter->second.m_buffer.resize(size / sizeof(int16_t));
      file.read(reinterpret_cast<char *>(iter->second.m_buffer.data()), size);
    } else {
      throw std::runtime_error("Could not read file " + fullFileName);
    }
  }

  m_accessCounter++;
  iter->second.m_accessCounter = m_accessCounter;

  if (m_files.size() > 4) {
    auto oldestAccess = m_accessCounter;
    auto oldestEntry = m_files.end();

    for (auto iter = m_files.begin(); iter != m_files.end(); ++iter) {
      if (iter->second.m_accessCounter < oldestAccess) {
        oldestEntry = iter;
        oldestAccess = iter->second.m_accessCounter;
      }
    }

    if (oldestEntry != m_files.end()) {
      m_files.erase(oldestEntry);
    }
  }

  return iter->second;
}

double Elevation::getElevation(const LatLng &point) {
  try {
    static Profiler load("load");
    static Profiler compute("compute");

    auto &file = loadFile(point);

    return file.getElevationAtPoint(point);
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }

  return 0;
}

Terrain Elevation::getElevationArea(const LatLng &point, int dimensions) {
  try {
    auto &file = loadFile(point);

    std::vector<std::vector<uint16_t>> points;
    std::vector<std::vector<double>> centers;

    int row{}, col{};

    std::tie(row, col) = file.getFilePosition(point);

    row -= dimensions / 2;
    col -= dimensions / 2;

    points.resize(dimensions + 1);

    for (int j = 0; j < dimensions + 1; j++) {
      for (int i = 0; i < dimensions + 1; i++) {
        points[j].push_back(
            swapBytes(file.m_buffer[(row + j) * 3601 + col + i]));
      }
    }

    centers.resize(points.size() - 1);

    for (size_t j = 0; j < points.size() - 1; j++) {
      for (size_t i = 0; i < points[j].size() - 1; i++) {
        centers[j].push_back(bilinearInterpolation(
            points[j][i], points[j + 1][i], points[j][i + 1],
            points[j + 1][i + 1], 0.5, 0.5));
      }
    }

    Terrain terrain;

    terrain.sw =
        LatLng(std::floor(point.m_lat) + (3600 - (row + dimensions)) / 3600.0,
               std::floor(point.m_lng) + col / 3600.0);

    terrain.ne = LatLng(std::floor(point.m_lat) + (3600 - row) / 3600.0,
                        std::floor(point.m_lng) + (col + dimensions) / 3600.0);

    terrain.points = points;
    terrain.centers = centers;

    return terrain;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }

  return {};
}

double tile2lng(int x, int z) { return x / std::pow(2, z) * 360 - 180; }

double tile2lat(int y, int z) {
  double PI = 3.14159265359;
  double n = PI - 2 * PI * y / std::pow(2, z);
  return (180 / PI * std::atan(0.5 * (std::exp(n) - std::exp(-n))));
}

Terrain Elevation::getElevationTile(int x, int y, int z) {
  try {
    LatLng nw(tile2lat(y, z), tile2lng(x, z));
    LatLng se(tile2lat(y + 1, z), tile2lng(x + 1, z));

    auto &file = loadFile(nw);

    std::vector<std::vector<uint16_t>> points;
    std::vector<std::vector<double>> centers;

    int northRow{}, westCol{};
    int southRow{}, eastCol{};

    std::tie(northRow, westCol) = file.getFilePosition(nw);
    std::tie(southRow, eastCol) = file.getFilePosition(se);

    // Expand the area by one from ne corner
    // getFIlePosition will provide the row and col
    // that is south and to the west of the latlng.
    // So, we need to expand to the north and to the east.
    northRow -= 1;
    eastCol += 1;

    int dimensionX = eastCol - westCol;
    int dimensionY = southRow - northRow;

    points.resize(dimensionY + 1);

    for (int j = 0; j < dimensionY + 1; j++) {
      for (int i = 0; i < dimensionX + 1; i++) {
        points[j].push_back(
            swapBytes(file.m_buffer[(southRow - j) * 3601 + westCol + i]));
      }
    }

    centers.resize(points.size() - 1);

    for (size_t j = 0; j < points.size() - 1; j++) {
      for (size_t i = 0; i < points[j].size() - 1; i++) {
        centers[j].push_back(bilinearInterpolation(
            points[j][i], points[j + 1][i], points[j][i + 1],
            points[j + 1][i + 1], 0.5, 0.5));
      }
    }

    Terrain terrain;

    terrain.sw = file.pointToLatLng(southRow, westCol);
    terrain.ne = file.pointToLatLng(northRow, eastCol);

    auto tileHeight = terrain.ne.m_lat - terrain.sw.m_lat;
    auto tileWidth = se.m_lng - nw.m_lng;

    terrain.textureNE.s = (terrain.ne.m_lng - nw.m_lng) / tileWidth;
    terrain.textureSW.s = (terrain.sw.m_lng - nw.m_lng) / tileWidth;
    terrain.textureNE.t = (terrain.sw.m_lat - se.m_lat) / tileHeight;
    terrain.textureSW.t = (terrain.ne.m_lat - se.m_lat) / tileHeight;

    terrain.points = points;
    terrain.centers = centers;

    return terrain;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }

  return {};
}

std::vector<Json::Value> Elevation::getRoutes(const LatLng &sw,
                                              const LatLng &ne) {
  auto dbConnection = std::make_shared<DBConnection>();

  PreparedStatement getRoutes(
      dbConnection,
      "select ST_AsGeoJSON(ST_Transform(ST_Intersection( "
      "  ST_SetSRID(ST_MakeBox2D( "
      "    ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857), "
      "    ST_Transform(ST_SetSRID(ST_MakePoint($3, $4), 4326), 3857) "
      "  ), 3857), "
      "  way2), 4326)) AS linestring "
      "from planet_osm_route "
      "where ST_Intersects( "
      "  ST_SetSRID(ST_MakeBox2D( "
      "    ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857), "
      "    ST_Transform(ST_SetSRID(ST_MakePoint($3, $4), 4326), 3857) "
      "  ), 3857), "
      "  way) ");

  Json::Reader reader;

  auto rows = getRoutes.exec(sw.m_lng, sw.m_lat, ne.m_lng, ne.m_lat);

  std::vector<Json::Value> lineStrings;

  for (const auto &row : rows) {
    Json::Value lineString;
    reader.parse(row["linestring"].as<std::string>(), lineString);

    // for (auto &coord: lineString["coordinates"])
    // {
    //   if (lineString["type"] == "LineString")
    //   {
    //     coord[2] = getElevationAtPoint(buffer, {coord[1].asDouble(),
    //     coord[0].asDouble()});
    //   }
    // }

    lineStrings.push_back(lineString["coordinates"]);
  }

  return lineStrings;
}

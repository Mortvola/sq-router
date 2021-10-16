#include "ElevationFile.h"
#include <cmath>

LatLng normalize(const LatLng &point) {
  auto x = point.m_lng - std::floor(point.m_lng);
  auto y = 1 - (point.m_lat - std::floor(point.m_lat));

  return {(y - (std::floor(y * 3600) / 3600)) * 3600,
          (x - (std::floor(x * 3600) / 3600)) * 3600};
}

double bilinearInterpolation(double q11, double q12, double q21, double q22,
                             double x, double y) {
  double x2x = 1 - x;
  double y2y = 1 - y;
  return (q11 * x2x * y2y + q12 * x * y2y + q21 * x2x * y + q22 * x * y);
}

double interpolateElevation(const std::array<uint16_t, 4> &ele,
                            const LatLng &point) {
  auto p = normalize(point);

  auto elevation =
      bilinearInterpolation(ele[2], ele[3], ele[0], ele[1], p.m_lng, p.m_lat);

  return elevation;
}

LatLng ElevationFile::pointToLatLng(int row, int col) const {
  return LatLng(m_latLng.m_lat + 1 - row / 3600.0,
                m_latLng.m_lng + col / 3600.0);
}

// Returns the row and col of the lower left corner relative to the point
std::tuple<int, int> ElevationFile::getFilePosition(const LatLng &point) const {
  // Determine file name
  // int latInt = std::floor(point.m_lat);
  // int lngInt = std::floor(point.m_lng);
  int row{};
  int col{};

  row = 3600 - std::floor((point.m_lat - m_latLng.m_lat) * 3600);
  col = std::floor((point.m_lng - m_latLng.m_lng) * 3600);

  return std::tuple<int, int>(row, col);
}

double ElevationFile::getElevationAtPoint(const LatLng &point) const {
  std::array<uint16_t, 4> ele;
  int row{}, col{};

  std::tie(row, col) = getFilePosition(point);

  // read the lower left
  ele[0] = swapBytes(m_buffer[row * 3601 + col]);

  // read the lower right
  ele[1] = swapBytes(m_buffer[row * 3601 + col + 1]);

  // Read the upper left elevation
  ele[2] = swapBytes(m_buffer[(row - 1) * 3601 + col]);

  // Read the upper right elevation;
  ele[3] = swapBytes(m_buffer[(row - 1) * 3601 + col + 1]);

  return interpolateElevation(ele, point);
}

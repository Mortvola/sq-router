#pragma once

#include "LatLng.h"

class ElevationFile {
public:
  LatLng pointToLatLng(int row, int col) const;

  // Returns the row and col of the lower left corner relative to the point
  std::tuple<int, int> getFilePosition(const LatLng &point) const;

  double getElevationAtPoint(const LatLng &point) const;

  LatLng m_latLng;
  std::vector<uint16_t> m_buffer;
  uint64_t m_accessCounter{0};

private:
};

double bilinearInterpolation(double q11, double q12, double q21, double q22,
                             double x, double y);

inline uint16_t swapBytes(uint16_t value) {
  return ((value & 0xff) << 8) | ((value >> 8) & 0x00ff);
}


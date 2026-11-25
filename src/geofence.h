#pragma once
// Geofence maths in plain C++ (no Arduino), so it can be unit-tested on a PC.
#include <math.h>
#include <stdio.h>

// Distance in metres between two points (haversine formula).
inline double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double toRad = 3.14159265358979323846 / 180;
  double dLat = (lat2 - lat1) * toRad, dLon = (lon2 - lon1) * toRad;
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1 * toRad) * cos(lat2 * toRad) * sin(dLon / 2) * sin(dLon / 2);
  return 2 * 6371000.0 * asin(sqrt(fmin(1.0, a)));
}

// Parses "lat,lon,radius" (degrees, degrees, metres). Returns false if invalid.
inline bool parseFence(const char* text, double& lat, double& lon, double& radius) {
  char extra;
  return sscanf(text, "%lf,%lf,%lf %c", &lat, &lon, &radius, &extra) == 3 &&
         fabs(lat) <= 90 && fabs(lon) <= 180 && radius > 0;
}

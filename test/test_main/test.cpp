// Runs on your PC:  pio test -e native
#include <unity.h>
#include "geofence.h"

void setUp() {}
void tearDown() {}

void test_distance_same_point_is_zero() {
  TEST_ASSERT_FLOAT_WITHIN(0.01, 0, distanceMeters(48.1173, 11.5167, 48.1173, 11.5167));
}

void test_distance_one_degree_of_latitude() {
  TEST_ASSERT_FLOAT_WITHIN(1, 111195, distanceMeters(0, 0, 1, 0));   // ~111.2 km
}

void test_distance_paris_to_london() {
  TEST_ASSERT_FLOAT_WITHIN(1000, 343500, distanceMeters(48.8566, 2.3522, 51.5074, -0.1278));
}

void test_parseFence_valid() {
  double lat, lon, radius;
  TEST_ASSERT_TRUE(parseFence("51.5007,-0.1246,150", lat, lon, radius));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 51.5007, lat);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, -0.1246, lon);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 150, radius);
  TEST_ASSERT_TRUE(parseFence("51.5,-0.12,150\r\n", lat, lon, radius));   // trailing newline OK
}


// Inside/outside is "distance <= radius": a point ~185 m from the centre.
void test_inside_and_outside() {
  double distance = distanceMeters(48.1173, 11.516667, 48.1173 + 0.1 / 60, 11.516667);
  TEST_ASSERT_FLOAT_WITHIN(1, 185.3, distance);
  TEST_ASSERT_FALSE(distance <= 100);
  TEST_ASSERT_TRUE(distance <= 200);
}


int main() {
  UNITY_BEGIN();
  RUN_TEST(test_distance_same_point_is_zero);
  RUN_TEST(test_distance_one_degree_of_latitude);
  RUN_TEST(test_distance_paris_to_london);
  RUN_TEST(test_parseFence_valid);
  RUN_TEST(test_inside_and_outside);

  return UNITY_END();
}

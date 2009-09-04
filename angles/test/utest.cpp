#include "angles/angles.h"
#include <gtest/gtest.h>

using namespace angles;

TEST(Angles, shortestDistanceWithLimits){
  double shortest_angle;
  bool result = angles::shortest_angular_distance_with_limits(-0.5, 0.5,-0.25,0.25,shortest_angle);
  EXPECT_TRUE(!result);

  result = angles::shortest_angular_distance_with_limits(-0.5, 0.5,0.25,0.25,shortest_angle);
  EXPECT_TRUE(!result);

  result = angles::shortest_angular_distance_with_limits(-0.5, 0.5,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(result);
  EXPECT_NEAR(shortest_angle, -2*M_PI+1.0,1e-6);

  result = angles::shortest_angular_distance_with_limits(0.5, 0.5,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(result);
  EXPECT_NEAR(shortest_angle, 0,1e-6);

  result = angles::shortest_angular_distance_with_limits(0.5, 0,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(!result);
  EXPECT_NEAR(shortest_angle, -0.5,1e-6);

  result = angles::shortest_angular_distance_with_limits(-0.5, 0,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(!result);
  EXPECT_NEAR(shortest_angle, 0.5,1e-6);

  result = angles::shortest_angular_distance_with_limits(-0.2,0.2,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(!result);
  EXPECT_NEAR(shortest_angle, -2*M_PI+0.4,1e-6);

  result = angles::shortest_angular_distance_with_limits(0.2,-0.2,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(!result);
  EXPECT_NEAR(shortest_angle,2*M_PI-0.4,1e-6);

  result = angles::shortest_angular_distance_with_limits(0.2,0,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(!result);
  EXPECT_NEAR(shortest_angle,2*M_PI-0.2,1e-6);

  result = angles::shortest_angular_distance_with_limits(-0.2,0,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(!result);
  EXPECT_NEAR(shortest_angle,-2*M_PI+0.2,1e-6);

  result = angles::shortest_angular_distance_with_limits(-0.25,-0.5,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(result);
  EXPECT_NEAR(shortest_angle,-0.25,1e-6);

  result = angles::shortest_angular_distance_with_limits(-0.25,0.5,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(result);
  EXPECT_NEAR(shortest_angle,-2*M_PI+0.75,1e-6);

  result = angles::shortest_angular_distance_with_limits(-0.2500001,0.5,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(result);
  EXPECT_NEAR(shortest_angle,-2*M_PI+0.5+0.2500001,1e-6);

}

TEST(Angles, from_degrees)
{
  double epsilon = 1e-9;
  EXPECT_NEAR(0, from_degrees(0), epsilon);
  EXPECT_NEAR(M_PI/2, from_degrees(90), epsilon);
  EXPECT_NEAR(M_PI, from_degrees(180), epsilon);
  EXPECT_NEAR(M_PI*3/2, from_degrees(270), epsilon);
  EXPECT_NEAR(2*M_PI, from_degrees(360), epsilon);
  EXPECT_NEAR(M_PI/3, from_degrees(60), epsilon);
  EXPECT_NEAR(M_PI*2/3, from_degrees(120), epsilon);
  EXPECT_NEAR(M_PI/4, from_degrees(45), epsilon);
  EXPECT_NEAR(M_PI*3/4, from_degrees(135), epsilon);
  EXPECT_NEAR(M_PI/6, from_degrees(30), epsilon);
  
}

TEST(Angles, to_degrees)
{
  double epsilon = 1e-9;
  EXPECT_NEAR(to_degrees(0), 0, epsilon);
  EXPECT_NEAR(to_degrees(M_PI/2), 90, epsilon);
  EXPECT_NEAR(to_degrees(M_PI), 180, epsilon);
  EXPECT_NEAR(to_degrees(M_PI*3/2), 270, epsilon);
  EXPECT_NEAR(to_degrees(2*M_PI), 360, epsilon);
  EXPECT_NEAR(to_degrees(M_PI/3), 60, epsilon);
  EXPECT_NEAR(to_degrees(M_PI*2/3), 120, epsilon);
  EXPECT_NEAR(to_degrees(M_PI/4), 45, epsilon);
  EXPECT_NEAR(to_degrees(M_PI*3/4), 135, epsilon);
  EXPECT_NEAR(to_degrees(M_PI/6), 30, epsilon);
}

TEST(Angles, normalize_angle_positive)
{
 double epsilon = 1e-9;
 EXPECT_NEAR(0, normalize_angle_positive(0), epsilon);
 EXPECT_NEAR(M_PI, normalize_angle_positive(M_PI), epsilon);
 EXPECT_NEAR(0, normalize_angle_positive(2*M_PI), epsilon);
 EXPECT_NEAR(M_PI, normalize_angle_positive(3*M_PI), epsilon);
 EXPECT_NEAR(0, normalize_angle_positive(4*M_PI), epsilon);

 EXPECT_NEAR(0, normalize_angle_positive(-0), epsilon);
 EXPECT_NEAR(M_PI, normalize_angle_positive(-M_PI), epsilon);
 EXPECT_NEAR(0, normalize_angle_positive(-2*M_PI), epsilon);
 EXPECT_NEAR(M_PI, normalize_angle_positive(-3*M_PI), epsilon);
 EXPECT_NEAR(0, normalize_angle_positive(-4*M_PI), epsilon);

 EXPECT_NEAR(0, normalize_angle_positive(-0), epsilon);
 EXPECT_NEAR(3*M_PI/2, normalize_angle_positive(-M_PI/2), epsilon);
 EXPECT_NEAR(M_PI, normalize_angle_positive(-M_PI), epsilon);
 EXPECT_NEAR(M_PI/2, normalize_angle_positive(-3*M_PI/2), epsilon);
 EXPECT_NEAR(0, normalize_angle_positive(-4*M_PI/2), epsilon);

 EXPECT_NEAR(0, normalize_angle_positive(0), epsilon);
 EXPECT_NEAR(M_PI/2, normalize_angle_positive(M_PI/2), epsilon);
 EXPECT_NEAR(M_PI/2, normalize_angle_positive(5*M_PI/2), epsilon);
 EXPECT_NEAR(M_PI/2, normalize_angle_positive(9*M_PI/2), epsilon);
 EXPECT_NEAR(M_PI/2, normalize_angle_positive(-3*M_PI/2), epsilon);

}


TEST(Angles, normalize_angle)
{
 double epsilon = 1e-9;
 EXPECT_NEAR(0, normalize_angle(0), epsilon);
 EXPECT_NEAR(M_PI, normalize_angle(M_PI), epsilon);
 EXPECT_NEAR(0, normalize_angle(2*M_PI), epsilon);
 EXPECT_NEAR(M_PI, normalize_angle(3*M_PI), epsilon);
 EXPECT_NEAR(0, normalize_angle(4*M_PI), epsilon);

 EXPECT_NEAR(0, normalize_angle(-0), epsilon);
 EXPECT_NEAR(M_PI, normalize_angle(-M_PI), epsilon);
 EXPECT_NEAR(0, normalize_angle(-2*M_PI), epsilon);
 EXPECT_NEAR(M_PI, normalize_angle(-3*M_PI), epsilon);
 EXPECT_NEAR(0, normalize_angle(-4*M_PI), epsilon);

 EXPECT_NEAR(0, normalize_angle(-0), epsilon);
 EXPECT_NEAR(-M_PI/2, normalize_angle(-M_PI/2), epsilon);
 EXPECT_NEAR(M_PI, normalize_angle(-M_PI), epsilon);
 EXPECT_NEAR(M_PI/2, normalize_angle(-3*M_PI/2), epsilon);
 EXPECT_NEAR(0, normalize_angle(-4*M_PI/2), epsilon);

 EXPECT_NEAR(0, normalize_angle(0), epsilon);
 EXPECT_NEAR(M_PI/2, normalize_angle(M_PI/2), epsilon);
 EXPECT_NEAR(M_PI/2, normalize_angle(5*M_PI/2), epsilon);
 EXPECT_NEAR(M_PI/2, normalize_angle(9*M_PI/2), epsilon);
 EXPECT_NEAR(M_PI/2, normalize_angle(-3*M_PI/2), epsilon);

}

TEST(Angles, shortest_angular_distance)
{
  double epsilon = 1e-9;
  EXPECT_NEAR(M_PI/2, shortest_angular_distance(0, M_PI/2), epsilon);
  EXPECT_NEAR(-M_PI/2, shortest_angular_distance(0, -M_PI/2), epsilon);
  EXPECT_NEAR(-M_PI/2, shortest_angular_distance(M_PI/2, 0), epsilon);
  EXPECT_NEAR(M_PI/2, shortest_angular_distance(-M_PI/2, 0), epsilon);

  EXPECT_NEAR(-M_PI/2, shortest_angular_distance(M_PI, M_PI/2), epsilon);
  EXPECT_NEAR(M_PI/2, shortest_angular_distance(M_PI, -M_PI/2), epsilon);
  EXPECT_NEAR(M_PI/2, shortest_angular_distance(M_PI/2, M_PI), epsilon);
  EXPECT_NEAR(-M_PI/2, shortest_angular_distance(-M_PI/2, M_PI), epsilon);

  EXPECT_NEAR(-M_PI/2, shortest_angular_distance(5*M_PI, M_PI/2), epsilon);
  EXPECT_NEAR(M_PI/2, shortest_angular_distance(7*M_PI, -M_PI/2), epsilon);
  EXPECT_NEAR(M_PI/2, shortest_angular_distance(9*M_PI/2, M_PI), epsilon);
  EXPECT_NEAR(M_PI/2, shortest_angular_distance(-3*M_PI/2, M_PI), epsilon);

}

TEST(Angles, two_pi_complement)
{
  double epsilon = 1e-9;
  EXPECT_NEAR(two_pi_complement(0), -2*M_PI, epsilon); //Should this be?
  EXPECT_NEAR(two_pi_complement(2*M_PI), 0, epsilon); 
  EXPECT_NEAR(two_pi_complement(-2*M_PI), 0, epsilon); 
  EXPECT_NEAR(two_pi_complement(2*M_PI-epsilon), -epsilon, epsilon); 
  EXPECT_NEAR(two_pi_complement(-2*M_PI+epsilon), epsilon, epsilon); 
  EXPECT_NEAR(two_pi_complement(M_PI/2), -3*M_PI/2, epsilon);
  EXPECT_NEAR(two_pi_complement(M_PI), -M_PI, epsilon);
  EXPECT_NEAR(two_pi_complement(-M_PI), M_PI, epsilon);
  EXPECT_NEAR(two_pi_complement(-M_PI/2), 3*M_PI/2, epsilon);

  EXPECT_NEAR(two_pi_complement(3*M_PI), -M_PI, epsilon); 
  EXPECT_NEAR(two_pi_complement(-3.0*M_PI), M_PI, epsilon);
  EXPECT_NEAR(two_pi_complement(-5.0*M_PI/2.0), 3*M_PI/2, epsilon);



}

/** \todo find_min_max_delta is always returning false commented until fixed */
TEST(Angles, find_min_max_delta)
{
  double epsilon = 1e-9;
  double min_delta, max_delta;
  EXPECT_TRUE(find_min_max_delta( 0, -M_PI, M_PI, min_delta, max_delta));
  EXPECT_NEAR(min_delta, -M_PI, epsilon);
  EXPECT_NEAR(max_delta, M_PI, epsilon);

  EXPECT_TRUE(find_min_max_delta( M_PI/2, -M_PI, M_PI, min_delta, max_delta));
  EXPECT_NEAR(min_delta, -3*M_PI/2, epsilon);
  EXPECT_NEAR(max_delta, M_PI/2, epsilon);

  EXPECT_TRUE(find_min_max_delta( M_PI/2, -M_PI, M_PI, min_delta, max_delta));
  EXPECT_NEAR(min_delta, -3*M_PI/2, epsilon);
  EXPECT_NEAR(max_delta, M_PI/2, epsilon);

  EXPECT_TRUE(find_min_max_delta( -M_PI/2, -M_PI, M_PI, min_delta, max_delta));
  EXPECT_NEAR(min_delta, -M_PI/2, epsilon);
  EXPECT_NEAR(max_delta, 3*M_PI/2, epsilon);

  
  //Test out of range
  EXPECT_FALSE(find_min_max_delta( -M_PI, -M_PI/2, M_PI/2, min_delta, max_delta));

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

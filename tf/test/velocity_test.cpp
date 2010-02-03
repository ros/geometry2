#include <gtest/gtest.h>
#include <tf/tf.h>
#include <sys/time.h>

#include "LinearMath/btVector3.h"

using namespace tf;


// The fixture for testing class Foo.
class LinearVelocitySquareTest : public ::testing::Test {
protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  LinearVelocitySquareTest() {
    double x = 0;
    double y = 0;
    double z = 0;
    for (double t = 0; t < 6; t += 0.1)
    {
      if      (t < 1) x += .1;
      else if (t < 2) y += .1;
      else if (t < 3) x -= .1;
      else if (t < 4) y -= .1;
      else if (t < 5) z += .1;
      else            z -= .1;
      tf_.setTransform(StampedTransform(btTransform(tf::createIdentityQuaternion(), btVector3(x, y, z)), ros::Time(t), "foo", "bar"));
    }

    // You can do set-up work for each test here.
  }

  virtual ~LinearVelocitySquareTest() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for LinearVelocity.

  tf::Transformer tf_;

};

TEST_F(LinearVelocitySquareTest, lvt)
{
  geometry_msgs::TwistStamped tw;
  try
  {
    tf_.lookupVelocity("foo", "bar", ros::Time(0.5), ros::Duration(0.1), tw);
  EXPECT_FLOAT_EQ(tw.twist.linear.x, 1.0);
  EXPECT_FLOAT_EQ(tw.twist.linear.y, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.linear.z, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.x, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.y, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.z, 0.0);
  
  tf_.lookupVelocity("foo", "bar", ros::Time(1.5), ros::Duration(0.1), tw);
  EXPECT_FLOAT_EQ(tw.twist.linear.x, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.linear.y, 1.0);
  EXPECT_FLOAT_EQ(tw.twist.linear.z, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.x, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.y, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.z, 0.0);

  tf_.lookupVelocity("foo", "bar", ros::Time(2.5), ros::Duration(0.1), tw);
  EXPECT_FLOAT_EQ(tw.twist.linear.x, -1.0);
  EXPECT_FLOAT_EQ(tw.twist.linear.y, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.linear.z, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.x, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.y, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.z, 0.0);

  tf_.lookupVelocity("foo", "bar", ros::Time(3.5), ros::Duration(0.1), tw);
  EXPECT_FLOAT_EQ(tw.twist.linear.x, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.linear.y, -1.0);
  EXPECT_FLOAT_EQ(tw.twist.linear.z, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.x, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.y, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.z, 0.0);

  tf_.lookupVelocity("foo", "bar", ros::Time(4.5), ros::Duration(0.1), tw);
  EXPECT_FLOAT_EQ(tw.twist.linear.x, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.linear.y, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.linear.z, 1.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.x, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.y, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.z, 0.0);

  tf_.lookupVelocity("foo", "bar", ros::Time(5.5), ros::Duration(0.1), tw);
  EXPECT_FLOAT_EQ(tw.twist.linear.x, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.linear.y, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.linear.z, -1.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.x, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.y, 0.0);
  EXPECT_FLOAT_EQ(tw.twist.angular.z, 0.0);
  }
  catch(tf::TransformException &ex)
  {
    ASSERT_FALSE(ex.what());
  }
};




int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


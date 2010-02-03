#include <gtest/gtest.h>
#include <tf/tf.h>
#include <sys/time.h>

#include "LinearMath/btVector3.h"

using namespace tf;


// The fixture for testing class Foo.
class LinearVelocityTest : public ::testing::Test {
protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  LinearVelocityTest() {
    double x = 0;
    double y = 0;
    for (double t = 0; t < 10; t += 0.1)
    {
      if ( t < 2.5) x += .1;
      else if (t < 5) y += .1;
      else if (t < 7.5) x -= .1;
      else y -= .1;
      tf_.setTransform(StampedTransform(btTransform(tf::createIdentityQuaternion(), btVector3(x, y, 0)), ros::Time(t), "foo", "bar"));
    }

    // You can do set-up work for each test here.
  }

  virtual ~LinearVelocityTest() {
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

TEST_F(LinearVelocityTest, lvt)
{
  geometry_msgs::TwistStamped tw;
  tf_.lookupVelocity("foo", "bar", ros::Time(1.25), ros::Duration(0.1), tw);
  EXPECT_FLOAT_EQ(tw.twist.linear.x, 1.0);
  EXPECT_FLOAT_EQ(tw.twist.linear.y, 0.0);
  
};




int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


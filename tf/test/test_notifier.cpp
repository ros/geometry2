/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Josh Faust */

#include <tf/message_notifier.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>

#include "ros/node.h"

#include <gtest/gtest.h>

using namespace tf;

ros::Node* g_node = NULL;
TransformListener* g_tf = NULL;
TransformBroadcaster* g_broadcaster = NULL;

class Notification
{
public:
	Notification(int expected_count)
	: count_(0)
	, expected_count_(expected_count)
	{
	  lock_ = new boost::timed_mutex::scoped_lock(mutex_);
	}

	~Notification()
	{
		if (count_ < expected_count_)
		{
		  boost::timed_mutex::scoped_lock* lock = lock_;
		  lock_ = 0;
		  delete lock;
		}
	}

	void notify(const MessageNotifier<geometry_msgs::PointStamped>::MessagePtr& message)
	{
		++count_;

		//printf("Notify: %d\n", count_);

		if (count_ == expected_count_)
		{
		  boost::timed_mutex::scoped_lock* lock = lock_;
      lock_ = 0;
      delete lock;
		}
	}


  boost::timed_mutex& getMutex()
  {
    if (count_ == expected_count_) //deal with zero length
    {
      boost::timed_mutex::scoped_lock* lock = lock_;
      lock_ = 0;
      delete lock;
    }
    return mutex_;
  };
	int count_;
	int expected_count_;

	boost::timed_mutex::scoped_lock* lock_;
	boost::timed_mutex mutex_;
};

template<class T>
class Counter
{
public:
	Counter(const std::string& topic, int expected_count)
	: count_(0)
	, expected_count_(expected_count)
	, topic_(topic)
	{
	  lock_ = new boost::timed_mutex::scoped_lock(mutex_);

		g_node->subscribe(topic_, message_, &Counter::callback, this, 0);
	}

	~Counter()
	{
		g_node->unsubscribe(topic_, &Counter::callback, this);

		if (count_ < expected_count_)
		{
		  boost::timed_mutex::scoped_lock* lock = lock_;
      lock_ = 0;
      delete lock;
		}
	}

	void callback()
	{
		++count_;

		//printf("Counter: %d\n", count_);

		if (count_ == expected_count_)
		{
		  boost::timed_mutex::scoped_lock* lock = lock_;
      lock_ = 0;
      delete lock;
		}
	}

  boost::timed_mutex& getMutex()
  {
    if (count_ == expected_count_) //deal with zero length
    {
      boost::timed_mutex::scoped_lock* lock = lock_;
      lock_ = 0;
      delete lock;
    }
    return mutex_;
  };

	T message_;

	int count_;
	int expected_count_;
	std::string topic_;

	boost::timed_mutex::scoped_lock* lock_;
	boost::timed_mutex mutex_;
};

TEST(MessageNotifier, noTransforms)
{
	Notification n(1);
	MessageNotifier<geometry_msgs::PointStamped>* notifier = new MessageNotifier<geometry_msgs::PointStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n, _1), "test_message", "frame1", 1);
	std::auto_ptr<MessageNotifier<geometry_msgs::PointStamped> > notifier_ptr(notifier);

	Counter<geometry_msgs::PointStamped> c("test_message", 1);

	ros::Duration().fromSec(0.2).sleep();

	geometry_msgs::PointStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "frame2";
	g_node->publish("test_message", msg);

	ros::Duration().fromSec(0.2).sleep();

	EXPECT_EQ(0, n.count_);
}

TEST(MessageNotifier, preexistingTransforms)
{
	Notification n(1);
	Counter<tf::tfMessage> c("tf_message", 1);
	MessageNotifier<geometry_msgs::PointStamped>* notifier = new MessageNotifier<geometry_msgs::PointStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n, _1), "test_message", "frame1", 1);
	std::auto_ptr<MessageNotifier<geometry_msgs::PointStamped> > notifier_ptr(notifier);

	ros::Duration().fromSec(0.2).sleep();

	ros::Time stamp = ros::Time::now();

	g_broadcaster->sendTransform(btTransform(btQuaternion(0,0,0), btVector3(1,2,3)), stamp, "frame1", "frame2");

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(c.mutex_, xt);

		EXPECT_EQ(true, lock.owns_lock());
	}

	geometry_msgs::PointStamped msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = "frame2";
	g_node->publish("test_message", msg);

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(n.mutex_, xt);

		EXPECT_EQ(true, lock.owns_lock());
	}

	EXPECT_EQ(1, n.count_);
}

TEST(MessageNotifier, postTransforms)
{
	Notification n(1);
	MessageNotifier<geometry_msgs::PointStamped>* notifier = new MessageNotifier<geometry_msgs::PointStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n, _1), "test_message", "frame3", 1);
	std::auto_ptr<MessageNotifier<geometry_msgs::PointStamped> > notifier_ptr(notifier);

	Counter<geometry_msgs::PointStamped> c("test_message", 1);

	ros::Duration().fromSec(0.2).sleep();

	ros::Time stamp = ros::Time::now();

	geometry_msgs::PointStamped msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = "frame4";
	g_node->publish("test_message", msg);

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(c.mutex_, xt);

		EXPECT_EQ(true, lock.owns_lock());
	}

	g_broadcaster->sendTransform(btTransform(btQuaternion(0,0,0), btVector3(1,2,3)), stamp, "frame3", "frame4");

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(n.mutex_, xt);

		EXPECT_EQ(true, lock.owns_lock());
	}

	EXPECT_EQ(1, n.count_);
}

TEST(MessageNotifier, queueSize)
{
	Notification n(10);
	MessageNotifier<geometry_msgs::PointStamped>* notifier = new MessageNotifier<geometry_msgs::PointStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n, _1), "test_message", "frame5", 10);
	std::auto_ptr<MessageNotifier<geometry_msgs::PointStamped> > notifier_ptr(notifier);

	Counter<geometry_msgs::PointStamped> c("test_message", 20);

	ros::Duration().fromSec(0.5).sleep();

	ros::Time stamp = ros::Time::now();

	for (int i = 0; i < 20; ++i)
	{
		geometry_msgs::PointStamped msg;
		msg.header.stamp = stamp;
		msg.header.frame_id = "frame6";
		g_node->publish("test_message", msg);

		//ros::Duration().fromSec(0.01).sleep();
	}

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(c.mutex_, xt);

		EXPECT_EQ(true, lock.owns_lock());
	}

	ros::Duration().fromSec(0.1).sleep();

	g_broadcaster->sendTransform(btTransform(btQuaternion(0,0,0), btVector3(1,2,3)), stamp, "frame5", "frame6");

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(n.mutex_, xt);

		EXPECT_EQ(true, lock.owns_lock());
	}

	EXPECT_EQ(10, n.count_);
}

TEST(MessageNotifier, setTopic)
{
	Notification n(1);
	Counter<tf::tfMessage> c("tf_message", 1);
	MessageNotifier<geometry_msgs::PointStamped>* notifier = new MessageNotifier<geometry_msgs::PointStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n, _1), "test_message", "frame7", 1);
	std::auto_ptr<MessageNotifier<geometry_msgs::PointStamped> > notifier_ptr(notifier);
	notifier->setTopic("test_message2");

	ros::Duration().fromSec(0.2).sleep();

	ros::Time stamp = ros::Time::now();

	g_broadcaster->sendTransform(btTransform(btQuaternion(0,0,0), btVector3(1,2,3)), stamp, "frame7", "frame8");

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(c.mutex_, xt);

		EXPECT_EQ(true, lock.owns_lock());
	}

	geometry_msgs::PointStamped msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = "frame8";
	g_node->publish("test_message2", msg);

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(n.mutex_, xt);

		EXPECT_EQ(true, lock.owns_lock());
	}

	EXPECT_EQ(1, n.count_);
}

TEST(MessageNotifier, setTargetFrame)
{
	Notification n(1);
	Counter<tf::tfMessage> c("tf_message", 1); /// \todo Switch this to tf_message once rosTF goes away completely
	MessageNotifier<geometry_msgs::PointStamped>* notifier = new MessageNotifier<geometry_msgs::PointStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n, _1), "test_message", "frame9", 1);
	std::auto_ptr<MessageNotifier<geometry_msgs::PointStamped> > notifier_ptr(notifier);
	notifier->setTargetFrame("frame1000");

	ros::Duration().fromSec(0.2).sleep();

	ros::Time stamp = ros::Time::now();

	g_broadcaster->sendTransform(btTransform(btQuaternion(0,0,0), btVector3(1,2,3)), stamp, "frame1000", "frame10");

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(c.mutex_, xt);

		EXPECT_EQ(true, lock.owns_lock());
	}

	geometry_msgs::PointStamped msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = "frame10";
	g_node->publish("test_message", msg);

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(n.mutex_, xt);

		EXPECT_EQ(true, lock.owns_lock());
	}

	EXPECT_EQ(1, n.count_);
}

TEST(MessageNotifier, setMultipleTargetFrame)
{
	Notification n(1);
	Notification n2(1);
	Counter<tf::tfMessage> c("tf_message", 1); /// \todo Switch this to tf_message once rosTF goes away completely
	MessageNotifier<geometry_msgs::PointStamped>* notifier = new MessageNotifier<geometry_msgs::PointStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n, _1), "test_message", "frame9", 1);
	MessageNotifier<geometry_msgs::PointStamped>* notifier2 = new MessageNotifier<geometry_msgs::PointStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n2, _1), "test_message", "frame9", 1);
	std::auto_ptr<MessageNotifier<geometry_msgs::PointStamped> > notifier_ptr(notifier);
	std::auto_ptr<MessageNotifier<geometry_msgs::PointStamped> > notifier_ptr2(notifier2);
        std::vector<std::string> target_frames;
        target_frames.resize(2);
        target_frames[0] = "frame1000";
        target_frames[1] = "frame10";
	notifier->setTargetFrame(target_frames);

        target_frames[0] = "frame1000";
        target_frames[1] = "frame10000";
	notifier2->setTargetFrame(target_frames);


	ros::Duration().fromSec(0.2).sleep();

	ros::Time stamp = ros::Time::now();

	g_broadcaster->sendTransform(btTransform(btQuaternion(0,0,0), btVector3(1,2,3)), stamp, "frame1000", "frame10");

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(c.mutex_, xt);

		EXPECT_EQ(true, lock.owns_lock());
	}

	geometry_msgs::PointStamped msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = "frame10";
	g_node->publish("test_message", msg);

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(n.mutex_, xt);

		EXPECT_EQ(true, lock.owns_lock());
	}

	EXPECT_EQ(1, n.count_);
	EXPECT_EQ(0, n2.count_); //n2 is waiting for 10000 to be available

	g_broadcaster->sendTransform(btTransform(btQuaternion(0,0,0), btVector3(1,2,3)), stamp, "frame10000", "frame1000");
	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(n2.mutex_, xt);
                printf("HHHHHHHHHHHHHHHEEEEEEEEEEEEEEEEEEEERRRRRRRRRRRRRRRRREEEEEEEEEEEEEEEEEEE %s\n", notifier2->getTargetFramesString().c_str());
		EXPECT_EQ(true, lock.owns_lock());
	}

	EXPECT_EQ(1, n2.count_);

}


TEST(MessageNotifier, setTolerance)
{
  ros::Duration offset(0.2);
	Notification n(0);
	Notification n1(1);
	Notification n2(1);
	Counter<tf::tfMessage> c("tf_message", 1);
	boost::scoped_ptr<MessageNotifier<geometry_msgs::PointStamped> > notifier(new MessageNotifier<geometry_msgs::PointStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n, _1), "test_message", "frame1", 1));
	boost::scoped_ptr<MessageNotifier<geometry_msgs::PointStamped> > notifier1(new MessageNotifier<geometry_msgs::PointStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n1, _1), "test_message", "frame1", 1));
	boost::scoped_ptr<MessageNotifier<geometry_msgs::PointStamped> > notifier2(new MessageNotifier<geometry_msgs::PointStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n2, _1), "test_message", "frame1", 1));

  notifier->setTolerance(offset);
  notifier1->setTolerance(offset);
  notifier2->setTolerance(offset);

	ros::Duration().fromSec(0.2).sleep();

	ros::Time stamp = ros::Time::now();

	g_broadcaster->sendTransform(btTransform(btQuaternion(0,0,0), btVector3(1,2,3)), stamp, "frame1", "frame2");

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(c.mutex_, xt);

		EXPECT_EQ(true, lock.owns_lock());
	}


	geometry_msgs::PointStamped msg;
	msg.header.stamp = stamp + offset;
	msg.header.frame_id = "frame2";
	g_node->publish("test_message", msg);

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(n.getMutex(), xt);

		EXPECT_EQ(true, lock.owns_lock());
	}
	EXPECT_EQ(0, n.count_); //No return due to lack of space for offset

	Counter<tf::tfMessage> c2("tf_message", 1);

	g_broadcaster->sendTransform(btTransform(btQuaternion(0,0,0), btVector3(1,2,3)), stamp + offset*1.1, "frame1", "frame2");

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(c2.getMutex(), xt);

		EXPECT_EQ(true, lock.owns_lock());
	}


	msg.header.stamp = stamp;
	msg.header.frame_id = "frame2";
	g_node->publish("test_message", msg);

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(n1.getMutex(), xt);

		EXPECT_EQ(true, lock.owns_lock());
	}
	EXPECT_EQ(1, n1.count_); //recieved one callback

	msg.header.stamp = stamp + offset;
	msg.header.frame_id = "frame2";
	g_node->publish("test_message", msg);

	{
		boost::xtime xt;
		boost::xtime_get(&xt, boost::TIME_UTC);
		xt.sec += 10;

		boost::timed_mutex::scoped_timed_lock lock(n2.mutex_, xt);

		EXPECT_EQ(true, lock.owns_lock());
	}
	EXPECT_EQ(1, n2.count_); //recieved 1 first and last are out of range
}


int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "test_notifier", ros::init_options::AnonymousName);
	g_node = new ros::Node();
	g_node->advertise<geometry_msgs::PointStamped>("test_message", 0);
	g_node->advertise<geometry_msgs::PointStamped>("test_message2", 0);

	g_tf = new TransformListener();
	g_broadcaster = new TransformBroadcaster();

	int ret = RUN_ALL_TESTS();

	delete g_broadcaster;
	delete g_tf;

	delete g_node;

	return ret;
}

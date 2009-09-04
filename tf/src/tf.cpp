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

/** \author Tully Foote */

#include "tf/tf.h"
#include <sys/time.h>
#include "ros/assert.h"
#include "ros/ros.h"

using namespace tf;

// Must provide storage for non-integral static const class members.
// Otherwise you get undefined symbol errors on OS X (why not on Linux?).
// Thanks to Rob for pointing out the right way to do this.
const double tf::Transformer::DEFAULT_CACHE_TIME;


std::string tf::remap(const std::string& prefix, const std::string& frame_id)
{
  //  printf ("remapping prefix:%s with frame_id:%s\n", prefix.c_str(), frame_id.c_str());
  if (frame_id.size() > 0)
    if (frame_id[0] == '/')
    {
      return frame_id;
    }
  if (prefix.size() > 0)
  {
    if (prefix[0] == '/')
    {
      std::string composite = prefix;
      composite.append("/");
      composite.append(frame_id);
      return composite;
    }
    else
    {
      std::string composite;
      composite = "/";
      composite.append(prefix);
      composite.append("/");
      composite.append(frame_id);
      return composite;
    }

  }
  else
  {
    std::string composite;
    composite = "/";
    composite.append(frame_id);
    return composite;
  }
};



Transformer::Transformer(bool interpolating,
                                ros::Duration cache_time):
  cache_time(cache_time),
  interpolating (interpolating), 
  using_dedicated_thread_(false)
{
  max_extrapolation_distance_.fromNSec(DEFAULT_MAX_EXTRAPOLATION_DISTANCE);
  frameIDs_["NO_PARENT"] = 0;
  frames_.push_back(NULL);// new TimeCache(interpolating, cache_time, max_extrapolation_distance));//unused but needed for iteration over all elements
  frameIDs_reverse.push_back("NO_PARENT");

  return;
}

Transformer::~Transformer()
{
  /* deallocate all frames */
  boost::mutex::scoped_lock(frame_mutex_);
  for (std::vector<TimeCache*>::iterator  cache_it = frames_.begin(); cache_it != frames_.end(); ++cache_it)
  {
    delete (*cache_it);
  }

};


void Transformer::clear()
{
  boost::mutex::scoped_lock(frame_mutex_);
  if ( frames_.size() > 1 )
  {
    for (std::vector< TimeCache*>::iterator  cache_it = frames_.begin() + 1; cache_it != frames_.end(); ++cache_it)
    {
      (*cache_it)->clearList();
    }
  }
}

bool Transformer::setTransform(const Stamped<btTransform>& transform, const std::string& authority)
{

  Stamped<btTransform> mapped_transform = transform;
  mapped_transform.frame_id_ = remap(tf_prefix_, transform.frame_id_);
  mapped_transform.parent_id_ = remap(tf_prefix_, transform.parent_id_);

 
  bool error_exists = false;
  if (mapped_transform.frame_id_ == mapped_transform.parent_id_)
  {
    ROS_ERROR("TF_SELF_TRANSFORM: Ignoring transform from authority \"%s\" with parent_id and frame_id  \"%s\" because they are the same",  authority.c_str(), mapped_transform.frame_id_.c_str());
    error_exists = true;
  }

  if (mapped_transform.frame_id_ == "/")//empty frame id will be mapped to "/"
  {
    ROS_ERROR("TF_NO_FRAME_ID: Ignoring transform from authority \"%s\" because frame_id not set ", authority.c_str());
    error_exists = true;
  }

  if (mapped_transform.parent_id_ == "/")//empty parent id will be mapped to "/"
  {
    ROS_ERROR("TF_NO_PARENT_ID: Ignoring transform with frame_id \"%s\"  from authority \"%s\" because parent_id not set", mapped_transform.frame_id_.c_str(), authority.c_str());
    error_exists = true;
  }

  if (std::isnan(mapped_transform.getOrigin().x()) || std::isnan(mapped_transform.getOrigin().y()) || std::isnan(mapped_transform.getOrigin().z())||
      std::isnan(mapped_transform.getRotation().x()) ||       std::isnan(mapped_transform.getRotation().y()) ||       std::isnan(mapped_transform.getRotation().z()) ||       std::isnan(mapped_transform.getRotation().w()))
  {
    ROS_ERROR("TF_NAN_INPUT: Ignoring transform for frame_id \"%s\" from authority \"%s\" because of a nan value in the transform (%f %f %f) (%f %f %f %f)",
              mapped_transform.frame_id_.c_str(), authority.c_str(),
              mapped_transform.getOrigin().x(), mapped_transform.getOrigin().y(), mapped_transform.getOrigin().z(),
              mapped_transform.getRotation().x(), mapped_transform.getRotation().y(), mapped_transform.getRotation().z(), mapped_transform.getRotation().w()
              );
    error_exists = true;
  }

  if (error_exists)
    return false;
  unsigned int frame_number = lookupOrInsertFrameNumber(mapped_transform.frame_id_);
  if (getFrame(frame_number)->insertData(TransformStorage(mapped_transform, lookupOrInsertFrameNumber(mapped_transform.parent_id_))))
  {
    frame_authority_[frame_number] = authority;
  }
  else
  {
    ROS_WARN("TF_OLD_DATA ignoring data from the past for frame %s at time %g according to authority %s\nPossible reasons are listed at ", mapped_transform.frame_id_.c_str(), mapped_transform.stamp_.toSec(), authority.c_str());
    return false;
  }

  {
    boost::mutex::scoped_lock lock(transforms_changed_mutex_);
    transforms_changed_();
  }

  return true;
};


void Transformer::lookupTransform(const std::string& target_frame, const std::string& source_frame,
                     const ros::Time& time, Stamped<btTransform>& transform) const
{
  std::string mapped_target_frame = remap(tf_prefix_, target_frame);
  std::string mapped_source_frame = remap(tf_prefix_, source_frame);

  // Short circuit if zero length transform to allow lookups on non existant links
  if (mapped_source_frame == mapped_target_frame)
  {
    transform.setIdentity();

    if (time == ros::Time())
      transform.stamp_ = ros::Time::now();
    else
      transform.stamp_  = time;

    transform.frame_id_ = target_frame;
    return;
  }

  //  printf("Mapped Source: %s \nMapped Target: %s\n", mapped_source_frame.c_str(), mapped_target_frame.c_str());
  int retval = NO_ERROR;
  ros::Time temp_time;
  std::string error_string;
  //If getting the latest get the latest common time
  if (time == ros::Time())
    retval = getLatestCommonTime(mapped_target_frame, mapped_source_frame, temp_time, &error_string);
  else
    temp_time = time;

  TransformLists t_list;

  if (retval == NO_ERROR)
    try
    {
      retval = lookupLists(lookupFrameNumber( mapped_target_frame), temp_time, lookupFrameNumber( mapped_source_frame), t_list, &error_string);
    }
    catch (tf::LookupException &ex)
    {
      error_string = ex.what();
      retval = LOOKUP_ERROR;
    }
  if (retval != NO_ERROR)
  {
    std::stringstream ss;
    ss << " When trying to transform between " << mapped_source_frame << " and " << mapped_target_frame <<".";
    if (retval == LOOKUP_ERROR)
      throw LookupException(error_string + ss.str());
    if (retval == CONNECTIVITY_ERROR)
      throw ConnectivityException(error_string + ss.str());
  }

  if (test_extrapolation(temp_time, t_list, &error_string))
    {
    std::stringstream ss;
    if (time == ros::Time())// Using latest common time if we extrapolate this means that one of the links is out of date
    {
      ss << "Could not find a common time " << mapped_source_frame << " and " << mapped_target_frame <<".";
      throw ConnectivityException(ss.str());
    }
    else
    {
      ss << " When trying to transform between " << mapped_source_frame << " and " << mapped_target_frame <<"."
         << " See http://pr.willowgarage.com/pr-docs/ros-packages/tf/html/faq.html" << std::endl;
      throw ExtrapolationException(error_string + ss.str());
    }
    }


  transform.setData( computeTransformFromList(t_list));
  transform.stamp_ = temp_time;
  transform.frame_id_ = target_frame;

};

void Transformer::lookupTransform(const std::string& target_frame,const ros::Time& target_time, const std::string& source_frame,
                     const ros::Time& source_time, const std::string& fixed_frame, Stamped<btTransform>& transform) const
{
  tf::Stamped<tf::Transform> temp1, temp2;
  lookupTransform(fixed_frame, source_frame, source_time, temp1);
  lookupTransform(target_frame, fixed_frame, target_time, temp2);
  transform.setData( temp2 * temp1);
  transform.stamp_ = temp2.stamp_;
  transform.frame_id_ = target_frame;

};



bool Transformer::waitForTransform(const std::string& target_frame, const std::string& source_frame,
                                   const ros::Time& time,
                                   const ros::Duration& timeout, const ros::Duration& polling_sleep_duration,
                                   std::string* error_msg) const
{
  
  ROS_ASSERT_MSG(using_dedicated_thread_, "Do not call waitForTransform unless you are using another thread for populating data. If you are using multiple threads setUsingDedicatedThread(true)");

  ros::Time start_time = ros::Time::now();
  while (!canTransform(target_frame, source_frame, time, error_msg))
  {
    if ((ros::Time::now() - start_time) >= timeout)
      return false;
    ros::Duration(polling_sleep_duration).sleep(); //\todo remove copy construction after ros 0.5.1 is released
  }
  return true;
}


bool Transformer::canTransform(const std::string& target_frame, const std::string& source_frame,
                               const ros::Time& time,
                               std::string* error_msg) const
{
  std::string mapped_target_frame = remap(tf_prefix_, target_frame);
  std::string mapped_source_frame = remap(tf_prefix_, source_frame);

  ros::Time local_time = time;

  //break out early if no op transform
  if (mapped_source_frame == mapped_target_frame) return true;

  if (local_time == ros::Time())
    if (NO_ERROR != getLatestCommonTime(mapped_source_frame, mapped_target_frame, local_time, error_msg)) // set time if zero
    {
      return false;
    }
  


  TransformLists t_list;
  ///\todo check return
  int retval;
  try
  {
    retval = lookupLists(lookupFrameNumber( mapped_target_frame), local_time, lookupFrameNumber( mapped_source_frame), t_list, error_msg);
  }
  catch (tf::LookupException &ex)
  {
    return false;
  }
  


  ///\todo WRITE HELPER FUNCITON TO RETHROW
  if (retval != NO_ERROR)
  {
    if (retval == LOOKUP_ERROR)
    {
      return false;
    }
    if (retval == CONNECTIVITY_ERROR)
    {
      return false;
    }
  }

  if (test_extrapolation(local_time, t_list, error_msg))
    {
      return false;
    }

  return true;
};

bool Transformer::canTransform(const std::string& target_frame,const ros::Time& target_time, const std::string& source_frame,
                               const ros::Time& source_time, const std::string& fixed_frame,
                               std::string* error_msg) const
{
  return canTransform(target_frame, fixed_frame, target_time) && canTransform(fixed_frame, source_frame, source_time, error_msg);
};

bool Transformer::waitForTransform(const std::string& target_frame,const ros::Time& target_time, const std::string& source_frame,
                                   const ros::Time& source_time, const std::string& fixed_frame,
                                   const ros::Duration& timeout, const ros::Duration& polling_sleep_duration,
                                   std::string* error_msg) const
{
  return waitForTransform(target_frame, fixed_frame, target_time, timeout, polling_sleep_duration, error_msg) && waitForTransform(fixed_frame, source_frame, source_time, timeout, polling_sleep_duration, error_msg);
};


bool Transformer::getParent(const std::string& frame_id, ros::Time time, std::string& parent) const
{
  std::string mapped_frame_id = tf::remap(tf_prefix_, frame_id);
  tf::TimeCache* cache;
  try
  {
    cache = getFrame(lookupFrameNumber(mapped_frame_id));
  }
  catch  (tf::LookupException &ex)
  {
    ROS_ERROR("Transformer::getParent: %s",ex.what());
    return false;
  }

  TransformStorage temp;
  if (! cache->getData(time, temp)) {
    ROS_DEBUG("Transformer::getParent: No data for parent of %s", mapped_frame_id.c_str());
    return false;
  }
  if (temp.parent_id_ == "NO_PARENT") {
    ROS_DEBUG("Transformer::getParent: No parent for %s", mapped_frame_id.c_str());
    return false;
  }
  parent= temp.parent_id_;
  return true;

};


bool Transformer::frameExists(const std::string& frame_id_str) const
{
  boost::mutex::scoped_lock(frame_mutex_);
  std::string frame_id_remapped = tf::remap(tf_prefix_, frame_id_str);
  
  std::map<std::string, unsigned int>::const_iterator map_it = frameIDs_.find(frame_id_remapped);
  if (map_it == frameIDs_.end())
  {
      return false;
  }
  else
    return true;
}

void Transformer::setExtrapolationLimit(const ros::Duration& distance)
{
  max_extrapolation_distance_ = distance;
}

int Transformer::getLatestCommonTime(const std::string& source, const std::string& dest, ros::Time & time, std::string * error_string) const
{
  std::string mapped_source = tf::remap(tf_prefix_, source);
  std::string mapped_dest = tf::remap(tf_prefix_, dest);

  time = ros::Time(UINT_MAX, 999999999);///\todo replace with ros::TIME_MAX when it is merged from stable
  int retval;
  TransformLists lists;
  try
  {
    retval = lookupLists(lookupFrameNumber(mapped_dest), ros::Time(), lookupFrameNumber(mapped_source), lists, error_string);
  }
  catch (tf::LookupException &ex)
  {
    time = ros::Time();
    if (error_string) *error_string = ex.what();
    return LOOKUP_ERROR;
  }
  if (retval == NO_ERROR)
  {
    //Set time to latest timestamp of frameid in case of target and mapped_source frame id are the same
    if (lists.inverseTransforms.size() == 0 && lists.forwardTransforms.size() == 0)
    {
      time = ros::Time::now();
      return retval;
    }

    for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      if (time > lists.inverseTransforms[i].stamp_)
        time = lists.inverseTransforms[i].stamp_;
    }
    for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++)
    {
      if (time > lists.forwardTransforms[i].stamp_)
        time = lists.forwardTransforms[i].stamp_;
    }

  }
  else
    time.fromSec(0);

  return retval;
};



int Transformer::lookupLists(unsigned int target_frame, ros::Time time, unsigned int source_frame, TransformLists& lists, std::string * error_string) const
{
  /*  timeval tempt;
  gettimeofday(&tempt,NULL);
  std::cerr << "Looking up list at " <<tempt.tv_sec * 1000000ULL + tempt.tv_usec << std::endl;
  */

  ///\todo add fixed frame support

  //Clear lists before operating
  lists.forwardTransforms.clear();
  lists.inverseTransforms.clear();
  //  TransformLists mTfLs;
  if (target_frame == source_frame)
    return 0;  //Don't do anythign if we're not going anywhere

  TransformStorage temp;

  unsigned int frame = source_frame;
  unsigned int counter = 0;  //A counter to keep track of how deep we've descended
  unsigned int last_inverse;
  if (getFrame(frame) == NULL) //Test if source frame exists this will throw a lookup error if it does not (inside the loop it will be caught)
  {
    if (error_string) *error_string = "Source Frame Doesn't Exist";
    return LOOKUP_ERROR;//throw LookupException("Frame didn't exist");
  }
  while (true)
    {
      //      printf("getting data from %d:%s \n", frame, lookupFrameString(frame).c_str());

      TimeCache* pointer = getFrame(frame);
      ROS_ASSERT(pointer);

      if (! pointer->getData(time, temp))
      {
        last_inverse = frame;
        // this is thrown when there is no data
        break;
      }

      //break if parent is NO_PARENT (0)
      if (frame == 0)
      {
        last_inverse = frame;
        break;
      }
      lists.inverseTransforms.push_back(temp);

      frame = temp.parent_frame_id;


      /* Check if we've gone too deep.  A loop in the tree would cause this */
      if (counter++ > MAX_GRAPH_DEPTH)
      {
        if (error_string)
        {
          std::stringstream ss;
          ss<<"Recursed too deep into graph ( > MAX_GRAPH_DEPTH) there is probably a loop in the graph" << std::endl
            << allFramesAsString() << std::endl;
          *error_string =ss.str();
        }
        return LOOKUP_ERROR;
        //        throw(LookupException(ss.str()));
      }
    }
  /*
    timeval tempt2;
  gettimeofday(&tempt2,NULL);
  std::cerr << "Side A " <<tempt.tv_sec * 1000000LL + tempt.tv_usec- tempt2.tv_sec * 1000000LL - tempt2.tv_usec << std::endl;
  */
  frame = target_frame;
  counter = 0;
  unsigned int last_forward;
  if (getFrame(frame) == NULL)
  {
    if (error_string) *error_string = "Target Frame Did Not Exist";
    return LOOKUP_ERROR;
  }//throw LookupException("fixme");; //Test if source frame exists this will throw a lookup error if it does not (inside the loop it will be caught)
  while (true)
    {

      TimeCache* pointer = getFrame(frame);
      ROS_ASSERT(pointer);


      if(!  pointer->getData(time, temp))
      {
        last_forward = frame;
        break;
      }

      //break if parent is NO_PARENT (0)
      if (frame == 0)
      {
        last_forward = frame;
        break;
      }
      //      std::cout << "pushing back" << temp.frame_id_ << std::endl;
      lists.forwardTransforms.push_back(temp);
      frame = temp.parent_frame_id;

      /* Check if we've gone too deep.  A loop in the tree would cause this*/
      if (counter++ > MAX_GRAPH_DEPTH){
        if (error_string)
        {
          std::stringstream ss;
          ss<<"Recursed too deep into graph ( > MAX_GRAPH_DEPTH) there is probably a loop in the graph" << std::endl
            << allFramesAsString() << std::endl;
          *error_string = ss.str();
        }
        return LOOKUP_ERROR;//throw(LookupException(ss.str()));
      }
    }
  /*
  gettimeofday(&tempt2,NULL);
  std::cerr << "Side B " <<tempt.tv_sec * 1000000LL + tempt.tv_usec- tempt2.tv_sec * 1000000LL - tempt2.tv_usec << std::endl;
  */

  /* Check the zero length cases*/
  if (lists.inverseTransforms.size() == 0)
  {
    if (lists.forwardTransforms.size() == 0) //If it's going to itself it's already been caught
    {
      if (error_string)
      {
        std::stringstream ss;
        ss<< "No Common Parent Case D between "<< lookupFrameString(target_frame) <<" and " << lookupFrameString(source_frame)
          << std::endl << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return CONNECTIVITY_ERROR;
    }

    if (last_forward != source_frame)  //\todo match with case A
    {
      if (error_string)
      {
        std::stringstream ss;
        ss<< "No Common Parent Case C between " << lookupFrameString(target_frame) <<" and " << lookupFrameString(source_frame)
          << std::endl << allFramesAsString() << std::endl << lists.forwardTransforms.size() << " forward length"
          << " with " << lookupFrameString(last_forward) << std::endl;
        *error_string = ss.str();
      }
      return CONNECTIVITY_ERROR;
    }
    else return 0;
  }

  if (lists.forwardTransforms.size() == 0)
  {
    if (lists.inverseTransforms.size() == 0)  //If it's going to itself it's already been caught
    {//\todo remove THis is the same as case D
      if (error_string)
      {
        std::stringstream ss;
        ss<< "No Common Parent Case B between "<< lookupFrameString(target_frame) <<" and " << lookupFrameString(source_frame) << std::endl << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return CONNECTIVITY_ERROR;
    }

    try
    {
      if (lookupFrameNumber(lists.inverseTransforms.back().parent_id_) != target_frame)
      {
        std::stringstream ss;
      ss<< "No Common Parent Case A between "<< lookupFrameString(target_frame) <<" and " << lookupFrameString(source_frame)  << std::endl << allFramesAsString() << std::endl << lists.inverseTransforms.back().parent_id_ << std::endl;
      if (error_string) *error_string = ss.str();
      return CONNECTIVITY_ERROR;
    }
    else return 0;
    }
    catch (tf::LookupException & ex)
    {
      if (error_string) *error_string = ex.what();
      return LOOKUP_ERROR;
    }
  }


  /* Make sure the end of the search shares a parent. */
  if (last_forward != last_inverse)
  {
    if (error_string)
    {
      std::stringstream ss;
      ss<< "No Common Parent, at top of search between "<< lookupFrameString(target_frame) <<" and " << lookupFrameString(source_frame)  << std::endl << allFramesAsString() << std::endl;
      *error_string = ss.str();
    }
    return CONNECTIVITY_ERROR;
  }
  /* Make sure that we don't have a no parent at the top */
  try
  {
    if (lookupFrameNumber(lists.inverseTransforms.back().frame_id_) == 0 || lookupFrameNumber( lists.forwardTransforms.back().frame_id_) == 0)
    {
      if (error_string) *error_string = "NO_PARENT at top of tree";
      return CONNECTIVITY_ERROR;
    }


    /*
      gettimeofday(&tempt2,NULL);
      std::cerr << "Base Cases done" <<tempt.tv_sec * 1000000LL + tempt.tv_usec- tempt2.tv_sec * 1000000LL - tempt2.tv_usec << std::endl;
    */

    while (lookupFrameNumber(lists.inverseTransforms.back().frame_id_) == lookupFrameNumber(lists.forwardTransforms.back().frame_id_))
    {
      lists.inverseTransforms.pop_back();
      lists.forwardTransforms.pop_back();

      // Make sure we don't go beyond the beginning of the list.
      // (The while statement above doesn't fail if you hit the beginning of the list,
      // which happens in the zero distance case.)
      if (lists.inverseTransforms.size() == 0 || lists.forwardTransforms.size() == 0)
	break;
    }
  }
  catch (tf::LookupException & ex)
  {
    if (error_string) *error_string = ex.what();
    return LOOKUP_ERROR;
  }  /*
       gettimeofday(&tempt2,NULL);
       std::cerr << "Done looking up list " <<tempt.tv_sec * 1000000LL + tempt.tv_usec- tempt2.tv_sec * 1000000LL - tempt2.tv_usec << std::endl;
     */
  return 0;

  }


bool Transformer::test_extrapolation(const ros::Time& target_time, const TransformLists& lists, std::string * error_string) const
{
  bool retval = false;
  std::stringstream ss;
  ss << std::fixed;
  ss.precision(3);
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      if (lists.inverseTransforms[i].mode_ == ONE_VALUE)
      {
        if (lists.inverseTransforms[i].stamp_ - target_time > max_extrapolation_distance_ || target_time - lists.inverseTransforms[i].stamp_ > max_extrapolation_distance_)
        {
          retval = true;
          if (error_string) {
            ss << "Extrapolation Too Far from single value: target_time iss "<< (target_time).toSec() <<", but the closest tf  data is at"
               << lists.inverseTransforms[i].stamp_.toSec()  <<" which is "<<(target_time - lists.inverseTransforms[i].stamp_).toSec()
               << " seconds away.";
            if ( max_extrapolation_distance_ > ros::Duration(0))
            {
              ss << "This is greater than the max_extrapolation_distance of "
                 << (max_extrapolation_distance_).toSec() <<".";
            }
          }
        }
      }
      else if (lists.inverseTransforms[i].mode_ == EXTRAPOLATE_BACK)
      {
        if ( lists.inverseTransforms[i].stamp_ - target_time > max_extrapolation_distance_)
        {
          retval = true;
          if (error_string) {
            ss << "Extrapolation Too Far in the past: target_time is "<< (target_time).toSec() <<", but the closest tf  data is at "
               << lists.inverseTransforms[i].stamp_.toSec()  <<" which is "<< (target_time - lists.inverseTransforms[i].stamp_).toSec()
               << " seconds away.";
            if ( max_extrapolation_distance_ > ros::Duration(0))
            {
              ss << "This is greater than the max_extrapolation_distance of "
                 << (max_extrapolation_distance_).toSec() <<".";
            }
          }
        }
      }
      else if( lists.inverseTransforms[i].mode_ == EXTRAPOLATE_FORWARD)
      {
        if ( target_time - lists.inverseTransforms[i].stamp_ > max_extrapolation_distance_)
        {
          retval = true;
          if (error_string)
            ss << "Extrapolation Too Far in the future: target_time is "<< (target_time).toSec() <<", but the closest tf  data is at "
               << lists.inverseTransforms[i].stamp_.toSec()  <<" which is " << (target_time - lists.inverseTransforms[i].stamp_).toSec()
               << " seconds away.";
            if ( max_extrapolation_distance_ > ros::Duration(0))
            {
              ss << "This is greater than the max_extrapolation_distance of "
                 << (max_extrapolation_distance_).toSec() <<".";
            }
        }
      }
    }

  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++)
    {
      if (lists.forwardTransforms[i].mode_ == ONE_VALUE)
      {
        if (lists.forwardTransforms[i].stamp_ - target_time > max_extrapolation_distance_ || target_time - lists.forwardTransforms[i].stamp_ > max_extrapolation_distance_)
        {
          retval = true;
          if (error_string) {
            ss << "Extrapolation Too Far from single value: target_time is "<< (target_time).toSec() <<", but the closest tf  data is at "
               << lists.forwardTransforms[i].stamp_.toSec()  <<" which is "<< (target_time - lists.forwardTransforms[i].stamp_).toSec()
               << " seconds away.";
            if ( max_extrapolation_distance_ > ros::Duration(0))
            {
              ss << "This is greater than the max_extrapolation_distance of "
                 << (max_extrapolation_distance_).toSec() <<".";
            }
          }
        }
      }
      else if (lists.forwardTransforms[i].mode_ == EXTRAPOLATE_BACK)
      {
        if ( lists.forwardTransforms[i].stamp_ - target_time > max_extrapolation_distance_)
        {
          retval = true;
          if (error_string)
            ss << "Extrapolation Too Far in the past: target_time is "<< (target_time).toSec() <<", but the closest tf  data is at "
               << lists.forwardTransforms[i].stamp_.toSec()  <<" which is " << (target_time - lists.forwardTransforms[i].stamp_).toSec()
               << " seconds away.";
          if ( max_extrapolation_distance_ > ros::Duration(0))
          {
            ss << "This is greater than the max_extrapolation_distance of "
               << (max_extrapolation_distance_).toSec() <<".";
          }
        }
      }
      else if( lists.forwardTransforms[i].mode_ == EXTRAPOLATE_FORWARD)
      {
        if (target_time - lists.forwardTransforms[i].stamp_ > max_extrapolation_distance_)
        {
          retval = true;
          if (error_string)
            ss << "Extrapolation Too Far in the future: target_time is "<< (target_time).toSec() <<", but the closest tf  data is at "
               << lists.forwardTransforms[i].stamp_.toSec()  <<" which is "<< (target_time - lists.forwardTransforms[i].stamp_).toSec()
               << " seconds away.";
          if ( max_extrapolation_distance_ > ros::Duration(0))
          {
            ss << "This is greater than the max_extrapolation_distance of "
               << (max_extrapolation_distance_).toSec() <<".";
          }
        }
      }
    }

  if (error_string) ss << " See http://pr.willowgarage.com/pr-docs/ros-packages/tf/html/faq.html for more info.";

  if (error_string) *error_string = ss.str();
  return retval;


}


btTransform Transformer::computeTransformFromList(const TransformLists & lists) const
{
  btTransform retTrans;
  retTrans.setIdentity();
  ///@todo change these to iterators
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      retTrans *= (lists.inverseTransforms[lists.inverseTransforms.size() -1 - i]); //Reverse to get left multiply
    }
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++)
    {
      retTrans = (lists.forwardTransforms[lists.forwardTransforms.size() -1 - i]).inverse() * retTrans; //Do this list backwards(from backwards) for it was generated traveling the wrong way
    }

  return retTrans;
}


std::string Transformer::chainAsString(const std::string & target_frame, ros::Time target_time, const std::string & source_frame, ros::Time source_time, const std::string& fixed_frame) const
{
  std::string error_string;
  std::stringstream mstream;
  TransformLists lists;
  ///\todo check return code
  try
  {
    lookupLists(lookupFrameNumber(target_frame), target_time, lookupFrameNumber(source_frame), lists, &error_string);
  }
  catch (tf::LookupException &ex)
  {
    mstream << ex.what();
    return mstream.str();
  }
  mstream << "Inverse Transforms:" <<std::endl;
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      mstream << lists.inverseTransforms[i].frame_id_<<", ";
    }
  mstream << std::endl;

  mstream << "Forward Transforms: "<<std::endl ;
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++)
    {
      mstream << lists.forwardTransforms[i].frame_id_<<", ";
    }
  mstream << std::endl;
  return mstream.str();
}

void Transformer::chainAsVector(const std::string & target_frame, ros::Time target_time, const std::string & source_frame, ros::Time source_time, const std::string& fixed_frame, std::vector<std::string>& output) const
{
  std::string error_string;
  TransformLists lists;
  ///\todo check return code
  try
  {
    lookupLists(lookupFrameNumber(target_frame), target_time, lookupFrameNumber(source_frame), lists, &error_string);
  }
  catch (tf::LookupException &ex)
  {
    return;
  }

  output.clear(); //empty vector
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      output.push_back(lists.inverseTransforms[i].frame_id_);
    }
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++)
    {
      output.push_back(lists.forwardTransforms[i].frame_id_);
    }
}

std::string Transformer::allFramesAsString() const
{
  std::stringstream mstream;
  boost::mutex::scoped_lock(frame_mutex_);

  TransformStorage temp;



  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    unsigned int parent_id;
    if(  getFrame(counter)->getData(ros::Time(), temp))
      parent_id = temp.parent_frame_id;
    else
    {
      parent_id = 0;
    }
    mstream << "Frame "<< frameIDs_reverse[counter] << " exists with parent " << frameIDs_reverse[parent_id] << "." <<std::endl;
  }
  return mstream.str();
}

std::string Transformer::allFramesAsDot() const
{
  std::stringstream mstream;
  mstream << "digraph G {" << std::endl;
  boost::mutex::scoped_lock(frame_mutex_);

  TransformStorage temp;

  ros::Time current_time = ros::Time::now();

  if (frames_.size() ==1)
    mstream <<"\"no tf data recieved\"";

  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)//one referenced for 0 is no frame
  {
    unsigned int parent_id;
    if(  getFrame(counter)->getData(ros::Time(), temp))
      parent_id = temp.parent_frame_id;
    else
    {
      parent_id = 0;
    }
    if (parent_id != 0)
    {
      std::string authority = "no recorded authority";
      std::map<unsigned int, std::string>::const_iterator it = frame_authority_.find(counter);
      if (it != frame_authority_.end())
        authority = it->second;

      double rate = getFrame(counter)->getListLength() / std::max((getFrame(counter)->getLatestTimestamp().toSec() -
                                                                   getFrame(counter)->getOldestTimestamp().toSec() ), 0.0001);

      mstream << std::fixed; //fixed point notation
      mstream.precision(3); //3 decimal places
      mstream << "\"" << frameIDs_reverse[parent_id]   << "\"" << " -> "
              << "\"" << frameIDs_reverse[counter] << "\"" << "[label=\""
              << "Authority: " << authority << "\\n"
              << getFrame(counter)->getListLength() << " Readings averaging " << rate <<" Hz\\n"
              << " Latest reading: \\n" << getFrame(counter)->getLatestTimestamp().toSec()
              << " ( " << (current_time - getFrame(counter)->getLatestTimestamp()).toSec()
              <<" seconds ago )\\n"
              << " Oldest reading:\\n"
              << getFrame(counter)->getOldestTimestamp().toSec()
              << " ( " << (current_time - getFrame(counter)->getOldestTimestamp()).toSec()
              <<" seconds ago )\\n"
              <<"\"];" <<std::endl;
    }
  }
  mstream << "}";
  return mstream.str();
}

void Transformer::getFrameStrings(std::vector<std::string> & vec) const
{
  vec.clear();

  boost::mutex::scoped_lock(frame_mutex_);

  TransformStorage temp;

  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    vec.push_back(frameIDs_reverse[counter]);
  }
  return;
}

tf::TimeCache* Transformer::getFrame(unsigned int frame_id) const
{
  if (frame_id == 0) /// @todo check larger values too
    return NULL;
  else
    return frames_[frame_id];
};


void Transformer::transformQuaternion(const std::string& target_frame, const Stamped<Quaternion>& stamped_in, Stamped<Quaternion>& stamped_out) const
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  stamped_out.setData( transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformVector(const std::string& target_frame,
                                  const Stamped<tf::Vector3>& stamped_in,
                                  Stamped<tf::Vector3>& stamped_out) const
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  /** \todo may not be most efficient */
  btVector3 end = stamped_in;
  btVector3 origin = btVector3(0,0,0);
  btVector3 output = (transform * end) - (transform * origin);
  stamped_out.setData( output);

  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformPoint(const std::string& target_frame, const Stamped<Point>& stamped_in, Stamped<Point>& stamped_out) const
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
  stamped_out.parent_id_ = stamped_in.parent_id_;//only useful for transforms
};

void Transformer::transformPose(const std::string& target_frame, const Stamped<Pose>& stamped_in, Stamped<Pose>& stamped_out) const
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
  //  stamped_out.parent_id_ = stamped_in.parent_id_;//only useful for transforms
};


void Transformer::transformQuaternion(const std::string& target_frame, const ros::Time& target_time,
                                      const Stamped<Quaternion>& stamped_in,
                                      const std::string& fixed_frame,
                                      Stamped<Quaternion>& stamped_out) const
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  stamped_out.setData( transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformVector(const std::string& target_frame, const ros::Time& target_time,
                                  const Stamped<Vector3>& stamped_in,
                                  const std::string& fixed_frame,
                                  Stamped<Vector3>& stamped_out) const
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  /** \todo may not be most efficient */
  btVector3 end = stamped_in;
  btVector3 origin = btVector3(0,0,0);
  btVector3 output = (transform * end) - (transform * origin);
  stamped_out.setData( output);

  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformPoint(const std::string& target_frame, const ros::Time& target_time,
                                 const Stamped<Point>& stamped_in,
                                 const std::string& fixed_frame,
                                 Stamped<Point>& stamped_out) const
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
  stamped_out.parent_id_ = stamped_in.parent_id_;//only useful for transforms
};

void Transformer::transformPose(const std::string& target_frame, const ros::Time& target_time,
                                const Stamped<Pose>& stamped_in,
                                const std::string& fixed_frame,
                                Stamped<Pose>& stamped_out) const
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
  //  stamped_out.parent_id_ = stamped_in.parent_id_;//only useful for transforms
};

boost::signals::connection Transformer::addTransformsChangedListener(boost::function<void(void)> callback)
{
  boost::mutex::scoped_lock lock(transforms_changed_mutex_);
  return transforms_changed_.connect(callback);
}

void Transformer::removeTransformsChangedListener(boost::signals::connection c)
{
  boost::mutex::scoped_lock lock(transforms_changed_mutex_);
  c.disconnect();
}

/*
void Transformer::transformTransform(const std::string& target_frame,
                                  const geometry_msgs::TransformStamped& msg_in,
                                  geometry_msgs::TransformStamped& msg_out)
{
  Stamped<Transform> pin, pout;
  transformStampedMsgToTF(msg_in, pin);
  transformTransform(target_frame, pin, pout);
  transformStampedTFToMsg(pout, msg_out);
}

*/

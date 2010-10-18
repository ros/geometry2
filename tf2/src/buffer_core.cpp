/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include "tf2/buffer_core.h"
#include "tf2/exceptions.h"
#include "tf2_msgs/TF2Error.h"
//legacy
//#include "tf/tf.h"
//#include "tf/transform_datatypes.h"


using namespace tf2;

// Must provide storage for non-integral static const class members.
// Otherwise you get undefined symbol errors on OS X (why not on Linux?).
// Thanks to Rob for pointing out the right way to do this.
const double tf2::BufferCore::DEFAULT_CACHE_TIME;

/** \brief convert Transform msg to Transform */
void transformMsgToTF2(const geometry_msgs::Transform& msg, btTransform& bt)
{bt = btTransform(btQuaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), btVector3(msg.translation.x, msg.translation.y, msg.translation.z));};

/** \brief convert Transform to Transform msg*/
void transformTF2ToMsg(const btTransform& bt, geometry_msgs::Transform& msg)
{
  msg.translation.x = bt.getOrigin().x();
  msg.translation.y = bt.getOrigin().y();
  msg.translation.z = bt.getOrigin().z();
  msg.rotation.x = bt.getRotation().x();
  msg.rotation.y = bt.getRotation().y();
  msg.rotation.z = bt.getRotation().z();
  msg.rotation.w = bt.getRotation().w();
};


void setIdentity(geometry_msgs::Transform& tx)
{
  tx.translation.x = 0;
  tx.translation.y = 0;
  tx.translation.z = 0;
  tx.rotation.x = 0;
  tx.rotation.y = 0;
  tx.rotation.z = 0;
  tx.rotation.w = 1;
}

bool startsWithSlash(const std::string& frame_id)
{
  if (frame_id.size() > 0)
    if (frame_id[0] == '/')
      return true;
  return false;
}

std::string stripSlash(const std::string& in)
{
  std::string out = in;
  if (startsWithSlash(in))
    out.erase(0,1);
  return out;
};


bool BufferCore::warnFrameId(const std::string& function_name_arg, const std::string& frame_id) const
{
  bool retval = false;
  if (frame_id.size() == 0)
  {
    std::stringstream ss;
    ss << "Invalid argument passed to "<< function_name_arg <<" in tf2 frame_ids cannot be empty";
    ROS_WARN("%s",ss.str().c_str());
    retval = true;
  }
  if (startsWithSlash(frame_id))
  {
    std::stringstream ss;
    ss << "Invalid argument \"" << frame_id << "\" passed to "<< function_name_arg <<" in tf2 frame_ids cannot start with a '/' like: ";
    ROS_WARN("%s",ss.str().c_str());
    retval = true;
  }
  if (lookupFrameNumber(frame_id) == CompactFrameID(0))
  {
    /* Don't warn here.  It's not an invalid use case.  
      std::stringstream ss;
    ss << "\"" << frame_id << "\" passed to "<< function_name_arg <<" does not exist. ";
    ROS_WARN("%s",ss.str().c_str()); */
    retval = true;
  }
  return retval;
};

void BufferCore::validateFrameId(const std::string& function_name_arg, const std::string& frame_id) const
{
  if (frame_id.size() == 0)
  {
    std::stringstream ss;
    ss << "Invalid argument passed to "<< function_name_arg <<" in tf2 frame_ids cannot be empty";
    throw tf2::InvalidArgumentException(ss.str().c_str());
  }
  if (startsWithSlash(frame_id))
  {
    std::stringstream ss;
    ss << "Invalid argument \"" << frame_id << "\" passed to "<< function_name_arg <<" in tf2 frame_ids cannot start with a '/' like: ";
    throw tf2::InvalidArgumentException(ss.str().c_str());
  }
  if (lookupFrameNumber(frame_id) == CompactFrameID(0))
  {
    std::stringstream ss;
    ss << "\"" << frame_id << "\" passed to "<< function_name_arg <<" does not exist. ";
    throw tf2::LookupException(ss.str().c_str());
  }
  
};

BufferCore::BufferCore(ros::Duration cache_time) : cache_time_(cache_time)//: old_tf_(true, cache_time)
{
  max_extrapolation_distance_.fromNSec(DEFAULT_MAX_EXTRAPOLATION_DISTANCE);
  frameIDs_["NO_PARENT"] = 0;
  frames_.push_back(NULL);// new TimeCache(interpolating, cache_time, max_extrapolation_distance));//unused but needed for iteration over all elements
  frameIDs_reverse.push_back("NO_PARENT");

  return;
}

BufferCore::~BufferCore()
{

}

void BufferCore::clear()
{
  //old_tf_.clear();


  boost::mutex::scoped_lock(frame_mutex_);
  if ( frames_.size() > 1 )
  {
    for (std::vector< TimeCacheInterface*>::iterator  cache_it = frames_.begin() + 1; cache_it != frames_.end(); ++cache_it)
    {
      (*cache_it)->clearList();
    }
  }
  
}

bool BufferCore::setTransform(const geometry_msgs::TransformStamped& transform_in, const std::string& authority, bool is_static)
{

  /////BACKEARDS COMPATABILITY 
  /* tf::StampedTransform tf_transform;
  tf::transformStampedMsgToTF(transform_in, tf_transform);
  if  (!old_tf_.setTransform(tf_transform, authority))
  {
    printf("Warning old setTransform Failed but was not caught\n");
    }*/

  /////// New implementation
  geometry_msgs::TransformStamped stripped = transform_in;
  stripped.header.frame_id = stripSlash(stripped.header.frame_id);
  stripped.child_frame_id = stripSlash(stripped.child_frame_id);


  bool error_exists = false;
  if (stripped.child_frame_id == stripped.header.frame_id)
  {
    ROS_ERROR("TF_SELF_TRANSFORM: Ignoring transform from authority \"%s\" with frame_id and child_frame_id  \"%s\" because they are the same",  authority.c_str(), stripped.child_frame_id.c_str());
    error_exists = true;
  }

  if (stripped.child_frame_id == "")
  {
    ROS_ERROR("TF_NO_CHILD_FRAME_ID: Ignoring transform from authority \"%s\" because child_frame_id not set ", authority.c_str());
    error_exists = true;
  }

  if (stripped.header.frame_id == "")
  {
    ROS_ERROR("TF_NO_FRAME_ID: Ignoring transform with child_frame_id \"%s\"  from authority \"%s\" because frame_id not set", stripped.child_frame_id.c_str(), authority.c_str());
    error_exists = true;
  }

  if (std::isnan(stripped.transform.translation.x) || std::isnan(stripped.transform.translation.y) || std::isnan(stripped.transform.translation.z)||
      std::isnan(stripped.transform.rotation.x) ||       std::isnan(stripped.transform.rotation.y) ||       std::isnan(stripped.transform.rotation.z) ||       std::isnan(stripped.transform.rotation.w))
  {
    ROS_ERROR("TF_NAN_INPUT: Ignoring transform for child_frame_id \"%s\" from authority \"%s\" because of a nan value in the transform (%f %f %f) (%f %f %f %f)",
              stripped.child_frame_id.c_str(), authority.c_str(),
              stripped.transform.translation.x, stripped.transform.translation.y, stripped.transform.translation.z,
              stripped.transform.rotation.x, stripped.transform.rotation.y, stripped.transform.rotation.z, stripped.transform.rotation.w
              );
    error_exists = true;
  }

  if (error_exists)
    return false;
  

  CompactFrameID frame_number = lookupOrInsertFrameNumber(stripped.child_frame_id);
  TimeCacheInterface* frame = getFrame(frame_number);
  if (frame == NULL)
    frame = allocateFrame(frame_number, is_static);
  
    
  if (frame->insertData(TransformStorage(stripped, lookupOrInsertFrameNumber(stripped.header.frame_id), frame_number)))
  {
    frame_authority_[frame_number] = authority;
  }
  else
  {
    ROS_WARN("TF_OLD_DATA ignoring data from the past for frame %s at time %g according to authority %s\nPossible reasons are listed at ", stripped.child_frame_id.c_str(), stripped.header.stamp.toSec(), authority.c_str());
    return false;
  }
  return true;

};

TimeCacheInterface* BufferCore::allocateFrame(CompactFrameID cfid, bool is_static)
{
  TimeCacheInterface* frame_ptr = frames_[cfid.num_];
  if ( frame_ptr != NULL)
    delete frame_ptr;
  if (is_static)
    frames_[cfid.num_] = new StaticCache();
  else
    frames_[cfid.num_] = new TimeCache(cache_time_, max_extrapolation_distance_);
  
  return frames_[cfid.num_];
}
geometry_msgs::TransformStamped BufferCore::lookupTransform(const std::string& target_frame, 
                                                            const std::string& source_frame,
                                                            const ros::Time& time) const
{
  validateFrameId("lookupTransform argument target_frame", target_frame);
  validateFrameId("lookupTransform argument source_frame", source_frame);
  

  geometry_msgs::TransformStamped output_transform;
  // Short circuit if zero length transform to allow lookups on non existant links
  if (source_frame == target_frame)
  {
    setIdentity(output_transform.transform);


    output_transform.header.stamp = time;
    /*    if (time == ros::Time())
      output_transform.header.stamp = ros::Time(ros::TIME_MAX); ///\todo review what this should be
    else
      output_transform.header.stamp  = time;
    */

    output_transform.child_frame_id = source_frame;
    output_transform.header.frame_id = target_frame;
    return output_transform;
  }
  //  printf("Mapped Source: %s \nMapped Target: %s\n", source_frame.c_str(), target_frame.c_str());
  int retval = tf2_msgs::TF2Error::NO_ERROR;
  ros::Time temp_time;
  std::string error_string;
  //If getting the latest get the latest common time
  if (time == ros::Time())
    retval = getLatestCommonTime(target_frame, source_frame, temp_time, &error_string);
  else
    temp_time = time;

  TransformLists t_list;

  if (retval == tf2_msgs::TF2Error::NO_ERROR)
    retval = lookupLists(lookupFrameNumber( target_frame), temp_time, lookupFrameNumber( source_frame), t_list, &error_string);
  else //if (retval != tf2_msgs::TF2Error::NO_ERROR)
  {
    std::stringstream ss;
    ss << " When trying to transform between " << source_frame << " and " << target_frame <<".";
    if (retval == tf2_msgs::TF2Error::LOOKUP_ERROR)
      throw LookupException(error_string + ss.str());
    if (retval == tf2_msgs::TF2Error::CONNECTIVITY_ERROR)
      throw ConnectivityException(error_string + ss.str());
  }

  if (test_extrapolation(temp_time, t_list, &error_string))
    {
    std::stringstream ss;
    if (time == ros::Time())// Using latest common time if we extrapolate this means that one of the links is out of date
    {
      ss << "Could not find a common time " << source_frame << " and " << target_frame <<".";
      throw ConnectivityException(ss.str());
    }
    else
    {
      ss << " When trying to transform between " << source_frame << " and " << target_frame <<"."<< std::endl;
      throw ExtrapolationException(error_string + ss.str());
    }
    }


  btTransform output = computeTransformFromList(t_list);
  transformTF2ToMsg(output, output_transform.transform);
  output_transform.header.stamp = temp_time;
  output_transform.header.frame_id = target_frame;
  output_transform.child_frame_id = source_frame;
  return output_transform;
};

                                                       
geometry_msgs::TransformStamped BufferCore::lookupTransform(const std::string& target_frame, 
                                                        const ros::Time& target_time,
                                                        const std::string& source_frame,
                                                        const ros::Time& source_time,
                                                        const std::string& fixed_frame) const
{
  validateFrameId("lookupTransform argument target_frame", target_frame);
  validateFrameId("lookupTransform argument source_frame", source_frame);
  validateFrameId("lookupTransform argument source_frame", fixed_frame);

  geometry_msgs::TransformStamped output;
  geometry_msgs::TransformStamped temp1 =  lookupTransform(fixed_frame, source_frame, source_time);
  geometry_msgs::TransformStamped temp2 =  lookupTransform(target_frame, fixed_frame, target_time);
  
  btTransform bt1, bt2;
  transformMsgToTF2(temp1.transform, bt1);
  transformMsgToTF2(temp2.transform, bt2);
  transformTF2ToMsg(bt2*bt1, output.transform);
  output.header.stamp = temp2.header.stamp;
  output.header.frame_id = target_frame;
  output.child_frame_id = source_frame;
  return output;
};



/*
geometry_msgs::Twist BufferCore::lookupTwist(const std::string& tracking_frame, 
                                          const std::string& observation_frame, 
                                          const ros::Time& time, 
                                          const ros::Duration& averaging_interval) const
{
  try
  {
  geometry_msgs::Twist t;
  old_tf_.lookupTwist(tracking_frame, observation_frame, 
                      time, averaging_interval, t);
  return t;
  }
  catch (tf::LookupException& ex)
  {
    throw tf2::LookupException(ex.what());
  }
  catch (tf::ConnectivityException& ex)
  {
    throw tf2::ConnectivityException(ex.what());
  }
  catch (tf::ExtrapolationException& ex)
  {
    throw tf2::ExtrapolationException(ex.what());
  }
  catch (tf::InvalidArgument& ex)
  {
    throw tf2::InvalidArgumentException(ex.what());
  }
};

geometry_msgs::Twist BufferCore::lookupTwist(const std::string& tracking_frame, 
                                          const std::string& observation_frame, 
                                          const std::string& reference_frame,
                                          const tf2::Point & reference_point, 
                                          const std::string& reference_point_frame, 
                                          const ros::Time& time, 
                                          const ros::Duration& averaging_interval) const
{
  try{
  geometry_msgs::Twist t;
  old_tf_.lookupTwist(tracking_frame, observation_frame, reference_frame, reference_point, reference_point_frame,
                      time, averaging_interval, t);
  return t;
  }
  catch (tf::LookupException& ex)
  {
    throw tf2::LookupException(ex.what());
  }
  catch (tf::ConnectivityException& ex)
  {
    throw tf2::ConnectivityException(ex.what());
  }
  catch (tf::ExtrapolationException& ex)
  {
    throw tf2::ExtrapolationException(ex.what());
  }
  catch (tf::InvalidArgument& ex)
  {
    throw tf2::InvalidArgumentException(ex.what());
  }
};
*/


bool BufferCore::canTransform(const std::string& target_frame, const std::string& source_frame,
                           const ros::Time& time, std::string* error_msg) const
{
  if (warnFrameId("canTransform argument target_frame", target_frame))
    return false;
  if (warnFrameId("canTransform argument source_frame", source_frame))
    return false;

  ros::Time local_time = time;

  //break out early if no op transform
  if (source_frame == target_frame) return true;

  if (local_time == ros::Time())
    if (getLatestCommonTime(source_frame, target_frame, local_time, error_msg) != tf2_msgs::TF2Error::NO_ERROR) // set time if zero
    {
      return false;
    }
  
  TransformLists t_list;
  ///\todo check return
  int retval;
  retval = lookupLists(lookupFrameNumber( target_frame), local_time, lookupFrameNumber( source_frame), t_list, error_msg);

  ///\todo WRITE HELPER FUNCITON TO RETHROW
  if (retval != tf2_msgs::TF2Error::NO_ERROR)
  {
    if (retval == tf2_msgs::TF2Error::LOOKUP_ERROR)
    {
      return false;
    }
    if (retval == tf2_msgs::TF2Error::CONNECTIVITY_ERROR)
    {
      return false;
    }
  }

  if (test_extrapolation(local_time, t_list, error_msg))
    {
      return false;
    }

  return true;
}

bool BufferCore::canTransform(const std::string& target_frame, const ros::Time& target_time,
                          const std::string& source_frame, const ros::Time& source_time,
                          const std::string& fixed_frame, std::string* error_msg) const
{
  if (warnFrameId("canTransform argument target_frame", target_frame))
    return false;
  if (warnFrameId("canTransform argument source_frame", source_frame))
    return false;
  if (warnFrameId("canTransform argument source_frame", fixed_frame))
    return false;
  return canTransform(target_frame, fixed_frame, target_time) && canTransform(fixed_frame, source_frame, source_time, error_msg);
}


tf2::TimeCacheInterface* BufferCore::getFrame(CompactFrameID frame_id) const
{
  if (frame_id == CompactFrameID(0) || frame_id.num_ > frames_.size()) /// @todo check larger values too
    return NULL;
  else
  {
    return frames_[frame_id.num_];
  }
};

CompactFrameID BufferCore::lookupFrameNumber(const std::string& frameid_str) const
{
  CompactFrameID retval;
  boost::mutex::scoped_lock(frame_mutex_);
  std::map<std::string, CompactFrameID>::const_iterator map_it = frameIDs_.find(frameid_str);
  if (map_it == frameIDs_.end())
  {
    retval = CompactFrameID(0);
  }
  else
    retval = map_it->second;
  return retval;
};

CompactFrameID BufferCore::lookupOrInsertFrameNumber(const std::string& frameid_str)
{
  CompactFrameID retval = 0;
  boost::mutex::scoped_lock(frame_mutex_);
  std::map<std::string, CompactFrameID>::iterator map_it = frameIDs_.find(frameid_str);
  if (map_it == frameIDs_.end())
  {
    retval = CompactFrameID(frames_.size());
    frames_.push_back( NULL);//new TimeCache(cache_time_, max_extrapolation_distance_));
    frameIDs_[frameid_str] = retval;
    frameIDs_reverse.push_back(frameid_str);
  }
  else
    retval = frameIDs_[frameid_str];

  return retval;
};

std::string BufferCore::lookupFrameString(CompactFrameID frame_id_num) const
{
    if (frame_id_num.num_ >= frameIDs_reverse.size())
    {
      std::stringstream ss;
      ss << "Reverse lookup of frame id " << frame_id_num.num_ << " failed!";
      throw tf2::LookupException(ss.str());
    }
    else
      return frameIDs_reverse[frame_id_num.num_];
};

void BufferCore::createConnectivityErrorString(CompactFrameID source_frame, CompactFrameID target_frame, std::string* out) const
{
  if (!out)
  {
    return;
  }
  *out = std::string("Could not find a connection between '"+lookupFrameString(target_frame)+"' and '"+
                     lookupFrameString(source_frame)+"' because they are not part of the same tree."+
                     "Tf has two or more unconnected trees.");
}

int BufferCore::lookupLists(CompactFrameID target_frame, ros::Time time, CompactFrameID source_frame, TransformLists& lists, std::string * error_string) const
{
  /*timeval tempt;
  gettimeofday(&tempt,NULL);
  std::cerr << "Looking up list at " <<tempt.tv_sec * 1000000ULL + tempt.tv_usec << std::endl;
  */  
  //Clear lists before operating
  lists.forwardTransforms.clear();
  lists.inverseTransforms.clear();
  //  TransformLists mTfLs;
  if (target_frame == source_frame)
    return 0;  //Don't do anythign if we're not going anywhere

  TransformStorage temp;

  CompactFrameID frame = source_frame;
  unsigned int counter = 0;  //A counter to keep track of how deep we've descended
  CompactFrameID last_inverse;
  while (true)
    {
      //      printf("getting data from %d:%s \n", frame, lookupFrameString(frame).c_str());

      TimeCacheInterface* pointer = getFrame(frame);
      if (! pointer)
      {
        last_inverse = frame;
        break;
        
      }
      else if ( ! pointer->getData(time, temp))
      {
        last_inverse = frame;
        break;
      }

      //break if parent is NO_PARENT (0)
      if (frame == 0)
      {
        last_inverse = frame;
        break;
      }
      lists.inverseTransforms.push_back(temp);

      frame = temp.frame_id_;


      /* Check if we've gone too deep.  A loop in the tree would cause this */
      if (counter++ > MAX_GRAPH_DEPTH)
      {
        if (error_string)
        {
          std::stringstream ss;
          ss<<"The tf tree is invalid because it contains a loop." << std::endl
            << allFramesAsString() << std::endl;
          *error_string =ss.str();
        }
        return tf2_msgs::TF2Error::LOOKUP_ERROR;
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
  CompactFrameID last_forward;
  while (true)
    {

      TimeCacheInterface* pointer = getFrame(frame);
      if (! pointer)
      {
        last_forward = frame;
        break;
        
      }
      else if( !  pointer->getData(time, temp))
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
      lists.forwardTransforms.push_back(temp);
      frame = temp.frame_id_;

      /* Check if we've gone too deep.  A loop in the tree would cause this*/
      if (counter++ > MAX_GRAPH_DEPTH){
        if (error_string)
        {
          std::stringstream ss;
          ss<<"The tf tree is invalid because it contains a loop." << std::endl
            << allFramesAsString() << std::endl;
          *error_string = ss.str();
        }
        return tf2_msgs::TF2Error::LOOKUP_ERROR;//throw(LookupException(ss.str()));
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
      createConnectivityErrorString(source_frame, target_frame, error_string);
      return tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
    }

    if (! (last_forward == source_frame) )  //\todo match with case A
    {
      createConnectivityErrorString(source_frame, target_frame, error_string);
      return tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
    }
    else return 0;
  }

  if (lists.forwardTransforms.size() == 0)
  {
    if (lists.inverseTransforms.size() == 0)  //If it's going to itself it's already been caught
    {//\todo remove THis is the same as case D
      createConnectivityErrorString(source_frame, target_frame, error_string);
      return tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
    }

    if (lists.inverseTransforms.back().frame_id_ != target_frame)
    {
      createConnectivityErrorString(source_frame, target_frame, error_string);
      return tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
    }
    else return 0;
    
  }
  

  /* Make sure the end of the search shares a parent. */
  if (!(last_forward == last_inverse))
  {
    createConnectivityErrorString(source_frame, target_frame, error_string);
    return tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
  }
  /* Make sure that we don't have a no parent at the top */
    {
    if (lists.inverseTransforms.back().child_frame_id_ == 0 || lists.forwardTransforms.back().child_frame_id_ == 0)
    {
      //if (error_string) *error_string = "NO_PARENT at top of tree";
      createConnectivityErrorString(source_frame, target_frame, error_string);
      return tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
    }

    /*
      gettimeofday(&tempt2,NULL);
      std::cerr << "Base Cases done" <<tempt.tv_sec * 1000000LL + tempt.tv_usec- tempt2.tv_sec * 1000000LL - tempt2.tv_usec << std::endl;
    */

    while (lists.inverseTransforms.back().child_frame_id_ == lists.forwardTransforms.back().child_frame_id_)
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
  /*
       gettimeofday(&tempt2,NULL);
       std::cerr << "Done looking up list " <<tempt.tv_sec * 1000000LL + tempt.tv_usec- tempt2.tv_sec * 1000000LL - tempt2.tv_usec << std::endl;
     */
  return 0;

  }


bool BufferCore::test_extrapolation_one_value(const ros::Time& target_time, const TransformStorage& tr, std::string* error_string) const
{
  if (tr.mode_ == ONE_VALUE)
  {
    if (tr.stamp_ - target_time > max_extrapolation_distance_ || target_time - tr.stamp_ > max_extrapolation_distance_)
    {
      if (error_string) {
        std::stringstream ss;
        ss << std::fixed;
        ss.precision(3);
        ss << "You requested a transform at time " << (target_time).toSec() 
           << ",\n but the tf buffer only contains a single transform " 
           << "at time " << tr.stamp_.toSec() << ".\n";
        if ( max_extrapolation_distance_ > ros::Duration(0))
        {
          ss << "The tf extrapollation distance is set to " 
             << (max_extrapolation_distance_).toSec() <<" seconds.\n";
        }
        *error_string = ss.str();
      }
      return true;
    }
  }
  return false;
}


bool BufferCore::test_extrapolation_past(const ros::Time& target_time, const TransformStorage& tr, std::string* error_string) const
{
  if (tr.mode_ == EXTRAPOLATE_BACK &&  tr.stamp_ - target_time > max_extrapolation_distance_)
  {
    if (error_string) {
      std::stringstream ss;
      ss << std::fixed;
      ss.precision(3);
      ss << "Extrapolating into the past.  You requested a transform at time " << target_time.toSec() << " seconds \n"
         << "but the tf buffer only has a history of until " << tr.stamp_.toSec()  << " seconds.\n";
      if ( max_extrapolation_distance_ > ros::Duration(0))
      {
        ss << "The tf extrapollation distance is set to " 
           << (max_extrapolation_distance_).toSec() <<" seconds.\n";
      }
      *error_string = ss.str();
    }
    return true;
  }
  return false;
}


bool BufferCore::test_extrapolation_future(const ros::Time& target_time, const TransformStorage& tr, std::string* error_string) const
{
  if( tr.mode_ == EXTRAPOLATE_FORWARD && target_time - tr.stamp_ > max_extrapolation_distance_)
  {
    if (error_string){
      std::stringstream ss;
      ss << std::fixed;
      ss.precision(3);

      ss << "Extrapolating into the future.  You requested a transform that is at time" << target_time.toSec() << " seconds, \n"
         << "but the most recent transform in the tf buffer is at " << tr.stamp_.toSec() << " seconds.\n";
      if ( max_extrapolation_distance_ > ros::Duration(0))
      {
        ss << "The tf extrapollation distance is set to " 
           << (max_extrapolation_distance_).toSec() <<" seconds.\n";
      }
      *error_string = ss.str();
    }
    return true;
  }
  return false;
}


bool BufferCore::test_extrapolation(const ros::Time& target_time, const TransformLists& lists, std::string * error_string) const
{
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
  {
    if (test_extrapolation_one_value(target_time, lists.inverseTransforms[i], error_string)) return true;
    if (test_extrapolation_past(target_time, lists.inverseTransforms[i], error_string)) return true;
    if (test_extrapolation_future(target_time, lists.inverseTransforms[i], error_string)) return true;
  }

  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++)
  {
    if (test_extrapolation_one_value(target_time, lists.forwardTransforms[i], error_string)) return true;
    if (test_extrapolation_past(target_time, lists.forwardTransforms[i], error_string)) return true;
    if (test_extrapolation_future(target_time, lists.forwardTransforms[i], error_string)) return true;
  }

  return false;
}



btTransform BufferCore::computeTransformFromList(const TransformLists & lists) const
{
  btTransform retTrans;
  retTrans.setIdentity();
  ///@todo change these to iterators
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
  {
    const TransformStorage& ts = lists.inverseTransforms[lists.inverseTransforms.size() -1 - i];
    btTransform transform(ts.rotation_, ts.translation_);
    retTrans *= transform; //Reverse to get left multiply
  }
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++)
  {
    const TransformStorage& ts = lists.forwardTransforms[lists.forwardTransforms.size() -1 - i];
    btTransform transform(ts.rotation_, ts.translation_);
    retTrans = transform.inverse() * retTrans; //Do this list backwards(from backwards) for it was generated traveling the wrong way
  }

  return retTrans;
}

std::string BufferCore::allFramesAsString() const
{
  std::stringstream mstream;
  boost::mutex::scoped_lock(frame_mutex_);

  TransformStorage temp;



  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)

  ///regular transforms
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    TimeCacheInterface* frame_ptr = getFrame(CompactFrameID(counter));
    if (frame_ptr == NULL)
      continue;
    CompactFrameID frame_id_num;
    if(  frame_ptr->getData(ros::Time(), temp))
      frame_id_num = temp.frame_id_;
    else
    {
      frame_id_num = 0;
    }
    mstream << "Frame "<< frameIDs_reverse[counter] << " exists with parent " << frameIDs_reverse[frame_id_num.num_] << "." <<std::endl;
  }

  return mstream.str();
}

int BufferCore::getLatestCommonTime(const std::string& source, const std::string& dest, ros::Time & time, std::string * error_string) const
{
  if (source == dest)
  {
    //Set time to latest timestamp of frameid in case of target and source frame id are the same
    time = ros::Time();                 ///\todo review was now();
    return tf2_msgs::TF2Error::NO_ERROR;
  }

  time = ros::Time(ros::TIME_MAX);
  int retval;
  TransformLists lists;
  {
    retval = lookupLists(lookupFrameNumber(dest), ros::Time(), lookupFrameNumber(source), lists, error_string);
  }
  if (retval == tf2_msgs::TF2Error::NO_ERROR)
  {
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

std::string BufferCore::allFramesAsYAML() const
{
  std::stringstream mstream;
  boost::mutex::scoped_lock(frame_mutex_);

  TransformStorage temp;

  if (frames_.size() ==1)
    mstream <<"[]";

  mstream.precision(3);
  mstream.setf(std::ios::fixed,std::ios::floatfield);
    
   //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)//one referenced for 0 is no frame
  {
    CompactFrameID cfid = CompactFrameID(counter);
    CompactFrameID frame_id_num;
    if(  getFrame(cfid)->getData(ros::Time(), temp))
      frame_id_num = temp.frame_id_;
    else
    {
      frame_id_num = 0;
    }
    if (frame_id_num.num_ != 0)
    {
      std::string authority = "no recorded authority";
      std::map<CompactFrameID, std::string>::const_iterator it = frame_authority_.find(cfid);
      if (it != frame_authority_.end())
        authority = it->second;

      double rate = getFrame(cfid)->getListLength() / std::max((getFrame(cfid)->getLatestTimestamp().toSec() -
                                                                   getFrame(cfid)->getOldestTimestamp().toSec() ), 0.0001);

      mstream << std::fixed; //fixed point notation
      mstream.precision(3); //3 decimal places
      mstream << frameIDs_reverse[cfid.num_] << ": " << std::endl;
      mstream << "  parent: '" << frameIDs_reverse[frame_id_num.num_] << "'" << std::endl;
      mstream << "  broadcaster: '" << authority << "'" << std::endl;
      mstream << "  rate: " << rate << std::endl;
      mstream << "  most_recent_transform: " << (getFrame(cfid)->getLatestTimestamp()).toSec() << std::endl;
      mstream << "  oldest_transform: " << (getFrame(cfid)->getOldestTimestamp()).toSec() << std::endl;
      mstream << "  buffer_length: " << (getFrame(cfid)->getLatestTimestamp()-getFrame(cfid)->getOldestTimestamp()).toSec() << std::endl;
    }
  }
  
  return mstream.str();
}


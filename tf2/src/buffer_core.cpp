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
#include "tf2/time_cache.h"
#include "tf2/exceptions.h"
#include "tf2_msgs/TF2Error.h"
//legacy
//#include "tf/tf.h"
//#include "tf/transform_datatypes.h"

namespace tf2
{

// Must provide storage for non-integral static const class members.
// Otherwise you get undefined symbol errors on OS X (why not on Linux?).
// Thanks to Rob for pointing out the right way to do this.
const double tf2::BufferCore::DEFAULT_CACHE_TIME;

/** \brief convert Transform msg to Transform */
void transformMsgToTF2(const geometry_msgs::Transform& msg, btTransform& bt)
{bt = btTransform(btQuaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), btVector3(msg.translation.x, msg.translation.y, msg.translation.z));}

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
}

/** \brief convert Transform to Transform msg*/
void transformTF2ToMsg(const btTransform& bt, geometry_msgs::TransformStamped& msg, ros::Time stamp, const std::string& frame_id, const std::string& child_frame_id)
{
  transformTF2ToMsg(bt, msg.transform);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
}

void transformTF2ToMsg(const btQuaternion& orient, const btVector3& pos, geometry_msgs::Transform& msg)
{
  msg.translation.x = pos.x();
  msg.translation.y = pos.y();
  msg.translation.z = pos.z();
  msg.rotation.x = orient.x();
  msg.rotation.y = orient.y();
  msg.rotation.z = orient.z();
  msg.rotation.w = orient.w();
}

void transformTF2ToMsg(const btQuaternion& orient, const btVector3& pos, geometry_msgs::TransformStamped& msg, ros::Time stamp, const std::string& frame_id, const std::string& child_frame_id)
{
  transformTF2ToMsg(orient, pos, msg.transform);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
}

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
}


bool BufferCore::warnFrameId(const char* function_name_arg, const std::string& frame_id) const
{
  if (frame_id.size() == 0)
  {
    std::stringstream ss;
    ss << "Invalid argument passed to "<< function_name_arg <<" in tf2 frame_ids cannot be empty";
    ROS_WARN("%s",ss.str().c_str());
    return true;
  }

  if (startsWithSlash(frame_id))
  {
    std::stringstream ss;
    ss << "Invalid argument \"" << frame_id << "\" passed to "<< function_name_arg <<" in tf2 frame_ids cannot start with a '/' like: ";
    ROS_WARN("%s",ss.str().c_str());
    return true;
  }

  return false;
}

CompactFrameID BufferCore::validateFrameId(const char* function_name_arg, const std::string& frame_id) const
{
  if (frame_id.empty())
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

  CompactFrameID id = lookupFrameNumber(frame_id);
  if (id == 0)
  {
    std::stringstream ss;
    ss << "\"" << frame_id << "\" passed to "<< function_name_arg <<" does not exist. ";
    throw tf2::LookupException(ss.str().c_str());
  }
  
  return id;
}

BufferCore::BufferCore(ros::Duration cache_time)
: cache_time_(cache_time)
{
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
  
  boost::mutex::scoped_lock lock(frame_mutex_);
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

}

TimeCacheInterface* BufferCore::allocateFrame(CompactFrameID cfid, bool is_static)
{
  TimeCacheInterface* frame_ptr = frames_[cfid];
  if ( frame_ptr != NULL)
    delete frame_ptr;
  if (is_static)
    frames_[cfid] = new StaticCache();
  else
    frames_[cfid] = new TimeCache(cache_time_);
  
  return frames_[cfid];
}

enum WalkEnding
{
  Identity,
  TargetParentOfSource,
  SourceParentOfTarget,
  FullPath,
};

template<typename F>
int BufferCore::walkToTopParent(F& f, ros::Time time, CompactFrameID target_id, CompactFrameID source_id, std::string* error_string) const
{
  // Short circuit if zero length transform to allow lookups on non existant links
  if (source_id == target_id)
  {
    f.finalize(Identity, time);
    return tf2_msgs::TF2Error::NO_ERROR;
  }

  //If getting the latest get the latest common time
  if (time == ros::Time())
  {
    int retval = getLatestCommonTime(target_id, source_id, time, error_string);
    if (retval != tf2_msgs::TF2Error::NO_ERROR)
    {
      return retval;
    }
  }

  // Walk the tree to its root from the source frame, accumulating the transform
  CompactFrameID frame = source_id;
  CompactFrameID top_parent = frame;
  uint32_t depth = 0;
  while (frame != 0)
  {
    TimeCacheInterface* cache = getFrame(frame);

    if (!cache)
    {
      // There will be no cache for the very root of the tree
      top_parent = frame;
      break;
    }

    CompactFrameID parent = f.gather(cache, time, 0);
    if (parent == 0)
    {
      // Just break out here... there may still be a path from source -> target
      break;
    }

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id)
    {
      f.finalize(TargetParentOfSource, time);
      return tf2_msgs::TF2Error::NO_ERROR;
    }

    f.accum(true);

    top_parent = frame;
    frame = parent;

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return tf2_msgs::TF2Error::LOOKUP_ERROR;
    }
  }

  // Now walk to the top parent from the target frame, accumulating its transform
  frame = target_id;
  depth = 0;
  while (frame != top_parent)
  {
    TimeCacheInterface* cache = getFrame(frame);

    if (!cache)
    {
      break;
    }

    CompactFrameID parent = f.gather(cache, time, error_string);
    if (parent == 0)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << *error_string << ", when looking up transform from frame [" << lookupFrameString(source_id) << "] to frame [" << lookupFrameString(target_id) << "]";
        *error_string = ss.str();
      }

      return tf2_msgs::TF2Error::EXTRAPOLATION_ERROR;
    }

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id)
    {
      f.finalize(SourceParentOfTarget, time);
      return tf2_msgs::TF2Error::NO_ERROR;
    }

    f.accum(false);

    frame = parent;

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return tf2_msgs::TF2Error::LOOKUP_ERROR;
    }
  }

  if (frame != top_parent)
  {
    createConnectivityErrorString(source_id, target_id, error_string);
    return tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
  }

  f.finalize(FullPath, time);

  return tf2_msgs::TF2Error::NO_ERROR;
}

struct TransformAccum
{
  TransformAccum()
  : source_to_top_quat(0.0, 0.0, 0.0, 1.0)
  , source_to_top_vec(0.0, 0.0, 0.0)
  , target_to_top_quat(0.0, 0.0, 0.0, 1.0)
  , target_to_top_vec(0.0, 0.0, 0.0)
  , result_quat(0.0, 0.0, 0.0, 1.0)
  , result_vec(0.0, 0.0, 0.0)
  {
  }

  CompactFrameID gather(TimeCacheInterface* cache, ros::Time time, std::string* error_string)
  {
    if (!cache->getData(time, st, error_string))
    {
      return 0;
    }

    return st.frame_id_;
  }

  void accum(bool source)
  {
    if (source)
    {
      source_to_top_vec += quatRotate(source_to_top_quat, st.translation_);
      source_to_top_quat *= st.rotation_;
    }
    else
    {
      target_to_top_vec += quatRotate(target_to_top_quat, st.translation_);
      target_to_top_quat *= st.rotation_;
    }
  }

  void finalize(WalkEnding end, ros::Time _time)
  {
    switch (end)
    {
    case Identity:
      break;
    case TargetParentOfSource:
      result_vec = source_to_top_vec;
      result_quat = source_to_top_quat;
      break;
    case SourceParentOfTarget:
      {
        btQuaternion inv_target_quat = target_to_top_quat.inverse();
        btVector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);
        result_vec = inv_target_vec;
        result_quat = inv_target_quat;
        break;
      }
    case FullPath:
      {
        btQuaternion inv_target_quat = target_to_top_quat.inverse();
        btVector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);

        result_vec = source_to_top_vec + quatRotate(source_to_top_quat, inv_target_vec);
        result_quat = source_to_top_quat * inv_target_quat;
      }
      break;
    };

    time = _time;
  }

  TransformStorage st;
  ros::Time time;
  btQuaternion source_to_top_quat;
  btVector3 source_to_top_vec;
  btQuaternion target_to_top_quat;
  btVector3 target_to_top_vec;

  btQuaternion result_quat;
  btVector3 result_vec;
};

geometry_msgs::TransformStamped BufferCore::lookupTransform(const std::string& target_frame,
                                                            const std::string& source_frame,
                                                            const ros::Time& time) const
{
  boost::mutex::scoped_lock lock(frame_mutex_);

  CompactFrameID target_id = validateFrameId("lookupTransform argument target_frame", target_frame);
  CompactFrameID source_id = validateFrameId("lookupTransform argument source_frame", source_frame);

  std::string error_string;
  TransformAccum accum;
  int retval = walkToTopParent(accum, time, target_id, source_id, &error_string);
  if (retval != tf2_msgs::TF2Error::NO_ERROR)
  {
    switch (retval)
    {
    case tf2_msgs::TF2Error::CONNECTIVITY_ERROR:
      throw ConnectivityException(error_string);
    case tf2_msgs::TF2Error::EXTRAPOLATION_ERROR:
      throw ExtrapolationException(error_string);
    case tf2_msgs::TF2Error::LOOKUP_ERROR:
      throw LookupException(error_string);
    default:
      ROS_ERROR("Unknown error code: %d", retval);
      ROS_BREAK();
    }
  }

  geometry_msgs::TransformStamped output_transform;
  transformTF2ToMsg(accum.result_quat, accum.result_vec, output_transform, accum.time, target_frame, source_frame);
  return output_transform;
}

                                                       
geometry_msgs::TransformStamped BufferCore::lookupTransform(const std::string& target_frame, 
                                                        const ros::Time& target_time,
                                                        const std::string& source_frame,
                                                        const ros::Time& source_time,
                                                        const std::string& fixed_frame) const
{
  validateFrameId("lookupTransform argument target_frame", target_frame);
  validateFrameId("lookupTransform argument source_frame", source_frame);
  validateFrameId("lookupTransform argument fixed_frame", fixed_frame);

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
}



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
}

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
}
*/

struct CanTransformAccum
{
  CompactFrameID gather(TimeCacheInterface* cache, ros::Time time, std::string* error_string)
  {
    if (!cache->getData(time, st, error_string))
    {
      return 0;
    }

    return st.frame_id_;
  }

  void accum(bool source)
  {
  }

  void finalize(WalkEnding end, ros::Time _time)
  {
  }

  TransformStorage st;
};

bool BufferCore::canTransform(const std::string& target_frame, const std::string& source_frame,
                           const ros::Time& time, std::string* error_msg) const
{
  if (warnFrameId("canTransform argument target_frame", target_frame))
    return false;
  if (warnFrameId("canTransform argument source_frame", source_frame))
    return false;

  CompactFrameID target_id = lookupFrameNumber(target_frame);
  CompactFrameID source_id = lookupFrameNumber(source_frame);

  if (target_id == 0 || source_id == 0)
  {
    return false;
  }

  CanTransformAccum accum;
  if (walkToTopParent(accum, time, target_id, source_id, error_msg) == tf2_msgs::TF2Error::NO_ERROR)
  {
    return true;
  }

  return false;
}

bool BufferCore::canTransform(const std::string& target_frame, const ros::Time& target_time,
                          const std::string& source_frame, const ros::Time& source_time,
                          const std::string& fixed_frame, std::string* error_msg) const
{
  if (warnFrameId("canTransform argument target_frame", target_frame))
    return false;
  if (warnFrameId("canTransform argument source_frame", source_frame))
    return false;
  if (warnFrameId("canTransform argument fixed_frame", fixed_frame))
    return false;

  return canTransform(target_frame, fixed_frame, target_time) && canTransform(fixed_frame, source_frame, source_time, error_msg);
}


tf2::TimeCacheInterface* BufferCore::getFrame(CompactFrameID frame_id) const
{
  if (frame_id == 0 || frame_id > frames_.size()) /// @todo check larger values too
    return NULL;
  else
  {
    return frames_[frame_id];
  }
}

CompactFrameID BufferCore::lookupFrameNumber(const std::string& frameid_str) const
{
  CompactFrameID retval;
  boost::mutex::scoped_lock(frame_mutex_);
  M_StringToCompactFrameID::const_iterator map_it = frameIDs_.find(frameid_str);
  if (map_it == frameIDs_.end())
  {
    retval = CompactFrameID(0);
  }
  else
    retval = map_it->second;
  return retval;
}

CompactFrameID BufferCore::lookupOrInsertFrameNumber(const std::string& frameid_str)
{
  CompactFrameID retval = 0;
  M_StringToCompactFrameID::iterator map_it = frameIDs_.find(frameid_str);
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
}

std::string BufferCore::lookupFrameString(CompactFrameID frame_id_num) const
{
    if (frame_id_num >= frameIDs_reverse.size())
    {
      std::stringstream ss;
      ss << "Reverse lookup of frame id " << frame_id_num << " failed!";
      throw tf2::LookupException(ss.str());
    }
    else
      return frameIDs_reverse[frame_id_num];
}

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
    mstream << "Frame "<< frameIDs_reverse[counter] << " exists with parent " << frameIDs_reverse[frame_id_num] << "." <<std::endl;
  }

  return mstream.str();
}

struct TimeAndFrameIDFrameComparator
{
  TimeAndFrameIDFrameComparator(CompactFrameID id)
  : id(id)
  {}

  bool operator()(const P_TimeAndFrameID& rhs) const
  {
    return rhs.second == id;
  }

  CompactFrameID id;
};

int BufferCore::getLatestCommonTime(CompactFrameID source_id, CompactFrameID target_id, ros::Time & time, std::string * error_string) const
{
  if (source_id == target_id)
  {
    //Set time to latest timestamp of frameid in case of target and source frame id are the same
    time = ros::Time();                 ///\todo review was now();
    return tf2_msgs::TF2Error::NO_ERROR;
  }

  lct_cache_.clear();

  // Walk the tree to its root from the source frame, accumulating the list of parent/time as well as the latest time
  // in the target is a direct parent
  CompactFrameID frame = source_id;
  P_TimeAndFrameID temp;
  uint32_t depth = 0;
  ros::Time latest_time;
  while (frame != 0)
  {
    TimeCacheInterface* cache = getFrame(frame);

    if (!cache)
    {
      // There will be no cache for the very root of the tree
      break;
    }

    P_TimeAndFrameID latest = cache->getLatestTimeAndParent();

    if (latest.second == 0)
    {
      // Just break out here... there may still be a path from source -> target
      break;
    }

    latest_time = std::max(latest.first, latest_time);

    lct_cache_.push_back(latest);

    frame = latest.second;

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id)
    {
      time = latest_time;
      return tf2_msgs::TF2Error::NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss<<"The tf tree is invalid because it contains a loop." << std::endl
          << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return tf2_msgs::TF2Error::LOOKUP_ERROR;
    }
  }

  // Now walk to the top parent from the target frame, accumulating the latest time and looking for a common parent
  frame = target_id;
  depth = 0;
  latest_time = ros::Time();
  CompactFrameID common_parent = 0;
  while (true)
  {
    TimeCacheInterface* cache = getFrame(frame);

    if (!cache)
    {
      break;
    }

    P_TimeAndFrameID latest = cache->getLatestTimeAndParent();

    if (latest.second == 0)
    {
      break;
    }

    latest_time = std::max(latest.first, latest_time);

    std::vector<P_TimeAndFrameID>::iterator it = std::find_if(lct_cache_.begin(), lct_cache_.end(), TimeAndFrameIDFrameComparator(latest.second));
    if (it != lct_cache_.end()) // found a common parent
    {
      common_parent = it->second;
      break;
    }

    frame = latest.second;

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id)
    {
      time = latest_time;
      return tf2_msgs::TF2Error::NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss<<"The tf tree is invalid because it contains a loop." << std::endl
          << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return tf2_msgs::TF2Error::LOOKUP_ERROR;
    }
  }

  if (common_parent == 0)
  {
    createConnectivityErrorString(source_id, target_id, error_string);
    return tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
  }

  // Loop through the source -> root list until we hit the common parent
  {
    std::vector<P_TimeAndFrameID>::iterator it = lct_cache_.begin();
    std::vector<P_TimeAndFrameID>::iterator end = lct_cache_.end();
    for (; it != end; ++it)
    {
      if (it->second == common_parent)
      {
        break;
      }

      latest_time = std::max(latest_time, it->first);
    }
  }

  time = latest_time;
  return tf2_msgs::TF2Error::NO_ERROR;
}

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
    TimeCacheInterface* cache = getFrame(cfid);
    if (!cache)
    {
      continue;
    }

    if(!cache->getData(ros::Time(), temp))
    {
      continue;
    }

    frame_id_num = temp.frame_id_;

    std::string authority = "no recorded authority";
    std::map<CompactFrameID, std::string>::const_iterator it = frame_authority_.find(cfid);
    if (it != frame_authority_.end())
      authority = it->second;

    double rate = getFrame(cfid)->getListLength() / std::max((getFrame(cfid)->getLatestTimestamp().toSec() -
                                                                 getFrame(cfid)->getOldestTimestamp().toSec() ), 0.0001);

    mstream << std::fixed; //fixed point notation
    mstream.precision(3); //3 decimal places
    mstream << frameIDs_reverse[cfid] << ": " << std::endl;
    mstream << "  parent: '" << frameIDs_reverse[frame_id_num] << "'" << std::endl;
    mstream << "  broadcaster: '" << authority << "'" << std::endl;
    mstream << "  rate: " << rate << std::endl;
    mstream << "  most_recent_transform: " << (getFrame(cfid)->getLatestTimestamp()).toSec() << std::endl;
    mstream << "  oldest_transform: " << (getFrame(cfid)->getOldestTimestamp()).toSec() << std::endl;
    mstream << "  buffer_length: " << (getFrame(cfid)->getLatestTimestamp()-getFrame(cfid)->getOldestTimestamp()).toSec() << std::endl;
  }
  
  return mstream.str();
}

} // namespace tf2

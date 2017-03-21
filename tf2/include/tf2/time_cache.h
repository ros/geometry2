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

#ifndef TF2_TIME_CACHE_H
#define TF2_TIME_CACHE_H

#include "transform_storage.h"

#include <memory>
#include <list>
#include <sstream>

#include <tf2/visibility_control.h>

namespace tf2
{

typedef std::pair<TimePoint, CompactFrameID> P_TimeAndFrameID;

class TimeCacheInterface
{
public:
  /** \brief Access data from the cache */
  TF2_PUBLIC
  virtual bool getData(TimePoint time, TransformStorage & data_out, std::string* error_str = 0)=0; //returns false if data unavailable (should be thrown as lookup exception

  /** \brief Insert data into the cache */
  TF2_PUBLIC
  virtual bool insertData(const TransformStorage& new_data)=0;

  /** @brief Clear the list of stored values */
  TF2_PUBLIC
  virtual void clearList()=0;

  /** \brief Retrieve the parent at a specific time */
  TF2_PUBLIC
  virtual CompactFrameID getParent(TimePoint time, std::string* error_str) = 0;

  /**
   * \brief Get the latest time stored in this cache, and the parent associated with it.  Returns parent = 0 if no data.
   */
  TF2_PUBLIC
  virtual P_TimeAndFrameID getLatestTimeAndParent() = 0;


  /// Debugging information methods
  /** @brief Get the length of the stored list */
  TF2_PUBLIC
  virtual unsigned int getListLength()=0;

  /** @brief Get the latest timestamp cached */
  TF2_PUBLIC
  virtual TimePoint getLatestTimestamp()=0;

  /** @brief Get the oldest timestamp cached */
  TF2_PUBLIC
  virtual TimePoint getOldestTimestamp()=0;
};

using TimeCacheInterfacePtr = std::shared_ptr<TimeCacheInterface>;

constexpr tf2::Duration TIMECACHE_DEFAULT_MAX_STORAGE_TIME = std::chrono::seconds(10); //!< default value of 10 seconds storage

/** \brief A class to keep a sorted linked list in time
 * This builds and maintains a list of timestamped
 * data.  And provides lookup functions to get
 * data out as a function of time. */
class TimeCache : public TimeCacheInterface
{
 public:
  TF2_PUBLIC
  static const int MIN_INTERPOLATION_DISTANCE = 5; //!< Number of nano-seconds to not interpolate below.
  TF2_PUBLIC
  static const unsigned int MAX_LENGTH_LINKED_LIST = 1000000; //!< Maximum length of linked list, to make sure not to be able to use unlimited memory.

  TF2_PUBLIC
  TimeCache(tf2::Duration  max_storage_time = TIMECACHE_DEFAULT_MAX_STORAGE_TIME);


  /// Virtual methods

  TF2_PUBLIC
  virtual bool getData(TimePoint time, TransformStorage & data_out, std::string* error_str = 0);
  TF2_PUBLIC
  virtual bool insertData(const TransformStorage& new_data);
  TF2_PUBLIC
  virtual void clearList();
  TF2_PUBLIC
  virtual CompactFrameID getParent(TimePoint time, std::string* error_str);
  TF2_PUBLIC
  virtual P_TimeAndFrameID getLatestTimeAndParent();

  /// Debugging information methods
  TF2_PUBLIC
  virtual unsigned int getListLength();
  TF2_PUBLIC
  virtual TimePoint getLatestTimestamp();
  TF2_PUBLIC
  virtual TimePoint getOldestTimestamp();
  

private:
  typedef std::list<TransformStorage> L_TransformStorage;
  L_TransformStorage storage_;

  tf2::Duration max_storage_time_;


  /// A helper function for getData
  //Assumes storage is already locked for it
  inline uint8_t findClosest(TransformStorage*& one, TransformStorage*& two, TimePoint target_time, std::string* error_str);

  inline void interpolate(const TransformStorage& one, const TransformStorage& two, TimePoint time, TransformStorage& output);


  void pruneList();



};

class StaticCache : public TimeCacheInterface
{
 public:
  /// Virtual methods

  TF2_PUBLIC
  virtual bool getData(TimePoint time, TransformStorage & data_out, std::string* error_str = 0); //returns false if data unavailable (should be thrown as lookup exception
  TF2_PUBLIC
  virtual bool insertData(const TransformStorage& new_data);
  TF2_PUBLIC
  virtual void clearList();
  TF2_PUBLIC
  virtual CompactFrameID getParent(TimePoint time, std::string* error_str);
  TF2_PUBLIC
  virtual P_TimeAndFrameID getLatestTimeAndParent();


  /// Debugging information methods
  TF2_PUBLIC
  virtual unsigned int getListLength();
  TF2_PUBLIC
  virtual TimePoint getLatestTimestamp();
  TF2_PUBLIC
  virtual TimePoint getOldestTimestamp();
  

private:
  TransformStorage  storage_;
};

}

#endif // TF2_TIME_CACHE_H

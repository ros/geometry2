/*
 * Copyright (c) 2015, Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <iomanip>
#include <ctime>
#include <thread>

namespace tf2
{
  using Duration = std::chrono::duration<double, std::nano>;
  using TimePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;


  using IDuration = std::chrono::duration<int, std::nano>;
  // This is the zero time in ROS
  static const TimePoint TimePointZero = TimePoint(IDuration::zero());

  inline TimePoint get_now()
  {
    return std::chrono::system_clock::now();
  }

  inline double durationToSec(const tf2::Duration & input){
    return (double)std::chrono::duration_cast<std::chrono::seconds>(input).count();
  }
  
  inline double timeToSec(const TimePoint& timepoint)
  {
    return durationToSec(Duration(timepoint.time_since_epoch()));
  }

  // Display functions as there is no default display
  // TODO: find a proper way to handle display
  inline std::string displayTimePoint(const TimePoint& stamp)
  {
    // Below would only work with GCC 5.0 and above
    //return std::put_time(&stamp, "%c");
    std::time_t time = std::chrono::system_clock::to_time_t(std::chrono::time_point_cast<std::chrono::milliseconds>(stamp));
#ifdef __STDC_LIB_EXT1__
    // Result string will not be longer than "Www Mmm dd hh:mm:ss yyyy\n"
    char time_point_s[26];
    std::memset(time_point_s, 0, sizeof time_point_s);
    // TODO(esteve): check errno_t
    std::ctime_s(time_point_s, sizeof time_point_s, &time);
    return std::string(time_point_s);
#else
    return std::ctime(&time);
#endif
  }


}

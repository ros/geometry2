/*
 * Copyright (c) 2015-2016, Open Source Robotics Foundation, Inc.
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

 #ifndef TF2_TIME_H
 #define TF2_TIME_H

#include <chrono>
#include <new>
#include <stdio.h>
#include <string>
#include <string.h>
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

  inline std::string displayTimePoint(const TimePoint& stamp)
  {
    const char * format_str = "%.6f";
    double current_time = timeToSec(stamp);
    int buff_size = snprintf(NULL, 0, format_str, current_time);
    if (buff_size < 0) {
      throw std::runtime_error(strerror(errno));
    }
    char * buffer = new char[buff_size];
    int bytes_written = snprintf(buffer, buff_size, format_str, current_time);
    if (bytes_written < 0) {
      throw std::runtime_error(strerror(errno));
    }
    std::string result = std::string(buffer);
    delete[] buffer;
    return result;
  }
}

#endif // TF2_TIME_H

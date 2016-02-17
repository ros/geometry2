/*
 * Copyright (c) 2016, Open Source Robotics Foundation, Inc.
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

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#include "tf2/time.h"

using namespace tf2;

std::string tf2::displayTimePoint(const TimePoint& stamp)
{
  const char * format_str = "%.6f";
  double current_time = timeToSec(stamp);
  int buff_size = snprintf(NULL, 0, format_str, current_time);
  if (buff_size < 0) {
#ifdef _WIN32
    // Using fixed buffer size since, strerrorlen_s not yet available
    const int errormsglen = 200;
    char errmsg[errormsglen];
    strerror_s(errmsg, errormsglen, errno);
    throw std::runtime_error(errmsg);
#else
    throw std::runtime_error(strerror(errno));
#endif // _WIN32
}

  char * buffer = new char[buff_size];
  int bytes_written = snprintf(buffer, buff_size, format_str, current_time);
  if (bytes_written < 0) {
#ifdef _WIN32
    // Using fixed buffer size since, strerrorlen_s not yet available
    const int errormsglen = 200;
    char errmsg[errormsglen];
    strerror_s(errmsg, errormsglen, errno);
    throw std::runtime_error(errmsg);
#else
    throw std::runtime_error(strerror(errno));
#endif // _WIN32
  }
  std::string result = std::string(buffer);
  delete[] buffer;
  return result;
}

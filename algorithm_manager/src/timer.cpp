// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "algorithm_manager/timer.hpp"

namespace cyberdog
{
namespace algorithm
{

Timer::Timer()
: started_(false), paused_(false) {}

void Timer::Start()
{
  started_ = true;
  paused_ = false;
  start_time_ = std::chrono::high_resolution_clock::now();
}

void Timer::Restart()
{
  started_ = false;
  Start();
}

void Timer::Pause()
{
  paused_ = true;
  pause_time_ = std::chrono::high_resolution_clock::now();
}

void Timer::Resume()
{
  paused_ = false;
  start_time_ += std::chrono::high_resolution_clock::now() - pause_time_;
}

void Timer::Reset()
{
  started_ = false;
  paused_ = false;
}

double Timer::ElapsedMicroSeconds() const
{
  if (!started_) {
    return 0.0;
  }
  if (paused_) {
    return std::chrono::duration_cast<std::chrono::microseconds>(
      pause_time_ - start_time_).count();
  } else {
    return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - start_time_).count();
  }
}

double Timer::ElapsedSeconds() const {return ElapsedMicroSeconds() / 1e6;}

double Timer::ElapsedMinutes() const {return ElapsedSeconds() / 60;}

double Timer::ElapsedHours() const {return ElapsedMinutes() / 60;}


}  // namespace algorithm
}  // namespace cyberdog

/*
 * Copyright 2021 Clearpath Robotics, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef OFFLOAD_SERVER__NODE__UTILS_HPP_
#define OFFLOAD_SERVER__NODE__UTILS_HPP_

#include <string>
#include <vector>
#include <functional>
#include <chrono>
#include <map>

namespace offload_server
{

enum class OffloadTaskStatus
{
  UNKNOWN = 0,
  OFF = 1,
  IN_PROGRESS = 2,
  CANCELLED = 3,
  COMPLETED = 4
};


enum class ServerSchedulingAlgo
{
  FIFO_QUEUE = 0,
  ROUND_ROBIN = 1,
  RMS = 2,
  EDF = 3,
  LSTF = 4,
};


static std::map<ServerSchedulingAlgo, std::string> SchedAlgoName
{
  {ServerSchedulingAlgo::FIFO_QUEUE, "fifo"},
  {ServerSchedulingAlgo::ROUND_ROBIN, "round_robin"},
  {ServerSchedulingAlgo::RMS, "rms"},
  {ServerSchedulingAlgo::EDF, "edf"},
  {ServerSchedulingAlgo::LSTF, "lstf"}
};

typedef std::function<void (void)> offload_server_function_callback_t;
typedef std::function<void (std::string)> offload_server_function_call_callback_t;

}  // namespace offload_server

#endif  // OFFLOAD_SERVER_NODE__UTILS_HPP_

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

#include <rcutils/cmdline_parser.h>

#include <chrono>
#include <memory>
#include <string>
#include <csignal>

#include <rclcpp/rclcpp.hpp>

#include "offload_server/offload_server.hpp"
#include "offload_server/utils.hpp"

std::shared_ptr<offload_server::OffloadServer> server;

void signalHandler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        RCLCPP_INFO(rclcpp::get_logger("offload_server"), "Signal received, shutting down...");
        if (server) {
            server->stop();
        }
        rclcpp::shutdown();
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  server = std::make_shared<offload_server::OffloadServer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);

  RCLCPP_INFO(rclcpp::get_logger("offload_server"), "Starting OffloadServer...");
  executor.spin();

  RCLCPP_INFO(rclcpp::get_logger("offload_server"), "Shutting down OffloadServer...");
  rclcpp::shutdown();

  return 0;
}

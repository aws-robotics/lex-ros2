/*
 * Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#include <rclcpp/rclcpp.hpp>

#include <aws_common/sdk_utils/client_configuration_provider.h>

#include <aws/core/config/AWSProfileConfigLoader.h>
#include <aws/core/Aws.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws/core/utils/logging/AWSLogging.h>

#include <aws_ros2_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>

#include <lex_common/error_codes.h>
#include <lex_common/lex_common.h>

#include <lex_node/lex_node.h>

#include <memory>


void shutdown(const Aws::SDKOptions & options)
{
  AWS_LOG_INFO(__func__, "Shutting down Lex Node...");
  Aws::Utils::Logging::ShutdownAWSLogging();
  Aws::ShutdownAPI(options);
  rclcpp::shutdown();
}
/**
 * Start the lex node program.
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);

  auto lex_node = std::make_shared<Aws::Lex::LexNode>(node_options);
  Aws::Utils::Logging::InitializeAWSLogging(
    Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>("lex_node",
    Aws::Utils::Logging::LogLevel::Debug, lex_node));
  Aws::SDKOptions options;
  Aws::InitAPI(options);
  {
    // Build a lex interactor and give it to the lex node to use it.
    // Lex has an internal conversation session, therefore the lex interactor
    // should only be available for use by one point of entry.
    auto lex_interactor = std::make_shared<Aws::Lex::LexInteractor>();
    auto params = std::make_shared<Aws::Client::Ros2NodeParameterReader>(lex_node);
    auto error_code = Aws::Lex::BuildLexInteractor(params, *lex_interactor);
    if (error_code != Aws::Lex::ErrorCode::SUCCESS) {
      shutdown(options);
      return error_code;
    }
    lex_node->Init(lex_interactor);
  }
  AWS_LOG_INFO(__func__, "Starting Lex Node...");

  // blocking here, waiting until shutdown.
  rclcpp::spin(lex_node);
  shutdown(options);

  return Aws::Lex::ErrorCode::SUCCESS;
}

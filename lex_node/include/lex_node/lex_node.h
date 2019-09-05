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

#ifndef LEX_NODE__LEX_NODE_H_
#define LEX_NODE__LEX_NODE_H_

#include <aws/lex/LexRuntimeServiceClient.h>
#include <lex_common_msgs/srv/audio_text_conversation.hpp>
#include <lex_common/lex_param_helper.h>
#include <lex_common/lex_common.h>

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace Aws
{
namespace Lex
{

class LexNode;

/**
 * LexNode is responsible for providing ROS2 API's and configuration for Amazon Lex.
 * The lex node will work on each incoming message serially and respond with the lex info.
 * @todo decide how the lex node will handle multiple requests.
 */
class LexNode : public rclcpp::Node
{
private:
  /**
   * Post content function.
   */
  std::shared_ptr<PostContentInterface> post_content_;

  /**
   * The ros server for lex requests.
   */
  std::shared_ptr<rclcpp::Service<lex_common_msgs::srv::AudioTextConversation>> lex_server_;

  /**
   * Service callback for lex. Only allow one interaction with Lex at a time. If a new request comes in,
   * fail the last request, then make a new request.
   *
   * @param request to handle
   * @param response to fill
   */
  void LexServerCallback(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<lex_common_msgs::srv::AudioTextConversation::Request> request,
    std::shared_ptr<lex_common_msgs::srv::AudioTextConversation::Response> response);

public:
  /**
   * Constructor.
   */
  LexNode(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());

  /**
   * Destructor.
   */
  ~LexNode() = default;

  /**
   * Initialize the lex node.
   *
   * @param post_content interface to post the lex callbacks to
   * @return SUCCESS if initialized properly
   */
  ErrorCode Init(std::shared_ptr<PostContentInterface> post_content);
};

}  // namespace Lex
}  // namespace Aws

#endif  // LEX_NODE__LEX_NODE_H_

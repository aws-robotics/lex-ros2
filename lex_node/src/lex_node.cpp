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

#include <aws/core/utils/logging/AWSLogging.h>

#include <lex_common_msgs/msg/key_value.hpp>

#include <lex_node/lex_node.h>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace Aws
{
namespace Lex
{

using AudioTextConversation = lex_common_msgs::srv::AudioTextConversation;

LexNode::LexNode(rclcpp::NodeOptions node_options)
: Node("lex_node", std::string(), node_options) {}

LexRequest & operator<<(
  LexRequest & out_request,
  const AudioTextConversation::Request & ros_request)
{
  out_request.accept_type = ros_request.accept_type;
  out_request.audio_request = ros_request.audio_request;
  out_request.content_type = ros_request.content_type;
  out_request.text_request = ros_request.text_request;
  return out_request;
}

AudioTextConversation::Response & operator<<(
  AudioTextConversation::Response & ros_response,
  const LexResponse & lex_response)
{
  ros_response.audio_response = lex_response.audio_response;
  ros_response.dialog_state = lex_response.dialog_state;
  ros_response.intent_name = lex_response.intent_name;
  ros_response.message_format_type = lex_response.message_format_type;
  ros_response.text_response = lex_response.text_response;
  ros_response.slots = std::vector<lex_common_msgs::msg::KeyValue>();
  std::transform(lex_response.slots.begin(), lex_response.slots.end(),
    std::back_inserter(ros_response.slots), [](auto & slot) {
      lex_common_msgs::msg::KeyValue key_value;
      key_value.key = slot.first;
      key_value.value = slot.second;
      return key_value;
    });
  return ros_response;
}

ErrorCode LexNode::Init(std::shared_ptr<PostContentInterface> post_content)
{
  if (!post_content) {
    return INVALID_ARGUMENT;
  }
  RCLCPP_DEBUG(this->get_logger(), "Initialized Lex Node");
  post_content_ = std::move(post_content);
  using namespace std::placeholders;
  auto service_func = std::bind(&LexNode::LexServerCallback, this, _1, _2, _3);
  lex_server_ = this->create_service<AudioTextConversation>("lex_conversation",
      service_func);
  return SUCCESS;
}

void LexNode::LexServerCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AudioTextConversation::Request> request,
  std::shared_ptr<AudioTextConversation::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "Receiving callback");
  LexRequest lex_request;
  lex_request << *request;
  LexResponse lex_response;
  ErrorCode result = post_content_->PostContent(lex_request, lex_response);
  if (!result) {
    *response << lex_response;
  } else {
    response->error_code = result;
    response->error_message = "Failed to interact with lex, check logs for more information.";
  }
  RCLCPP_DEBUG(this->get_logger(), "Responding to callback");
}

}  // namespace Lex
}  // namespace Aws

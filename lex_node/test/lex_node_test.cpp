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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <lex_common/error_codes.h>
#include <lex_common/lex_common.h>
#include <lex_common_msgs/srv/audio_text_conversation.hpp>
#include <lex_node/lex_node.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using testing::Return;
using testing::Invoke;
using testing::ElementsAreArray;
using testing::UnorderedElementsAreArray;
using testing::_;

using lex_common_msgs::srv::AudioTextConversation;
using Aws::Lex::ErrorCode;

rclcpp::Logger logger = rclcpp::get_logger("lex_node_test");

class MockPostContentInterface : public Aws::Lex::PostContentInterface
{
public:
  MOCK_METHOD2(PostContent,
    ErrorCode(const Aws::Lex::LexRequest & request, Aws::Lex::LexResponse & response));
};

typedef std::pair<std::string, std::string> SlotPair;

/**
 * Define a struct to convert between SlotPair and KeyValue.
 */
struct PairKeyValue
{
  SlotPair data;
  mutable lex_common_msgs::msg::KeyValue key_value;

  explicit PairKeyValue(const SlotPair & data_pair)
  {
    data = data_pair;
  }

  operator lex_common_msgs::msg::KeyValue & () const {
    key_value.key = data.first;
    key_value.value = data.second;
    return key_value;
  }
  operator SlotPair &() {
    return data;
  }
};

/**
 * Tests the creation of a Lex node instance with invalid parameters
 */
TEST(LexNodeSuite, BuildLexNodeWithEmptyParams)
{
  Aws::Lex::LexNode lex_node;
  ErrorCode error = lex_node.Init(nullptr);
  ASSERT_EQ(ErrorCode::INVALID_ARGUMENT, error);
}

/**
 * Spin up a lex node and initialize it with the mock_post_content.
 * Will Fail the running test should a timeout occur.
 *
 * @param mock_post_content for the lex node to call
 * @param test_request
 * @param test_result [out]
 */
void ExecuteLexServiceTest(
  const std::shared_ptr<MockPostContentInterface> & mock_post_content,
  const std::shared_ptr<AudioTextConversation::Request> & test_request,
  std::shared_ptr<AudioTextConversation::Response> & test_result)
{
  auto lex_node = std::make_shared<Aws::Lex::LexNode>();

  ErrorCode error = lex_node->Init(mock_post_content);
  ASSERT_EQ(ErrorCode::SUCCESS, error);

  using rclcpp::executors::SingleThreadedExecutor;
  SingleThreadedExecutor executor;
  auto test_node = std::make_shared<rclcpp::Node>("test_node");
  executor.add_node(lex_node);
  auto timeout = std::chrono::seconds(20);

  std::thread executor_thread(
    std::bind(&SingleThreadedExecutor::spin, &executor));

  auto client = test_node->create_client<lex_common_msgs::srv::AudioTextConversation>(
    "lex_conversation");
  client->wait_for_service(timeout);
  ASSERT_TRUE(client->service_is_ready()) << "Lex node service was not ready in time";
  RCLCPP_DEBUG(test_node->get_logger(), "Sending lex request");
  auto result_future = client->async_send_request(test_request);

  if (rclcpp::spin_until_future_complete(test_node, result_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    FAIL() << "Service call failed";
  }
  executor.cancel();
  executor_thread.join();
  RCLCPP_DEBUG(test_node->get_logger(), "Lex request complete");
  test_result = result_future.get();
}

/**
 * Tests error code from lex node when there was a failure to post content.
 */
TEST(LexNodeSuite, TestLexServiceFailedPostContent)
{
  auto test_request = std::make_shared<AudioTextConversation::Request>();
  test_request->text_request = "text_request_test";
  test_request->content_type = "content_type_test";
  test_request->accept_type = "accept_type_test";
  test_request->audio_request = std::vector<std::uint8_t>{1, 2, 3};
  auto mock_post_content = std::make_shared<MockPostContentInterface>();
  EXPECT_CALL(*mock_post_content, PostContent(_, _))
  .WillOnce(Return(ErrorCode::FAILED_POST_CONTENT));
  std::shared_ptr<AudioTextConversation::Response> result;
  ExecuteLexServiceTest(mock_post_content, test_request, result);
  EXPECT_EQ(ErrorCode::FAILED_POST_CONTENT, (ErrorCode) result->error_code);
}

/**
 * Tests successful interaction with the lex node service.
 */
TEST(LexNodeSuite, TestLexServiceSuccess)
{
  RCLCPP_DEBUG(logger, "Starting TestLexServiceSuccess");
  auto test_request = std::make_shared<AudioTextConversation::Request>();
  test_request->text_request = "text_request_test";
  test_request->content_type = "content_type_test";
  test_request->accept_type = "accept_type_test";
  test_request->audio_request = std::vector<std::uint8_t>{1, 2, 3};

  Aws::Lex::LexResponse test_response;
  test_response.text_response = "text_response_test";
  test_response.message_format_type = "message_format_type_test";
  test_response.intent_name = "intent_name_test";
  test_response.dialog_state = "dialog_state_test";
  test_response.audio_response = std::vector<std::uint8_t>{4, 5};
  test_response.session_attributes = "session_attributes_test";
  test_response.slots = {{"slot_1_key", "slot_1_value"}, {"slot_2_key", "slot_2_value"}};
  std::vector<PairKeyValue> slots;
  std::transform(test_response.slots.begin(), test_response.slots.end(), std::back_inserter(
      slots), [](auto pair) {
      return PairKeyValue(pair);
    });
  auto mock_post_content = std::make_shared<MockPostContentInterface>();
  auto record_content = [&test_request, &test_response](const Aws::Lex::LexRequest & request,
      Aws::Lex::LexResponse & response) -> ErrorCode {
      EXPECT_EQ(test_request->content_type, request.content_type);
      EXPECT_EQ(test_request->text_request, request.text_request);
      EXPECT_EQ(test_request->audio_request.size(), request.audio_request.size());
      EXPECT_THAT(request.audio_request, ElementsAreArray(test_request->audio_request));

      response.text_response = test_response.text_response;
      response.message_format_type = test_response.message_format_type;
      response.intent_name = test_response.intent_name;
      response.dialog_state = test_response.dialog_state;
      response.audio_response = test_response.audio_response;
      response.session_attributes = test_response.session_attributes;
      response.slots = test_response.slots;
      return ErrorCode::SUCCESS;
    };
  EXPECT_CALL(*mock_post_content, PostContent(_, _)).WillOnce(Invoke(record_content));
  std::shared_ptr<AudioTextConversation::Response> result;
  ExecuteLexServiceTest(mock_post_content, test_request, result);

  EXPECT_EQ(test_response.text_response, result->text_response);
//  EXPECT_EQ(test_response.session_attributes, result->session_attributes);
  ASSERT_THAT(result->audio_response, ElementsAreArray(test_response.audio_response));
  EXPECT_EQ(test_response.dialog_state, result->dialog_state);
  EXPECT_EQ(test_response.intent_name, result->intent_name);
  EXPECT_EQ(test_response.message_format_type, result->message_format_type);
  ASSERT_THAT(result->slots, UnorderedElementsAreArray(slots));
}

/**
 * Interrupt handler to ensure rclcpp is shutdown.
 *
 * @param signum
 */
void h_sig_sigint(int signum)
{
  std::cout << "Signal interrupt" << std::endl;
  RCLCPP_ERROR(logger, "Receive signum: %i", signum);
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  signal(SIGINT, h_sig_sigint);
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

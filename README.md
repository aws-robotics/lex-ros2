# lex_node

## Overview
The ROS `lex_node` node enables a robot to comprehend natural language commands by voice or textual input and respond through a set of actions, which an AWS Lex Bot maps to ROS messages. Out of the box this node provides a ROS interface to communicate with a specified Amazon Lex bot (configured via sample_configuration.yaml) and requires configuration of AWS credentials. The Amazon Lex bot needs to be defined with responses and slots for customer prompts. A set of default slots and mappings are demonstrated in the [sample app] and include actions as “Create <location_name>,” “Go to <location_name>” and “Stop.” Additional guides on configuring bots with are available at [Getting Started with Amazon Lex].

Delivering a voice-enabled customer experience (e.g. “Robot, go to x”) will require dialog facilitation, wake word, and offline processing which are not yet provided by this integration. A wake word would trigger the dialog facilitation node to start recording and send the audio to Amazon Lex, then prompt the user for more information should Amazon Lex require it.

The ROS `lex_node`  wraps the [aws-sdk-c++] in a ROS service API.

**Amazon Lex Summary**: Amazon Lex is a service for building conversational interfaces into any application using voice and text. Amazon Lex provides the advanced deep learning functionality of automatic speech recognition (ASR) for converting speech to text, and natural language understanding (NLU) to recognize the intent of the text, to enable you to build applications with highly engaging user experiences and lifelike conversational interactions. With Amazon Lex, the same deep learning technologies that power Amazon Alexa are now available to any developer, enabling you to quickly and easily build sophisticated, natural language, conversational bots (“chatbots”).

### License
The source code is released under an [Apache 2.0].

**Author**: AWS RoboMaker<br/>
**Affiliation**: [Amazon Web Services (AWS)]<br/>
**Maintainer**: AWS RoboMaker, ros-contributions@amazon.com

### Supported ROS Distributions
- Dashing 

### Build status

* Travis CI:
    * "master" branch [![Build Status](https://travis-ci.org/aws-robotics/lex-ros2.svg?branch=master)](https://travis-ci.org/aws-robotics/lex-ros2/branches)
    * "release-latest" branch [![Build Status](https://travis-ci.org/aws-robotics/lex-ros2.svg?branch=release-latest)](https://travis-ci.org/aws-robotics/lex-ros2/branches)
 * ROS build farm:
   * ROS Dashing @ u18.04 Bionic [![Build Status](http://build.ros2.org/job/Dbin_uB64__lex_node__ubuntu_bionic_amd64__binary/badge/icon)](http://build.ros2.org/job/Dbin_uB64__lex_node__ubuntu_bionic_amd64__binary)

## Installation

### AWS Credentials
You will need to create an AWS Account and configure the credentials to be able to communicate with AWS services. You may find [AWS Configuration and Credential Files] helpful.

This node requires an IAM User with the following permission policy:
- `AmazonLexRunBotsOnly`

### Binaries
On Ubuntu you can install the latest version of this package using the following command

        sudo apt-get update
        sudo apt-get install -y ros-$ROS_DISTRO-lex-node

### Building from Source

To build from source you'll need to create a new workspace, clone and checkout the latest release branch of this repository, install all the dependencies, and compile. If you need the latest development features you can clone from the `master` branch instead of the latest release branch. While we guarantee the release branches are stable, __the `master` should be considered to have an unstable build__ due to ongoing development. 

- Create a ROS workspace and a source directory

    mkdir -p ~/ros-workspace/src

- Clone the package into the source directory . 

        cd ~/ros-workspace/src
        git clone https://github.com/aws-robotics/lex-ros2.git -b release-latest

- Install dependencies

        cd ~/ros-workspace 
        sudo apt-get update && rosdep update
        rosdep install --from-paths src --ignore-src -r -y
        
_Note: If building the master branch instead of a release branch you may need to also checkout and build the master branches of the packages this package depends on._

- Build the packages

        cd ~/ros-workspace && colcon build

- Configure ROS library Path

        source ~/ros-workspace/install/local_setup.bash

- Build and run the unit tests

        colcon test --packages-select lex_node && colcon test-result --all


## Launch Files
An example launch file called `lex_node.launch.py` is provided.

## Usage

### Resource Setup
1. Go to Amazon Lex
2. Create sample bot: BookTrip
3. Select publish, create a new alias
4. Modify the configuration file in `config/sample_configuration.yaml` to reflect the new alias

### Run the node
  - `ros2 launch lex_node lex.launch.py`

### Send a test voice message 
    `ros2 service call /lex_conversation lex_common_msgs/AudioTextConversation "{content_type: 'text/plain; charset=utf-8', accept_type: 'text/plain; charset=utf-8', text_request: 'make a reservation'}"`

### Verify the test voice was received
- Receive response from Amazon Lex and continue conversation


## Configuration File and Parameters
An example configuration file called `sample_configuration.yaml` is provided. 

**Client Configuration**  
**Namespace**:

| Name | Type |
| --- | --- |
| region | *String* |
| userAgent | *String* |
| endpointOverride | *String* |
| proxyHost | *String* |
| proxyUserName | *String* |
| proxyPassword | *String* |
| caPath | *String* |
| caFile | *String* |
| requestTimeoutMs | *int* |
| connectTimeoutMs | *int* |
| maxConnections | *int* |
| proxyPort | *int* |
| useDualStack | *bool* |
| enableClockSkewAdjustment | *bool* |
| followRedirects | *bool* |

**Amazon Lex Configuration**  
**Namespace**:

| Key | Type | Description |
| --- | ---- | ---- |
| user_id | *string* | e.g. “lex_node” | 
| bot_name | *string* | e.g. “BookTrip” (corresponds to Amazon Lex bot) | 
| bot_alias | *string* | e.g. “Demo” | 

## Node

### lex_node
Enables a robot to comprehend natural language commands by voice or textual input and respond through a set of actions.

#### Services
**Topic**: /lex_conversation

#### AudioTextConversation
**Request**:

| Key | Type | Description |
| --- | ---- | ----------- |
| content_type | *string* | The input data type to request Amazon Lex |
| accept_type | *string* | The Amazon Lex output data type desired |
| text_request | *string* | Input text data for Lex |
| audio_request | *uint8[]* | Common audio msg format, input audio data for Lex |

**Response**:

| Key | Type | Description |
| --- | ---- | ----------- |
|text_response | *string* | Output text from Lex, if accept type was text |
|audio_response | *uint8[]* | Output audio data from Lex, if accept type was audio |
|slots | *KeyValuePair[]*| Slots returned from Lex |
|intent_name | *string* | The intent Amazon Lex is attempting to fulfill |
|message_format_type | *string* | Format of output data from Lex |
|dialog_state | *string* | Amazon Lex internal dialog_state |

#### Subscribed Topics
None

#### Published Topics
None


## Bugs & Feature Requests
Please contact the team directly if you would like to request a feature.

Please report bugs in [Issue Tracker].


[Amazon Web Services (AWS)]: https://aws.amazon.com/
[Apache 2.0]: https://aws.amazon.com/apache-2-0/
[AWS Configuration and Credential Files]: https://docs.aws.amazon.com/cli/latest/userguide/cli-config-files.html
[aws-sdk-c++]: https://github.com/aws/aws-sdk-cpp
[Getting Started with Amazon Lex]: https://docs.aws.amazon.com/lex/latest/dg/getting-started.html
[Issue Tracker]: https://github.com/aws-robotics/lex-ros2/issues
[ROS]: http://www.ros.org
[sample app]: https://github.com/aws/aws-robomaker-sample-application-voiceinteraction

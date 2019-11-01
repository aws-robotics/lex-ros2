^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lex_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.0 (2019-11-01)
------------------
* Merge pull request `#9 <https://github.com/aws-robotics/lex-ros2/issues/9>`_ from aws-robotics/bump-version
  3.1.0
* 3.1.0
* Merge pull request `#8 <https://github.com/aws-robotics/lex-ros2/issues/8>`_ from aws-robotics/add-region-userid
  Allow setting region or user_id from env var or param
* Allow setting region or user_id from env var or param
  - Allow launching this node with a specific aws region or user_id by
  setting ROS_AWS_REGION or LEX_USER_ID or passing them as parameters to
  the launch file.
* Merge pull request `#7 <https://github.com/aws-robotics/lex-ros2/issues/7>`_ from aws-robotics/allow-config-change
  Allow specifying config_file and node_name
* Make logging stream to stdout in code instead of env var
  - This is how the ros2/demos samples ensure that logs stream to stdout
  instead of buffering. Doing it this way will be more reliable than
  setting the env var in the launch file.
* Allow specifying config_file and node_name
  - Add parameters to allow the user of this node to specify their own
  config file and node name.
* Contributors: Tim Robinson

3.0.0 (2019-09-06)
------------------
* Merge pull request `#5 <https://github.com/aws-robotics/lex-ros2/issues/5>`_ from aws-robotics/version-bump
  Bump to version 3.0.0
* Bump to version 3.0.0
* Add launch dependencies
* Allow undeclared parameters
* Disable linting as a temporary measure
* Add missing lint dependencies
* Change ament_cmake_ros to ament_cmake
* Add test depend on ament_cmake_gmock
* Merge pull request `#1 <https://github.com/aws-robotics/lex-ros2/issues/1>`_ from aws-robotics/lex-dev
  Add lex ros2 implementation
* Add gmock dependencies and reduce package version
* Remove unnecessary launch file
  Also deletes unnecessary gtest init, gmock init takes care of it.
* Add lex ros2 implementation
  **Summary**
  Creates readme for how to build lex ros2
  * ament_uncrustify reformat code
  * ament_cpplint formatting changes
  **Tests**
  Unit Tests
* Contributors: AAlon, Avishay Alon, M M, M. M, Ross Desmond

# Scenario Management

## Overview
Scenario's are used to create different config deployments for a robot.

## Usage Instructions
An exampleconfig for a scenario can be found under [Example Config](ExampleConfig/).

## How It Works
eROS manages Scenario and config via the following mechanism:
- config files are primarily stored in a special `config` directory under the catkin_ws/src/ directory.  These config files include:
  - ROS Launch Files
  - Anything else that a user would like
- These files are typically referenced as a symbolic link to a managed repo for configuration management.  For example, the `eros_dev_app` repo tracks config files for eros Development, and can be used as a guide for initial setup.

The scenario should also include the following files:
- A `CMakeLists.txt` file
- A `package.xml` file

This allows the ROS ecosystem to be able to treat this config folder as its own package.
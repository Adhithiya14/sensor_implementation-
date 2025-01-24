# Sensor Node - ROS 2 Implementation

## Overview

This package implements a ROS 2 node for communicating with a sensor over TCP. It handles:
1. Sending start and stop commands to the sensor.
2. Receiving and decoding status messages.
3. Publishing the decoded values to ROS 2 topics.

## Topics

- `supply_voltage` (int): Voltage in millivolts.
- `env_temp` (int): Temperature in deci-Celsius.
- `yaw` (int): Yaw angle in deci-degrees.
- `pitch` (int): Pitch angle in deci-degrees.
- `roll` (int): Roll angle in deci-degrees.

## Parameters

- `interval` (int): Interval for status messages in milliseconds (default: 1000).

## Installation

1. Clone the repository:
   ```bash
   git clone <repo_link>
   cd sensor_project


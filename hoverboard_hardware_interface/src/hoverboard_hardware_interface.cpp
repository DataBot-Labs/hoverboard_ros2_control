// Copyright 2023 Robert Gruberski (Viola Robotics Sp. z o.o. Poland)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hoverboard_hardware_interface/hoverboard_hardware_interface.hpp"

namespace hoverboard_hardware_interface
{
    hardware_interface::return_type HoverboardHardwareInterface::configure(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::configure(info) != hardware_interface::return_type::OK) {
            return hardware_interface::return_type::ERROR;
        }

        hardwareConfig.leftWheelJointName = info.hardware_parameters.at("left_wheel_joint_name");
        hardwareConfig.rightWheelJointName = info.hardware_parameters.at("right_wheel_joint_name");
        hardwareConfig.loopRate = std::stof(info.hardware_parameters.at("loop_rate"));
        // hardwareConfig.encoderTicksPerRevolution = std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution"));

        serialPortConfig.device = info.hardware_parameters.at("device");
        serialPortConfig.baudRate = std::stoi(info.hardware_parameters.at("baud_rate"));
        serialPortConfig.timeout = std::stoi(info.hardware_parameters.at("timeout"));

        leftWheel = MotorWheel(info.hardware_parameters.at("left_wheel_joint_name"), 
                                std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution")));
        rightWheel = MotorWheel(info.hardware_parameters.at("right_wheel_joint_name"), 
                                std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution")));

        for (const hardware_interface::ComponentInfo & joint : info.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HoverboardHardwareInterface"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());

                return hardware_interface::return_type::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HoverboardHardwareInterface"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HoverboardHardwareInterface"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(), joint.state_interfaces.size());
                
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HoverboardHardwareInterface"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HoverboardHardwareInterface"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                
                return hardware_interface::return_type::ERROR;
            }
        }

        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> HoverboardHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(leftWheel.name, hardware_interface::HW_IF_POSITION, &leftWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(leftWheel.name, hardware_interface::HW_IF_VELOCITY, &leftWheel.velocity));

        state_interfaces.emplace_back(hardware_interface::StateInterface(rightWheel.name, hardware_interface::HW_IF_POSITION, &rightWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(rightWheel.name, hardware_interface::HW_IF_VELOCITY, &rightWheel.velocity));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> HoverboardHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(leftWheel.name, hardware_interface::HW_IF_VELOCITY, &leftWheel.command));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(rightWheel.name, hardware_interface::HW_IF_VELOCITY, &rightWheel.command));

        return command_interfaces;
    }

    hardware_interface::return_type HoverboardHardwareInterface::start()
    {
        RCLCPP_INFO(rclcpp::get_logger("HoverboardHardwareInterface"), "Activating... please wait a moment...");

        RCLCPP_INFO(rclcpp::get_logger("HoverboardHardwareInterface"), "Configuring... please wait a moment...");

        if (!serialPortService.connect(serialPortConfig.device, serialPortConfig.baudRate, serialPortConfig.timeout))
        {
            return hardware_interface::return_type::ERROR;
        }

        serialPortService.BindMotorWheelFeedbackCallback(
            std::bind(&HoverboardHardwareInterface::motorWheelFeedbackCallback, this, std::placeholders::_1)
        );

        hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HoverboardHardwareInterface::stop()
    {
        // TODO: add some logic
        RCLCPP_INFO(rclcpp::get_logger("HoverboardHardwareInterface"), "Deactivating... please wait a moment...");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HoverboardHardwareInterface::read()
    {
        auto timeNew = std::chrono::system_clock::now();
        std::chrono::duration<double> timeDifference = timeNew - timePrevious;
        double periodSeconds = timeDifference.count();
        timePrevious = timeNew;

        serialPortService.read();

        double lastPosition = leftWheel.position;
        leftWheel.position = leftWheel.calculateEncoderAngle();
        leftWheel.velocity = (leftWheel.position - lastPosition) / periodSeconds;

        lastPosition = rightWheel.position;
        rightWheel.position = rightWheel.calculateEncoderAngle();
        rightWheel.velocity = (rightWheel.position - lastPosition) / periodSeconds;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HoverboardHardwareInterface::write()
    {
        MotorWheelDriveControl motorWheelDriveControl;

        const double speed = ((leftWheel.command / 0.10472) + (rightWheel.command / 0.10472)) / 2.0;
        const double steer = ((leftWheel.command / 0.10472) - speed) * 2.0;

        // TODO: radius should be read from the urdf file, check calculations
        motorWheelDriveControl.speed = (int16_t) (speed);
        motorWheelDriveControl.steer = (int16_t) (steer);
        motorWheelDriveControl.checksum = (uint16_t)(motorWheelDriveControl.head ^ motorWheelDriveControl.steer ^ motorWheelDriveControl.speed);

         RCLCPP_INFO(rclcpp::get_logger("SerialPortService"), "%i %i", motorWheelDriveControl.speed, motorWheelDriveControl.steer);

        serialPortService.write((const char*) &motorWheelDriveControl, sizeof(MotorWheelDriveControl));

        return hardware_interface::return_type::OK;
    }

    void HoverboardHardwareInterface::motorWheelFeedbackCallback(MotorWheelFeedback motorWheelFeedback) 
    {
        leftWheel.updateEncoderTicks(motorWheelFeedback.leftMotorEncoderCumulativeCount);
        rightWheel.updateEncoderTicks(motorWheelFeedback.rightMotorEncoderCumulativeCount);
    }
}

PLUGINLIB_EXPORT_CLASS(hoverboard_hardware_interface::HoverboardHardwareInterface, hardware_interface::SystemInterface)

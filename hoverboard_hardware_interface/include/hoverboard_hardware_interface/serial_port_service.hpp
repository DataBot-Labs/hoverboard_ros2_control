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

#pragma once

#include <string>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "rclcpp/rclcpp.hpp"

#include "serial_port_protocol.hpp"

#define SERIAL_PORT_READ_BUF_SIZE 256

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

namespace hoverboard_hardware_interface
{
    class SerialPortService
    {
        public:

        SerialPortService() = default;

        bool connect(const std::string &serial_device, int baud_rate, int timeout);
        bool disconnect();

        void read();
        void asyncRead();

        int write(const char *, const int &);

        void BindMotorWheelFeedbackCallback(std::function<void(MotorWheelFeedback)>);

        private:

        boost::asio::io_service io_service;
        serial_port_ptr port;
        boost::mutex mutex;

        uint16_t head_frame = 0;
        uint16_t msg_counter = 0;
        uint8_t msg_command = 0;

        char prev_byte = 0;
        char* p{};

        char read_buf_raw[SERIAL_PORT_READ_BUF_SIZE]{};

        void onReceive(const boost::system::error_code&, size_t);

        std::function<void(MotorWheelFeedback)> motorWheelFeedbackCallback;

        MotorWheelFeedback motorWheelFeedback {};
    };
}

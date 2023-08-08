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

#include "hoverboard_hardware_interface/serial_port_service.hpp"

using namespace hoverboard_hardware_interface;

bool SerialPortService::connect(const std::string &serial_device, int baud_rate, int timeout)
{
    boost::system::error_code ec;

    if (port) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"), "Port is already opened...");
        return false;
    }

    port = serial_port_ptr(new boost::asio::serial_port(io_service));
    port->open(serial_device, ec);

    if (ec) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"), "Connection to the %s failed..., error: %s",
            serial_device, ec.message().c_str());
        return false;
    }

    port->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    port->set_option(boost::asio::serial_port_base::character_size(8));
    port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    // TODO: try to run it asynchronously
    // boost::thread t([ObjectPtr = &io_service] { return ObjectPtr->run(); });
    // t.detach();

    return true;
}

bool SerialPortService::disconnect()
{
    boost::mutex::scoped_lock look(mutex);

    if (port) {
        port->cancel();
        port->close();
        port.reset();
    }

    io_service.stop();
    io_service.reset();

    return true;
}

void SerialPortService::read()
{
    boost::mutex::scoped_lock look(mutex);

    size_t bytes_transferred = port->read_some(boost::asio::buffer(read_buf_raw, SERIAL_PORT_READ_BUF_SIZE));
    
    for (unsigned int i = 0; i < bytes_transferred; ++i) {

        head_frame = ((uint8_t) prev_byte << 8) | read_buf_raw[i];

        if (head_frame == HEAD_FRAME)
        {
            usableReadBuffer.clear();
            msg_counter = 1;
        }

        if (msg_counter > 0)
        {
            usableReadBuffer.push_back((uint8_t) read_buf_raw[i]);

            msg_counter++;
        }

        if (usableReadBuffer.size() >= 3 && usableReadBuffer.size() == usableReadBuffer[3] + 5)
        {
            if ((uint8_t) usableReadBuffer[2] == HEARTBEAT_PACKAGE_CMD)
            {
                motorWheelFeedback.leftMotorEncoderCumulativeCount = (usableReadBuffer[6] << 8) | usableReadBuffer[7];
                motorWheelFeedback.rightMotorEncoderCumulativeCount = (usableReadBuffer[4] << 8) | usableReadBuffer[5];
                motorWheelFeedback.leftMotorSpeed = (usableReadBuffer[10] << 8) | usableReadBuffer[11];
                motorWheelFeedback.rightMotorSpeed = (usableReadBuffer[8] << 8) | usableReadBuffer[9];
                motorWheelFeedback.leftMotorCurrent = (usableReadBuffer[14] << 8) | usableReadBuffer[15];
                motorWheelFeedback.rightMotorCurrent = (usableReadBuffer[12] << 8) | usableReadBuffer[13];
                motorWheelFeedback.leftMotorTemperature = (usableReadBuffer[18] << 8) | usableReadBuffer[19];
                motorWheelFeedback.rightMotorTemperature = (usableReadBuffer[16] << 8) | usableReadBuffer[17];

                // TODO: add the rest of parameters

                motorWheelFeedbackCallback(motorWheelFeedback);
            }

            msg_counter = 0;
        }

        prev_byte = read_buf_raw[i];
    }
}

void SerialPortService::asyncRead()
{
    if (port.get() == nullptr || !port->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"), "Port is already closed...");
        return;
    }

    port->async_read_some(
            boost::asio::buffer(read_buf_raw, SERIAL_PORT_READ_BUF_SIZE),
            boost::bind(&SerialPortService::onReceive,
                    this, boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
}

void SerialPortService::onReceive(const boost::system::error_code& ec, size_t bytes_transferred)
{
    RCLCPP_INFO(rclcpp::get_logger("SerialPortService"), "onReceive async event...");

    // boost::mutex::scoped_lock look(mutex);

    // if (port.get() == nullptr || !port->is_open()) return;

    // if (ec) {
    //     asyncRead();
    //     return;
    // }
}

int SerialPortService::write(std::vector<uint8_t> & message)
{
    boost::system::error_code ec;

    if (port.get() == nullptr || !port->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"), "Port is already closed...");
        return 0;
    }

    if (message.size() == 0) {
        return 0;
    }

    return port->write_some(boost::asio::buffer(message, message.size()), ec);
}

uint8_t SerialPortService::calculateChecksum(std::vector<uint8_t> & message)
{
    uint8_t checksum = 0;

     for(int i = 0; i < message.size(); i++)
     {
        checksum += message[i];
     }

     return checksum;
}

void SerialPortService::BindMotorWheelFeedbackCallback(std::function<void(MotorWheelFeedback)> fn) {
    motorWheelFeedbackCallback = fn;
}
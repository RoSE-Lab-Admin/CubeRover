#ifndef ROS_ARDUINO
#define ROS_ARDUINO

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}


class ArduinoComms
{

public:
  ArduinoComms(const rclcpp::Logger& logger) : logger_(logger) {}

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    for (size_t i = 0; i < 5; i++){
      try {
        serial_conn_.Open(serial_device);
        serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
        serial_conn_.FlushIOBuffers();
        return;
      } catch (const LibSerial::OpenFailed& e) {
        RCLCPP_ERROR(logger_, "error opening serial port: %s - %s", serial_device.c_str(), e.what());
        // std::cerr << "error opening serial port " << serial_device << ": " << e.what() <<std::endl;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "error opening serial port: %s - %s", serial_device.c_str(), e.what());
        std::cerr << "exception thrown! " << e.what();
      }
      rclcpp::sleep_for(std::chrono::seconds(5));
    }
    RCLCPP_FATAL(logger_, "cannot open serial port: %s", serial_device.c_str());
    throw std::runtime_error("no device found");
  }


  void disconnect()
  {
    serial_conn_.Close();
  }


  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);



    std::string response = "";
    try {
      auto start = std::chrono::high_resolution_clock::now();
      char c = '\0';
      while (true) {
        if (serial_conn_.IsDataAvailable()) {
          serial_conn_.ReadByte(c);
          response += c;
          if (c== '\n') break;
        }
        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() >= timeout_ms_) {
          std::cout << response << ": timed out" << std::endl;
          throw LibSerial::ReadTimeout("read timeout");
        }
      }

      // Responses end with \r\n so we will read up to (and including) the \n.
      // response = serial_conn_.ReadLine(timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout& e)
    {
        std::cerr << "Read timeout: " << e.what() << std::endl;
        response = "dnf";
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }


  void read_telem_values(std::vector<int> &telem)
  {
    std::stringstream ss(send_msg("t\r"));
    std::string token;
    //std::cerr << ss.str() << std::endl;
    ss >> token;
    if (token != "e") {
        std::cerr << "Expected 'e' token but got: '" << token << "'" << std::endl;
        return;
    }

    for (size_t i = 0; i < 12 && ss >> token; ++i) {
        try {
            telem[i] = std::stoi(token);
        } catch (const std::invalid_argument &e) {
            std::cerr << "Invalid token: " << token << std::endl;
            break;
        }
    }
  }


  void set_motor_values(int left_motors, int right_motors)
  {
    std::stringstream ss;
    ss << "m " << left_motors << " " << right_motors << "\r";
    serial_conn_.Write(ss.str());
  }


private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
    rclcpp::Logger logger_;
};



#endif
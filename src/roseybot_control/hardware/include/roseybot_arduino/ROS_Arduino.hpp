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
#include "commands.h"
#include "messages.h"


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


  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms, 
               int32_t noise_floor_pct, int32_t opp_dir_ms, int32_t max_vel_pct)
  {  
    timeout_ms_ = timeout_ms;
    for (size_t i = 0; i < 5; i++){
      try {
        // --- SERIAL CONNECTION CODE ---
        serial_conn_.Open(serial_device);
        serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));

        // --- ERROR CANCELLATION CODE ---
        write_msg(std::string{CLEAR_ERROR});
        // Give the Arduino 50ms to read the clear error character, exit the while(true) loop, 
        // and flush its own receive buffer.
        rclcpp::sleep_for(std::chrono::milliseconds(50));
        
        // --- SEND SAFETY PARAMS CODE ---
        std::stringstream ss;
        ss << SET_SAFETY_PARAMS << " " << noise_floor_pct << " " << opp_dir_ms << " " << max_vel_pct;
        send_msg(ss.str()); // Use send_msg so synchronously wait for the Arduino's acknowledgement

        return;
      } catch (const LibSerial::OpenFailed& e) {
        RCLCPP_ERROR(logger_, "error opening serial port: %s - %s", serial_device.c_str(), e.what());
        // std::cerr << "error opening serial port " << serial_device << ": " << e.what() <<std::endl;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "error opening serial port: %s - %s", serial_device.c_str(), e.what());
        std::cerr << "exception thrown! " << e.what();

        // Close the port so that the next loop iteration can reopen it
        if (serial_conn_.IsOpen()) {
            serial_conn_.Close();
        }
      }

      // Wait 5 seconds before attempting to open the port again
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


  /**
  * @brief Sends a pure ASCII message to the Arduino
  * @param msg_to_send The string payload to transmit.
  * @param print_output If true, echoes the transaction to standard out.
  */
  void write_msg(const std::string &msg_to_send)
  {
    serial_conn_.FlushIOBuffers(); 
    serial_conn_.Write(msg_to_send + "\r"); // Automatically append the carriage return
  }


  /**
  * @brief Handles reading one line of data
  * @return std::string The response string received from the Arduino.
  * @throws LibSerial::ReadTimeout If the Arduino fails to respond within timeout_ms_.
  */
  std::string read_line()
  {
    std::string response = "";
    auto start = std::chrono::high_resolution_clock::now();
    char c = '\0';

    while (true) {
      if (serial_conn_.IsDataAvailable()) {
        serial_conn_.ReadByte(c);
        response += c;
        if (c== '\n') break;
      }

      // Check for timeout
      if (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start).count() >= timeout_ms_) {
        throw LibSerial::ReadTimeout("read_line timeout");
      }
    }

    return response;
  }


  /**
  * @brief Sends a pure ASCII message to the Arduino and waits for a newline.
  * @param msg_to_send The string payload to transmit.
  * @param print_output If true, echoes the transaction to standard out.
  * @return std::string The response string received from the Arduino.
  * @throws LibSerial::ReadTimeout If the Arduino fails to respond within timeout_ms_.
  */
  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    write_msg(msg_to_send);
    std::string response = read_line();

    // Responses end with \r\n so we will read up to (and including) the \n.
    // response = serial_conn_.ReadLine(timeout_ms_);

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }
    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("");
  }


  void read_telem_values(std::vector<int> &telem)
  {
    // Use the Write primitive
    write_msg(std::string{GET_TELEM});

    // Keep reading lines while they exist
    while (true) {
      std::string line = read_line(); 
      std::stringstream ss(line);

      // Get the message type received
      char message_type;
      ss >> message_type;

      // Handle the different message types
      switch (message_type)
      {
        case TELEMETRY_MESSAGE: 
          on_telem_received(ss, telem);
          return;
              
        case GENERAL_MESSAGE: 
          on_message_received(ss);
          // Loop should also pull the next telemetry line
          break;
        
        default:
          std::cerr << "Invalid message type: '" << message_type << "'" << std::endl;
          return;
      }
    }
  }


  void set_motor_values(int left_motors, int right_motors)
  {
    std::stringstream ss;
    ss << SET_MOTOR_SPEEDS << " " << left_motors << " " << right_motors;
    write_msg(ss.str());
  }


private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
  rclcpp::Logger logger_;


  void on_telem_received(std::stringstream &ss, std::vector<int> &telem)
  {
    // RH: Changing to 14 elements to account for voltages
    // CG: Changing to 18 elements to accout for PWM data
    const uint16_t TELEMETRY_DATA_SIZE = 18;
    std::string token;
    for (size_t i = 0; i < TELEMETRY_DATA_SIZE && ss >> token; ++i) {
        try {
            telem[i] = std::stoi(token);
        } catch (const std::invalid_argument &e) {
            std::cerr << "Invalid token: " << token << std::endl;
            break;
        }
    }
  }


  void on_message_received(std::stringstream &ss)
  {
    int message_code;
    ss >> message_code; // Safely reads "1", "9", or "15" into an integer!

    // Grabs the rest of the string payload automatically
    // std::ws consumes the space between the error code and the message
    std::string message;
    std::getline(ss >> std::ws, message);

    // Print message
    std::cerr << "[Message " << message_code << " - " 
              << messageCodeToString(message_code) << "]: " 
              << message << std::endl;
  }
};



#endif

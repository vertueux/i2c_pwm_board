//
// Created by ros on 2/3/24.
//

#ifndef BOARD_HANDLER_H_
#define BOARD_HANDLER_H_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "i2c_pwm_board_msgs/msg/servo_array.hpp"
#include "i2c_pwm_board_msgs/srv/servos_config.hpp"
#include "i2c_pwm_board_msgs/srv/drive_mode.hpp"
#include "i2c_pwm_board_msgs/srv/int_value.hpp"

#include "board_controller.h"

// Public node to use in static functions.
extern std::shared_ptr<BoardNode> board_node;

class BoardHandler {

 public:
  explicit BoardHandler(const std::string &node_name);
  void init(int io_device, int frequency);
  void set_handlers(int board_number);
  static void sigint_handler(int signum);

  void servos_absolute_handler(std::shared_ptr<i2c_pwm_board_msgs::msg::ServoArray> msg);
  void servos_proportional_handler(std::shared_ptr<i2c_pwm_board_msgs::msg::ServoArray> msg);
  void servos_drive_handler(std::shared_ptr<geometry_msgs::msg::Twist> msg);
  bool set_pwm_frequency_handler(std::shared_ptr<i2c_pwm_board_msgs::srv::IntValue::Request> req,
                                 std::shared_ptr<i2c_pwm_board_msgs::srv::IntValue::Response> res);
  bool config_servos_handler(std::shared_ptr<i2c_pwm_board_msgs::srv::ServosConfig::Request> req,
                             std::shared_ptr<i2c_pwm_board_msgs::srv::ServosConfig::Response> res);
  bool config_drive_mode_handler(std::shared_ptr<i2c_pwm_board_msgs::srv::DriveMode::Request> req,
                                 std::shared_ptr<i2c_pwm_board_msgs::srv::DriveMode::Response> res);
  bool stop_servos_handler(std::shared_ptr<std_srvs::srv::Empty::Request> req,
                           std::shared_ptr<std_srvs::srv::Empty::Response> res);
 private:

  rclcpp::Service<i2c_pwm_board_msgs::srv::ServosConfig>::SharedPtr config_srv;
  rclcpp::Subscription<i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr abs_sub;
  rclcpp::Subscription<i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr rel_sub;
  rclcpp::Service<i2c_pwm_board_msgs::srv::IntValue>::SharedPtr freq_srv;
  rclcpp::Service<i2c_pwm_board_msgs::srv::DriveMode>::SharedPtr mode_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drive_sub;

};

#endif // BOARD_HANDLER_H_

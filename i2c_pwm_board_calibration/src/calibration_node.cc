#include <ctime>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/empty.hpp>

#include "i2c_pwm_board_msgs/msg/servo_array.hpp"
#include "i2c_pwm_board_msgs/msg/servo.hpp"

#define UNUSED(expr) do { (void)(expr); } while (0)

// Used for reading terminal values.
struct termios old_chars, new_chars;

// Static values.
rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_servos_client;
int board = 1;

namespace smov {

class CalibrationNode : public rclcpp::Node {
 public:
  CalibrationNode() : Node("i2c_pwm_board_calibration") {
    // Getting the default config.
    tcgetattr(0, &old_chars);
    signal(SIGINT, sigint_handler);
  
    // Initialize publishers & get user input.
    init();

    // Initializing the reader.
    fcntl(0, F_SETFL, O_NONBLOCK);
    new_chars = old_chars;
    new_chars.c_lflag &= ~ICANON;
    new_chars.c_lflag &= 0 ? ECHO : ~ECHO; // echo = 0.
    tcsetattr(0, TCSANOW, &new_chars);

    // Clear the terminal.
    RCLCPP_INFO(this->get_logger(), "\033[2J\033[;H");
    RCLCPP_INFO(this->get_logger(), "Use arrow keys (up/down) to modify that servo's value");

    timer = this->create_wall_timer(std::chrono::microseconds(100), std::bind(&CalibrationNode::timer_callback, this));
  }

  void timer_callback() {
    int c = getchar();
    switch (c) {
      case 65: // 65: Key up.
        RCLCPP_INFO(this->get_logger(), "\033[2J\033[;H");
        servo_value += sensibility;
        servo_array.servos[0].value = servo_value;
        RCLCPP_INFO(this->get_logger(), "Servo value: %f", servo_array.servos[0].value);
        abs_pub->publish(servo_array);
        break;
      case 66: // 66: Key down.
        RCLCPP_INFO(this->get_logger(), "\033[2J\033[;H");
        servo_value -= sensibility;
        if (servo_value > 0) {
          servo_array.servos[0].value = servo_value;
          RCLCPP_INFO(this->get_logger(), "Servo value: %f", servo_array.servos[0].value);
          abs_pub->publish(servo_array);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Servo value can't be negative. Defaulting to 50.");
          servo_value = 50;
        }
        break;
    }
  }

  void init() {
    RCLCPP_INFO(this->get_logger(), "Board (1 or etc..):");
    std::cin >> board;

    abs_pub = this->create_publisher<i2c_pwm_board_msgs::msg::ServoArray>("servos_absolute_" + std::to_string(board), 1);
    stop_servos_client = this->create_client<std_srvs::srv::Empty>("stop_servos_" + std::to_string(board));

    RCLCPP_INFO(this->get_logger(), "Servo number:");
    std::cin >> servo_number;

    RCLCPP_INFO(this->get_logger(), "Default (center) value:");
    std::cin >> servo_value;

    RCLCPP_INFO(this->get_logger(), "Sensibility (new value = default value +/- sensibility):");
    std::cin >> sensibility;

    set_up_abs_servo(servo_number, servo_value);
  }

  void set_up_abs_servo(int servo_number, int default_value) {
    i2c_pwm_board_msgs::msg::Servo temp_servos;

    temp_servos.servo = static_cast<int16_t>(servo_number);
    temp_servos.value = default_value;
    servo_array.servos.push_back(temp_servos);
    abs_pub->publish(servo_array);
  }

  static void sigint_handler(int signum) {
    UNUSED(signum);

    // Changing to default config.
    tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);

    // Stopping the servos.
    stop_servos();

    // Shutting down rclcpp.
    rclcpp::shutdown();

    // Abort the program to prevent showing rclcpp exceptions.
    abort();
  }

  static void stop_servos() {
    auto req = std::make_shared<std_srvs::srv::Empty::Request>();
    while (!stop_servos_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("i2c_pwm_board_calibration"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("i2c_pwm_board_calibration"), "Stop Servos service not available, waiting again...");
    }
    auto f_result = stop_servos_client->async_send_request(req);
  } 

 private:
  int servo_number = 0, servo_value = 400, sensibility = 2;

  rclcpp::TimerBase::SharedPtr timer;

  i2c_pwm_board_msgs::msg::ServoArray servo_array;

  rclcpp::Publisher<i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr abs_pub;
};

} // namespace smov

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::CalibrationNode>());
}

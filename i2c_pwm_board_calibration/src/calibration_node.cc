#include <ctime>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/empty.hpp>

#include "i2c_pwm_board_msgs/msg/servo_array.hpp"
#include "i2c_pwm_board_msgs/msg/servo.hpp"

#define KEY_UP 65
#define KEY_DOWN 66

// Used for reading terminal values.
struct termios old_chars, new_chars;

namespace smov {

class CalibrationNode : public rclcpp::Node {
 public:
  CalibrationNode() : Node("i2c_pwm_board_calibration") {
    front_abs_pub = this->create_publisher<i2c_pwm_board_msgs::msg::ServoArray>("front_servos_absolute", 1);
    back_abs_pub = this->create_publisher<i2c_pwm_board_msgs::msg::ServoArray>("back_servos_absolute", 1);

    get_base_input();
    init_reader(0);

    // Clear the terminal.
    RCLCPP_INFO(this->get_logger(), "\033[2J\033[;H");
    RCLCPP_INFO(this->get_logger(), "Use arrow keys (up/down) to modify that servo's value");

    timer = this->create_wall_timer(std::chrono::microseconds(100), std::bind(&CalibrationNode::timer_callback, this));
  }

  void timer_callback() {
    int c = getchar();
    int d = getchar();
    switch (c) {
      case KEY_UP:
        RCLCPP_INFO(this->get_logger(), "\033[2J\033[;H");
        servo_value += sensibility;
        if (board == 1) {
          front_servo_array.servos[0].value = servo_value;
          RCLCPP_INFO(this->get_logger(), "Press 'escape' to successfully exit the program.");
          RCLCPP_INFO(this->get_logger(), "Servo value: %f", front_servo_array.servos[0].value);
          front_abs_pub->publish(front_servo_array);
        } else {
          back_servo_array.servos[0].value = servo_value;
          RCLCPP_INFO(this->get_logger(), "Press 'escape' to successfully exit the program.");
          RCLCPP_INFO(this->get_logger(), "Servo value: %f", back_servo_array.servos[0].value);
          back_abs_pub->publish(back_servo_array);
        }
        break;
      case KEY_DOWN:
        RCLCPP_INFO(this->get_logger(), "\033[2J\033[;H");
        servo_value -= sensibility;
        if (servo_value > 0) {
          if (board == 1) {
            front_servo_array.servos[0].value = servo_value;
            RCLCPP_INFO(this->get_logger(), "Press 'escape' to successfully exit the program.");
            RCLCPP_INFO(this->get_logger(), "Servo value: %f", front_servo_array.servos[0].value);
            front_abs_pub->publish(front_servo_array);
          } else {
            back_servo_array.servos[0].value = servo_value;
            RCLCPP_INFO(this->get_logger(), "Press 'escape' to successfully exit the program.");
            RCLCPP_INFO(this->get_logger(), "Servo value: %f", back_servo_array.servos[0].value);
            back_abs_pub->publish(back_servo_array);
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Servo value can't be negative. Defaulting to 50.");
          servo_value = 50;
        }
        break;
    }
    if (c == 27 && d != 91) // 27: Escape key.
      stop_servos();
  }

  void set_up_abs_servo(int board, int servo_number, int default_value) {
    i2c_pwm_board_msgs::msg::Servo front_temp_servo;
    i2c_pwm_board_msgs::msg::Servo back_temp_servo;

    if (board == 1) {
      front_temp_servo.servo = static_cast<int16_t>(servo_number);
      front_temp_servo.value = default_value;
      front_servo_array.servos.push_back(front_temp_servo);
      front_abs_pub->publish(front_servo_array);
    } else {
      back_temp_servo.servo = static_cast<int16_t>(servo_number);
      back_temp_servo.value = default_value;
      back_servo_array.servos.push_back(back_temp_servo);
      back_abs_pub->publish(back_servo_array);
    }
  }

  void get_base_input() {
    RCLCPP_INFO(this->get_logger(), "Board (1 or etc..):");
    std::cin >> board;

    RCLCPP_INFO(this->get_logger(), "Servo number:");
    std::cin >> servo_number;

    RCLCPP_INFO(this->get_logger(), "Default (center) value:");
    std::cin >> servo_value;

    RCLCPP_INFO(this->get_logger(), "Sensibility (new value = default value +/- sensibility):");
    std::cin >> sensibility;

    set_up_abs_servo(board, servo_number, servo_value);
  }

 private:
  int servo_number = 0, board = 1, servo_value = 400, sensibility = 2;

  rclcpp::TimerBase::SharedPtr timer;

  i2c_pwm_board_msgs::msg::ServoArray front_servo_array;
  i2c_pwm_board_msgs::msg::ServoArray back_servo_array;

  rclcpp::Publisher<i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr front_abs_pub;
  rclcpp::Publisher<i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr back_abs_pub;

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr front_stop_servos_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr back_stop_servos_client;

  void init_reader(int echo) {
    fcntl(0, F_SETFL, O_NONBLOCK);
    tcgetattr(0, &old_chars);
    new_chars = old_chars;
    new_chars.c_lflag &= ~ICANON;
    new_chars.c_lflag &= echo ? ECHO : ~ECHO;
    tcsetattr(0, TCSANOW, &new_chars);
  }

  void stop_servos() {
    front_stop_servos_client = this->create_client<std_srvs::srv::Empty>("front_stop_servos");
    back_stop_servos_client = this->create_client<std_srvs::srv::Empty>("back_stop_servos");

    auto req = std::make_shared<std_srvs::srv::Empty::Request>();

    while (!front_stop_servos_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Front Stop Servos service not available, waiting again...");
    }
    while (!back_stop_servos_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Back Stop Servos service not available, waiting again...");
    }

    auto f_result = front_stop_servos_client->async_send_request(req);
    auto b_result = back_stop_servos_client->async_send_request(req);
  
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sent a request to stop all servos.");
  }
};

} // namespace smov

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::CalibrationNode>());
  tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);
  rclcpp::shutdown();
}

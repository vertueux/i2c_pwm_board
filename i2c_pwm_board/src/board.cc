//
// Created by ros on 2/3/24.
//

#include <cerrno>
#include <unistd.h>
#include <climits>  // for INT_MAX, INT_MIN.
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/empty.hpp>

#include "board_handler.h"

int main(int argc, char **argv) {

  int io_device = 0;
  rclcpp::init(argc, argv);

  auto board_handler = std::make_shared<BoardHandler>("i2c_pwm_board");
  char *p;
  errno = 0;
  if (argv[1] != nullptr) {
    long conv = strtol(argv[1], &p, 10);
    if (errno != 0 || *p != '\0' || conv > INT_MAX || conv < INT_MIN)
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Expecting an integer to open the i2c-n/ address.");
    else
      io_device = static_cast<int>(conv);
  } else
    io_device = 1;

  board_handler->set_handlers(io_device);

  board_handler->init(io_device, 50);    // Loads parameters and performs initialization.

  rclcpp::spin(board_node);

  close(io_device);
  rclcpp::shutdown();
}

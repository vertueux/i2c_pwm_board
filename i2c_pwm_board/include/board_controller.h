//
// Created by ros on 2/3/24.
//

#ifndef BOARD_CONTROLLER_H_
#define BOARD_CONTROLLER_H_

#include <rclcpp/rclcpp.hpp>

#define BASE_ADDR   0x40
#define CONST(s) ((char*)(s))
#define MAX_BOARDS 62
#define MAX_SERVOS (16*MAX_BOARDS)
#define UNUSED(expr) do { (void)(expr); } while (0)

typedef struct _servo_config {
  int center;
  int range;
  int direction;
  int mode_pos;
} ServoConfig;

typedef struct _drive_mode {
  int mode;
  float rpm;
  float radius;
  float track;
  float scale;
} DriveMode;

enum drive_modes {
  MODE_UNDEFINED = 0,
  MODE_ACKERMAN = 1,
  MODE_DIFFERENTIAL = 2,
  MODE_MECANUM = 3,
  MODE_INVALID = 4
};

enum drive_mode_positions {
  POSITION_UNDEFINED = 0,
  POSITION_LEFTFRONT = 1,
  POSITION_RIGHTFRONT = 2,
  POSITION_LEFTREAR = 3,
  POSITION_RIGHTREAR = 4,
  POSITION_INVALID = 5
};

enum pwm_regs {
  // Registers / etc.
  MODE1 = 0x00,
  MODE2 = 0x01,
  SUBADR1 = 0x02,              // Enable sub address 1 support.
  SUBADR2 = 0x03,              // Enable sub address 2 support.
  SUBADR3 = 0x04,              // Enable sub address 2 support.
  PRESCALE = 0xFE,
  CHANNEL_ON_L = 0x06,
  CHANNEL_ON_H = 0x07,
  CHANNEL_OFF_L = 0x08,
  CHANNEL_OFF_H = 0x09,
  ALL_CHANNELS_ON_L = 0xFA,
  ALL_CHANNELS_ON_H = 0xFB,
  ALL_CHANNELS_OFF_L = 0xFC,
  ALL_CHANNELS_OFF_H = 0xFD,
  RESTART = 0x80,
  SLEEP = 0x10,              // Enable low power mode.
  ALLCALL = 0x01,
  INVRT = 0x10,              // Invert the output control logic.
  OUTDRV = 0x04
};

// The class that handles messages between the controller and ROS2.
class BoardNode : public rclcpp::Node {
 public:
  explicit BoardNode(const std::string &node_name = "i2c_pwm_board", const std::string &node_namespace = "/");
  float convert_mps_to_proportional(float speed);
  void set_pwm_frequency(int freq);
  void set_pwm_interval_all(int start, int end);
  void set_active_internal_board(int board);
  void set_pwm_interval(int servo, int start, int end);
  void set_pwm_interval_proportional(int servo, float value);
  void config_servo(int servo, int center, int range, int direction);
  int config_servo_position(int servo, int position);
  int config_drive_mode(const std::string &mode, float rpm, float radius, float track, float scale);
  int init(int io_device, int frequency);

  int get_active_board() const;
  int get_last_servo() const;
  DriveMode get_active_drive() const;

  // We can support up to 62 boards (1..62), each with 16 PWM devices (1..16)
  // Made public to keep array usage and not convert to std for now
  ServoConfig servo_configs[MAX_SERVOS]{};
  DriveMode active_drive{};
  int pwm_boards[MAX_BOARDS] = {};

 private:
  void setup(const char *filename);

  int active_board; // Default is 0
  int last_servo; // defaults to -1
  int pwm_frequency;                    // Default is 50Hz.
  int controller_io_handle; // Defaults to 0
  int controller_io_device; // Defaults to 0

};

#endif //BOARD_CONTROLLER_H_

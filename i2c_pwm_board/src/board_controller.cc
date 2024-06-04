/*
 * Copyright (c) 2022-2023 Virtuous, Bradan Lane Studio.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <fcntl.h>
#include <sys/ioctl.h>

extern "C" {
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include <rclcpp/logging.hpp>

#include "i2c_pwm_board_msgs/srv/int_value.hpp"

#include <xmlrp/xmlrp.h>
#include "board_controller.h"

BoardNode::BoardNode(const std::string &node_name, const std::string &node_namespace) : rclcpp::Node(node_name,
                                                                                                     node_namespace) {
  this->last_servo = -1;
  this->active_board = 0;
  this->pwm_frequency = 50;
  this->controller_io_handle = 0;
  this->controller_io_device = 0;
}

/**
   \private Method to convert meters per second to a proportional value in the range of ±1.0.

   @param speed Float requested speed in meters per second.
   @returns Float value (±1.0) for servo speed.
 */
float BoardNode::convert_mps_to_proportional(float speed) {
  float initial, max_rate;
  initial = speed;

  if (this->active_drive.rpm <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid active drive mode RPM %6.4f :: RPM must be greater than 0",
                 this->active_drive.rpm);
    return 0.0;
  }
  if (this->active_drive.radius <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid active drive mode radius %6.4f :: wheel radius must be greater than 0",
                 this->active_drive.radius);
    return 0.0;
  }

  max_rate = static_cast<float>((this->active_drive.radius * M_PI * 2) * (this->active_drive.rpm / 60.0));

  speed = speed / max_rate;

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
               "%6.4f = convert_mps_to_proportional ( speed(%6.4f) / ((radus(%6.4f) * pi(%6.4f) * 2) * (rpm(%6.4f) / 60.0)) )",
               speed,
               initial,
               this->active_drive.radius,
               M_PI,
               this->active_drive.rpm);
  return speed;
}

/**
 * \private Method to set a pulse frequency.
 *
 * The pulse defined by start/stop will be active on all channels until any subsequent call changes it.
 * @param frequency an int value (1..15000) indicating the pulse frequency where 50 is typical for RC servos
 * Example set_frequency (68)  // set the pulse frequency to 68Hz
 */
void BoardNode::set_pwm_frequency(int freq) {
  int prescale;
  char old_mode, new_mode;

  this->pwm_frequency = freq;   // Save to global.

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "_set_pwm_frequency prescale");

  float prescale_val = 25000000.0; // 25MHz.
  prescale_val /= 4096.0;
  prescale_val /= (float) freq;
  prescale_val -= 1.0;
  prescale = floor(prescale_val + 0.5);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting PWM frequency to %d Hz", freq);

  const struct timespec timespec1 = {1, 000000L};
  nanosleep(&timespec1, nullptr);

  old_mode = i2c_smbus_read_byte_data(this->controller_io_handle, MODE1);
  new_mode = (old_mode & 0x7F) | 0x10; // Sleep.

  if (0 > i2c_smbus_write_byte_data(this->controller_io_handle, MODE1, new_mode))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to set PWM controller to sleep mode");

  if (0 > i2c_smbus_write_byte_data(this->controller_io_handle, PRESCALE, (prescale)))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to set PWM controller prescale");

  if (0 > i2c_smbus_write_byte_data(this->controller_io_handle, MODE1, old_mode))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to set PWM controller to active mode");

  const struct timespec timespec2 = {0, 5000000L};
  nanosleep(&timespec2, nullptr);   // Sleep 5 microseconds.

  if (0 > i2c_smbus_write_byte_data(this->controller_io_handle, MODE1, old_mode | 0x80))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to restore PWM controller to active mode");
}

/**
 * \private Method to set a common value for all PWM channels on the active board.
 *
 * The pulse defined by start/stop will be active on all channels until any subsequent call changes it.
 * @param start an int value (0..4096) indicating when the pulse will go high sending power to each channel.
 * @param end an int value (0..4096) indicating when the pulse will go low stoping power to each channel.
 * Example set_pwm_interval_all (0, 108).   // set all servos with a pulse width of 105
 */
void BoardNode::set_pwm_interval_all(int start, int end) {
  // The public API is ONE based and hardware is ZERO based.
  if ((this->active_board < 1) || (this->active_board > 62)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Internal error - invalid active board number %d :: PWM board numbers must be between 1 and 62",
                 this->active_board);
    return;
  }

  if (0 > i2c_smbus_write_byte_data(this->controller_io_handle, ALL_CHANNELS_ON_L, start & 0xFF))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Error setting PWM start low byte for all servos on internal board %d (bus %d)",
                 this->active_board,
                 this->controller_io_device);
  if (0 > i2c_smbus_write_byte_data(this->controller_io_handle, ALL_CHANNELS_ON_H, start >> 8))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Error setting PWM start high byte for all servos on internal board %d (bus %d)",
                 this->active_board,
                 this->controller_io_device);
  if (0 > i2c_smbus_write_byte_data(this->controller_io_handle, ALL_CHANNELS_OFF_L, end & 0xFF))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Error setting PWM end low byte for all servos on internal board %d (bus %d)",
                 this->active_board,
                 this->controller_io_device);
  if (0 > i2c_smbus_write_byte_data(this->controller_io_handle, ALL_CHANNELS_OFF_H, end >> 8))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Error setting PWM end high byte for all servos on internal board %d (bus %d)",
                 this->active_board,
                 this->controller_io_device);
}

/**
 * \private Method to set the active board.
 *
 * @param board An int value (1..62) indicating which board to activate for subsequent service and topic subscription activity where 1 coresponds to the default board address of 0x40 and value increment up.
 * Example set_active_internal_board (68)   // set the pulse frequency to 68Hz.
 */
void BoardNode::set_active_internal_board(int board) {
  char mode1res;

  if ((board < 1) || (board > 62)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Internal error :: invalid internal board number %d :: board numbers must be between 1 and 62",
                 board);
    return;
  }

  if (this->active_board != board) {
    this->active_board = board;   // Save to global.

    // the public API is ONE based and hardware is ZERO based.
    board--;

    if (0 > ioctl(this->controller_io_handle, I2C_SLAVE, (BASE_ADDR + (board)))) {
      RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),
                   "Failed to acquire bus access and/or talk to I2C slave at address 0x%02X",
                   (BASE_ADDR + board));
      return;
    }

    if (pwm_boards[board] < 0) {
      pwm_boards[board] = 1;
      if (0 > i2c_smbus_write_byte_data(this->controller_io_handle, MODE2, OUTDRV))
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to enable PWM outputs for totem-pole structure");

      if (0 > i2c_smbus_write_byte_data(this->controller_io_handle, MODE1, ALLCALL))
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to enable ALLCALL for PWM channels");

      const struct timespec timespec3 = {0, 5000000L};
      nanosleep(&timespec3, nullptr);   // Sleep 5 microseconds, wait for osci.

      mode1res = i2c_smbus_read_byte_data(this->controller_io_handle, MODE1);
      mode1res = mode1res & ~SLEEP;

      if (0 > i2c_smbus_write_byte_data(this->controller_io_handle, MODE1, mode1res))
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to recover from low power mode");

      nanosleep(&timespec3, nullptr);   // Sleep 5 microseconds, wait for osci/

      // The first time we activate a board, we mark it and set all of its servo channels to 0.
      this->set_pwm_interval_all(0, 0);
    }
  }
}

/**
 * \private method to set a value for a PWM channel on the active board
 *
 *The pulse defined by start/stop will be active on the specified servo channel until any subsequent call changes it.
 *@param servo an int value (1..16) indicating which channel to change power
 *@param start an int value (0..4096) indicating when the pulse will go high sending power to each channel.
 *@param end an int value (0..4096) indicating when the pulse will go low stoping power to each channel.
 *Example set_pwm_interval (3, 0, 350)    // set servo #3 (fourth position on the hardware board) with a pulse of 350
 */
void BoardNode::set_pwm_interval(int servo, int start, int end) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "_set_pwm_interval enter");

  if ((servo < 1) || (servo > (MAX_SERVOS))) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid servo number %d :: servo numbers must be between 1 and %d",
                 servo,
                 MAX_BOARDS);
    return;
  }

  int board = ((int) ((servo - 1) / 16)) + 1; // servo 1..16 is board #1, servo 17..32 is board #2, etc.
  this->set_active_internal_board(board);

  servo = ((servo - 1) % 16) + 1;             // servo numbers are 1..16.

  // the public API is ONE based and hardware is ZERO based.
  board = this->active_board - 1;             // The hardware enumerates boards as 0..61.
  int channel = servo - 1;                    // The hardware enumerates servos as 0..15.
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "_set_pwm_interval internal board=%d servo=%d", board, servo);

  if (0 > i2c_smbus_write_byte_data(this->controller_io_handle, CHANNEL_ON_L + 4 * channel, start & 0xFF))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Error setting PWM start low byte on servo %d on internal board %d (bus %d)",
                 servo,
                 this->active_board,
                 this->controller_io_device);
  if (0 > i2c_smbus_write_byte_data(this->controller_io_handle, CHANNEL_ON_H + 4 * channel, start >> 8))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Error setting PWM start high byte on servo %d on internal board %d (bus %d)",
                 servo,
                 this->active_board,
                 this->controller_io_device);
  if (0 > i2c_smbus_write_byte_data(this->controller_io_handle, CHANNEL_OFF_L + 4 * channel, end & 0xFF))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Error setting PWM end low byte on servo %d on internal board %d (bus %d)",
                 servo,
                 this->active_board,
                 this->controller_io_device);
  if (0 > i2c_smbus_write_byte_data(this->controller_io_handle, CHANNEL_OFF_H + 4 * channel, end >> 8))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Error setting PWM end high byte on servo %d on internal board %d (bus %d)",
                 servo,
                 this->active_board,
                 this->controller_io_device);
}

/**
 * \private Method to set a value for a PWM channel, based on a range of ±1.0, on the active board.
 *
 * The pulse defined by start/stop will be active on the specified servo channel until any subsequent call changes it.
 * @param servo an int value (1..16) indicating which channel to change power.
 * @param value an int value (±1.0) indicating when the size of the pulse for the channel.
 * Example _set_pwm_interval (3, 0, 350)    // set servo #3 (fourth position on the hardware board) with a pulse of 350.
 */
void BoardNode::set_pwm_interval_proportional(int servo, float value) {
  if ((value < -1.0001) || (value > 1.0001)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid proportion value %f :: proportion values must be between -1.0 and 1.0",
                 value);
    return;
  }

  ServoConfig *configp = &(this->servo_configs[servo - 1]);

  if ((configp->center < 0) || (configp->range < 0)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Missing servo configuration for servo[%d]", servo);
    return;
  }

  int pos = static_cast<int>((configp->direction * (((float) (configp->range) / 2) * value)) + configp->center);

  if ((pos < 0) || (pos > 4096)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid computed position servo[%d] = (direction(%d) * ((range(%d) / 2) * value(%6.4f))) + %d = %d",
                 servo,
                 configp->direction,
                 configp->range,
                 value,
                 configp->center,
                 pos);
    return;
  }
  set_pwm_interval(servo, 0, pos);
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
               "servo[%d] = (direction(%d) * ((range(%d) / 2) * value(%6.4f))) + %d = %d",
               servo,
               configp->direction,
               configp->range,
               value,
               configp->center,
               pos);
}

/**
 * \private Method to configure a servo on the active board.
 *
 * @param servo An int value (1..16).
 * @param center An int value gt 1.
 * @param range Int value gt 1.
 * @param direction An int  either -1 or 1.
 *Example config_server (1, 300, 100, -1)   // configure the first servo with a center of 300 and range of 100 and reversed direction.
 */
void BoardNode::config_servo(int servo, int center, int range, int direction) {
  if ((servo < 1) || (servo > (MAX_SERVOS))) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid servo number %d :: servo numbers must be between 1 and %d",
                 servo,
                 MAX_SERVOS);
    return;
  }

  if ((center < 0) || (center > 4096))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid center value %d :: center values must be between 0 and 4096",
                 center);

  if ((center < 0) || (center > 4096))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid range value %d :: range values must be between 0 and 4096",
                 range);

  if (((center - (range / 2)) < 0) || (((range / 2) + center) > 4096))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid range center combination %d ± %d :: range/2 ± center must be between 0 and 4096",
                 center,
                 (range / 2));

  this->servo_configs[servo - 1].center = center;
  this->servo_configs[servo - 1].range = range;
  this->servo_configs[servo - 1].direction = direction;

  if (servo > last_servo)    // Used for internal optimizations.
    last_servo = servo;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Servo #%d configured: center=%d, range=%d, direction=%d",
              servo,
              center,
              range,
              direction);
}

int BoardNode::config_servo_position(int servo, int position) {
  if ((servo < 1) || (servo > (MAX_SERVOS))) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid servo number %d :: servo numbers must be between 1 and %d",
                 servo,
                 MAX_SERVOS);
    return -1;
  }
  if ((position < POSITION_UNDEFINED) || (position > POSITION_RIGHTREAR)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid drive mode position %d :: positions are 0 = non-drive, 1 = left front, 2 = right front, 3 = left rear, and 4 = right rear",
                 position);
    return -1;
  }
  this->servo_configs[servo - 1].mode_pos = position;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servo #%d configured: position=%d", servo, position);
  return 0;
}

int BoardNode::config_drive_mode(const std::string &mode, float rpm, float radius, float track, float scale) {
  int mode_val = MODE_UNDEFINED;

  // Assumes the parameter was provided in the proper case.
  if (0 == strcmp(mode.c_str(), CONST("ackerman")))
    mode_val = MODE_ACKERMAN;
  else if (0 == strcmp(mode.c_str(), CONST("differential")))
    mode_val = MODE_DIFFERENTIAL;
  else if (0 == strcmp(mode.c_str(), CONST("mecanum")))
    mode_val = MODE_MECANUM;
  else {
    mode_val = MODE_INVALID;
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid drive mode %s :: drive mode must be one of ackerman, differential, or mecanum",
                 mode.c_str());
    return -1;
  }

  if (rpm <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid RPM %6.4f :: the motor's output RPM must be greater than 0.0",
                 rpm);
    return -1;
  }

  if (radius <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid radius %6.4f :: the wheel radius must be greater than 0.0 meters",
                 radius);
    return -1;
  }

  if (track <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid track %6.4f :: the axel track must be greater than 0.0 meters",
                 track);
    return -1;
  }

  if (scale <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid scale %6.4f :: the scalar for Twist messages must be greater than 0.0",
                 scale);
    return -1;
  }

  this->active_drive.mode = mode_val;
  this->active_drive.rpm = rpm;
  this->active_drive.radius = radius;    // The service takes the radius in meters.
  this->active_drive.track = track;        // The service takes the track in meters.
  this->active_drive.scale = scale;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Drive mode configured: mode=%s, rpm=%6.4f, radius=%6.4f, track=%6.4f, scale=%6.4f",
              mode.c_str(),
              rpm,
              radius,
              track,
              scale);
  return 0;
}

void BoardNode::setup(const char *filename) {
  int i;

  for (i = 0; i < MAX_BOARDS; i++) {
    this->pwm_boards[i] = -1;
  }

  this->active_board = -1;

  for (i = 0; i < (MAX_SERVOS); i++) {
    // These values have not useful meaning.
    this->servo_configs[i].center = -1;
    this->servo_configs[i].range = -1;
    this->servo_configs[i].direction = 1;
    this->servo_configs[i].mode_pos = -1;
  }

  this->last_servo = -1;

  this->active_drive.mode = MODE_UNDEFINED;
  this->active_drive.rpm = -1.0;
  this->active_drive.radius = -1.0;
  this->active_drive.track = -1.0;
  this->active_drive.scale = -1.0;

  if ((this->controller_io_handle = open(filename, O_RDWR)) < 0) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Failed to open I2C bus %s", filename);
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I2C bus opened on %s", filename);
}

int BoardNode::init(int io_device, int frequency) {

  this->controller_io_device = io_device;
  this->pwm_frequency = frequency;

  auto node = std::make_shared<BoardNode>("i2c_pwm_board_params");

  // Default I2C device on RPi2 and RPi3 = "/dev/i2c-1" Orange Pi Lite = "/dev/i2c-0".
  node->declare_parameter("i2c_device_number", this->controller_io_device);
  std::stringstream device;
  device << "/dev/i2c-" << this->controller_io_device;
  this->setup(device.str().c_str());

  this->set_active_internal_board(1); // Internal board is equal to one (considering boards inside the bus, and not external boards on different buses).

  int pwm = 50;
  node->declare_parameter("pwm_frequency", pwm);
  this->set_pwm_frequency(pwm);

  /*
    // Note: servos are numbered sequentially with '1' being the first servo on board #1, '17' is the first servo on board #2.
    servo_config:
     - {servo: 1, center: 333, direction: -1, range: 100}
     - {servo: 2, center: 336, direction: 1, range: 108}
  */
  // Attempt to load configuration for servos.
  if (node->has_parameter("servo_config")) {
    XmlRpc::XmlRpcValue servos;
    node->get_parameter("servo_config", servos);

    if (servos.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                   "Retrieving members from 'servo_config' in namespace(%s)",
                   node->get_namespace());

      for (int32_t i = 0; i < servos.size(); i++) {
        XmlRpc::XmlRpcValue servo;
        servo = servos[i];    // Get the data from the iterator.
        if (servo.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
          RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                       "Retrieving items from 'servo_config' member %d in namespace(%s)",
                       i,
                       node->get_namespace());

          // Get the servo settings.
          int id, center, direction, range;
          id = Xmlrp::get_int_param(servo, "servo");
          center = Xmlrp::get_int_param(servo, "center");
          direction = Xmlrp::get_int_param(servo, "direction");
          range = Xmlrp::get_int_param(servo, "range");

          if (id && center && direction && range) {
            if ((id >= 1) && (id <= MAX_SERVOS)) {
              int board = ((int) (id / 16)) + 1;
              this->set_active_internal_board(board);
              this->set_pwm_frequency(pwm);
              this->config_servo(id, center, range, direction);
            } else
              RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Parameter servo=%d is out of bounds", id);
          } else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Invalid parameters for servo=%d'", id);
        } else
          RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                      "Invalid type %d for member of 'servo_config' - expected TypeStruct(%d)",
                      servo.getType(),
                      XmlRpc::XmlRpcValue::TypeStruct);
      }
    } else
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "Invalid type %d for 'servo_config' - expected TypeArray(%d)",
                  servos.getType(),
                  XmlRpc::XmlRpcValue::TypeArray);
  } else
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                 "Parameter Server namespace[%s] does not contain 'servo_config",
                 node->get_namespace());
  /*
    drive_config:
        mode: mecanum
      radius: 0.062
      rpm: 60.0
      scale: 0.3
      track: 0.2
      servos:
          - {servo: 1, position: 1}
          - {servo: 2, position: 2}
          - {servo: 3, position: 3}
          - {servo: 4, position: 4}
  */

  // Attempt to load configuration for drive mode.
  if (node->has_parameter("drive_config")) {
    XmlRpc::XmlRpcValue drive;
    node->get_parameter("drive_config", drive);

    if (drive.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                   "Retrieving members from 'drive_config' in namespace(%s)",
                   node->get_namespace());

      // Get the drive mode settings.
      std::string mode;
      float radius, rpm, scale, track;

      mode = Xmlrp::get_string_param(drive, "mode");
      rpm = Xmlrp::get_float_param(drive, "rpm");
      radius = Xmlrp::get_float_param(drive, "radius");
      track = Xmlrp::get_float_param(drive, "track");
      scale = Xmlrp::get_float_param(drive, "scale");

      this->config_drive_mode(mode, rpm, radius, track, scale);

      XmlRpc::XmlRpcValue &servos = drive["servos"];
      if (servos.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                     "Retrieving members from 'drive_config/servos' in namespace(%s)",
                     node->get_namespace());

        for (int32_t i = 0; i < servos.size(); i++) {
          XmlRpc::XmlRpcValue servo;
          servo = servos[i];    // Get the data from the iterator.
          if (servo.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                         "Retrieving items from 'drive_config/servos' member %d in namespace(%s)",
                         i,
                         node->get_namespace());

            // Get the servo position settings.
            int id, position;
            id = Xmlrp::get_int_param(servo, "servo");
            position = Xmlrp::get_int_param(servo, "position");

            if (id && position)
              config_servo_position(id, position); // Had its own error reporting.
          } else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                        "Invalid type %d for member %d of 'drive_config/servos' - expected TypeStruct(%d)",
                        i,
                        servo.getType(),
                        XmlRpc::XmlRpcValue::TypeStruct);
        }
      } else
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Invalid type %d for 'drive_config/servos' - expected TypeArray(%d)",
                    servos.getType(),
                    XmlRpc::XmlRpcValue::TypeArray);
    } else
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "Invalid type %d for 'drive_config' - expected TypeStruct(%d)",
                  drive.getType(),
                  XmlRpc::XmlRpcValue::TypeStruct);
  } else
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                 "Parameter Server namespace[%s] does not contain 'drive_config",
                 node->get_namespace());

  return 1;
}

int BoardNode::get_active_board() const {
  return this->active_board;
}

int BoardNode::get_last_servo() const {
  return this->last_servo;
}

DriveMode BoardNode::get_active_drive() const {
  return this->active_drive;
}

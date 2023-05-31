#define _BASE_ADDR   0x40
#ifndef _PI
#define _PI 3.14159265358979323846
#endif
#define _CONST(s) ((char*)(s))
#define MAX_BOARDS 62
#define MAX_SERVOS (16*MAX_BOARDS)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

// Using "extern "C" {  for safety or warnings.
extern "C" {
  #include <linux/i2c.h>
  #include <linux/i2c-dev.h>
  #include <i2c/smbus.h>
}

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

// Messages used for any service with no parameters.
#include <std_srvs/srv/empty.hpp>
// Messages used for drive movement topic.
#include <geometry_msgs/msg/twist.hpp>

#include <xmlrpcpp/XmlRpc.h>

// Messages used for the absolute and proportional movement topics.
#include "i2c_pwm_board_msgs/msg/servo.hpp"
#include "i2c_pwm_board_msgs/msg/servo_array.hpp"

// Messages used for the servo setup service.
#include "i2c_pwm_board_msgs/msg/servo_config.hpp"
#include "i2c_pwm_board_msgs/msg/servo_config_array.hpp"

// Request/response of the servo setup service.
#include "i2c_pwm_board_msgs/srv/servos_config.hpp"

// request/response of the drive mode service.
#include "i2c_pwm_board_msgs/srv/drive_mode.hpp"
#include "i2c_pwm_board_msgs/msg/position.hpp"
#include "i2c_pwm_board_msgs/msg/position_array.hpp"

// Request/response of the integer parameter services.
#include "i2c_pwm_board_msgs/srv/int_value.hpp"

// The class that handles messages between the controller and ROS2.
class I2CPWMNode : public rclcpp::Node {
public:
    explicit I2CPWMNode(const std::string & node_name="i2c_pwm_controller", const std::string & node_namespace="/")
      : rclcpp::Node(node_name, node_namespace) {}
};

typedef struct _servo_config {
  int center;
  int range;
  int direction;
  int mode_pos;
} servo_config;

typedef struct _drive_mode {
  int mode;
  float rpm;
  float radius;
  float track;
  float scale;
} drive_mode;

enum drive_modes {
  MODE_UNDEFINED      = 0,
  MODE_ACKERMAN       = 1,
  MODE_DIFFERENTIAL   = 2,
  MODE_MECANUM        = 3,
  MODE_INVALID        = 4
};

enum drive_mode_positions {
  POSITION_UNDEFINED  = 0,
  POSITION_LEFTFRONT  = 1,
  POSITION_RIGHTFRONT = 2,
  POSITION_LEFTREAR   = 3,
  POSITION_RIGHTREAR  = 4,
  POSITION_INVALID    = 5
};

enum pwm_regs {
  // Registers / etc.
  __MODE1              = 0x00,
  __MODE2              = 0x01,
  __SUBADR1            = 0x02,      // Enable sub address 1 support.
  __SUBADR2            = 0x03,      // Enable sub address 2 support.
  __SUBADR3            = 0x04,      // Enable sub address 2 support.
  __PRESCALE           = 0xFE,
  __CHANNEL_ON_L       = 0x06,
  __CHANNEL_ON_H       = 0x07,
  __CHANNEL_OFF_L      = 0x08,
  __CHANNEL_OFF_H      = 0x09,
  __ALL_CHANNELS_ON_L  = 0xFA,
  __ALL_CHANNELS_ON_H  = 0xFB,
  __ALL_CHANNELS_OFF_L = 0xFC,
  __ALL_CHANNELS_OFF_H = 0xFD,
  __RESTART            = 0x80,
  __SLEEP              = 0x10,      // Enable low power mode.
  __ALLCALL            = 0x01,
  __INVRT              = 0x10,      // Invert the output control logic.
  __OUTDRV             = 0x04
};

servo_config _servo_configs[MAX_SERVOS];    // We can support up to 62 boards (1..62), each with 16 PWM devices (1..16).
drive_mode _active_drive;					// Used when converting Twist geometry to PWM values and which servos are for motion.
int _last_servo = -1;

int _pwm_boards[MAX_BOARDS];                // We can support up to 62 boards (1..62).
int _active_board = 0;                      // Used to determine if I2C SLAVE change is needed.
int _controller_io_handle;                  // Linux file handle for I2C.
int _controller_io_device;                  // Linux file for I2C.

int _pwm_frequency = 50;

// The next methods are only used for mathematical operations on the controller.
static float _abs (float v1) {
  if (v1 < 0)
      return (0 - v1);
  return v1;
}

static float _min (float v1, float v2) {
  if (v1 > v2)
    return v2;
  return v1;
}

static float _max (float v1, float v2) {
  if (v1 < v2)
    return v2;
  return v1;
}

static float _absmin (float v1, float v2) {
  float a1, a2;
  float sign = 1.0;
  //	if (v1 < 0)
  //		sign = -1.0;
  a1 = _abs(v1);
  a2 = _abs(v2);
  if (a1 > a2)
    return (sign * a2);
  return v1;
}

static float _absmax (float v1, float v2) {
  float a1, a2;
  float sign = 1.0;
  //	if (v1 < 0)
  //		sign = -1.0;
  a1 = _abs(v1);
  a2 = _abs(v2);
  if (a1 < a2)
    return (sign * a2);
  return v1;
}

/**
 \private Method to smooth a speed value.
 
 We calculate each speed using a cosine 'curve',  this results in the output curve 
 being shallow at 'stop', full forward, and full reverse and becoming 
 more aggressive in the middle or each direction.
 @param speed An int value (±1.0) indicating original speed.
 @returns An integer value (±1.0) smoothed for more gentle acceleration.
 */
static int _smoothing (float speed) {
    /* if smoothing is desired, then remove the commented code  */
    // speed = (cos(_PI*(((float)1.0 - speed))) + 1) / 2;
    return speed;
}

/**
   \private Method to convert meters per second to a proportional value in the range of ±1.0.
 
   @param speed Float requested speed in meters per second.
   @returns Float value (±1.0) for servo speed.
 */
static float _convert_mps_to_proportional (float speed)
{
	/* We use the drive mouter output rpm and wheel radius to compute the conversion. */

	float initial, max_rate;	// The max m/s is ((rpm/60) * (2*PI*radius)).
  initial = speed;

  if (_active_drive.rpm <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid active drive mode RPM %6.4f :: RPM must be greater than 0", _active_drive.rpm);
    return 0.0;
  }
  if (_active_drive.radius <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid active drive mode radius %6.4f :: wheel radius must be greater than 0", _active_drive.radius);
    return 0.0;
  }

  max_rate = (_active_drive.radius * _PI * 2) * (_active_drive.rpm / 60.0);

  speed = speed / max_rate;
  // speed = _absmin (speed, 1.0);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "%6.4f = convert_mps_to_proportional ( speed(%6.4f) / ((radus(%6.4f) * pi(%6.4f) * 2) * (rpm(%6.4f) / 60.0)) )", speed, initial, _active_drive.radius, _PI, _active_drive.rpm);
  return speed;
}

/**
 * \private Method to set a pulse frequency.
 *
 * The pulse defined by start/stop will be active on all channels until any subsequent call changes it.
 * @param frequency an int value (1..15000) indicating the pulse frequency where 50 is typical for RC servos
 * Example _set_frequency (68)  // set the pulse frequency to 68Hz
 */
static void _set_pwm_frequency (int freq)
{
  int prescale;
  char oldmode, newmode;
  int res;

  _pwm_frequency = freq;   // Save to global.
    
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "_set_pwm_frequency prescale");
  
  float prescaleval = 25000000.0; // 25MHz
  prescaleval /= 4096.0;
  prescaleval /= (float)freq;
  prescaleval -= 1.0;
  prescale = floor(prescaleval + 0.5);
  // prescale = (unsigned char)(25000000.0f / (4096.0f * freq) - 0.5f);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting PWM frequency to %d Hz", freq);

  nanosleep ((const struct timespec[]){{1, 000000L}}, NULL); 

  oldmode = i2c_smbus_read_byte_data (_controller_io_handle, __MODE1);
  newmode = (oldmode & 0x7F) | 0x10; // Sleep.

  if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __MODE1, newmode)) // Go to sleep.
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to set PWM controller to sleep mode"); 

  if (0 >  i2c_smbus_write_byte_data(_controller_io_handle, __PRESCALE, (prescale)))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to set PWM controller prescale"); 

  if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __MODE1, oldmode))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to set PWM controller to active mode"); 

  nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   // Sleep 5 microseconds.

  if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __MODE1, oldmode | 0x80))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to restore PWM controller to active mode");
}

/**
 * \private Method to set a common value for all PWM channels on the active board.
 *
 * The pulse defined by start/stop will be active on all channels until any subsequent call changes it.
 * @param start an int value (0..4096) indicating when the pulse will go high sending power to each channel.
 * @param end an int value (0..4096) indicating when the pulse will go low stoping power to each channel.
 * Example _set_pwm_interval_all (0, 108).   // set all servos with a pulse width of 105
 */
static void _set_pwm_interval_all (int start, int end) {
  // The public API is ONE based and hardware is ZERO based.
  if ((_active_board < 1) || (_active_board > 62)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Internal error - invalid active board number %d :: PWM board numbers must be between 1 and 62", _active_board);
    return;
  }
  int board = _active_board - 1;

  if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __ALL_CHANNELS_ON_L, start & 0xFF))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM start low byte for all servos on board %d", _active_board);
  if (0 >  i2c_smbus_write_byte_data (_controller_io_handle, __ALL_CHANNELS_ON_H, start  >> 8))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM start high byte for all servos on board %d", _active_board);
  if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __ALL_CHANNELS_OFF_L, end & 0xFF))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM end low byte for all servos on board %d", _active_board);
  if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __ALL_CHANNELS_OFF_H, end >> 8))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM end high byte for all servos on board %d", _active_board);
}

/**
 * \private Method to set the active board.
 *
 * @param board An int value (1..62) indicating which board to activate for subsequent service and topic subscription activity where 1 coresponds to the default board address of 0x40 and value increment up.
 * Example _set_active_board (68)   // set the pulse frequency to 68Hz.
 */
static void _set_active_board (int board) {
	char mode1res;

	if ((board < 1) || (board > 62)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Internal error :: invalid board number %d :: board numbers must be between 1 and 62", board);
    return;
  }

  if (_active_board != board) {
    _active_board = board;   // Save to global.
        
    // the public API is ONE based and hardware is ZERO based.
    board--;
        
    if (0 > ioctl (_controller_io_handle, I2C_SLAVE, (_BASE_ADDR+(board)))) {
      RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Failed to acquire bus access and/or talk to I2C slave at address 0x%02X", (_BASE_ADDR+board));
      return; /* exit(1) */   /* Additional ERROR HANDLING information is available with 'errno'. */
    }

    if (_pwm_boards[board] < 0) {
      _pwm_boards[board] = 1;

      /* This is guess but I believe the following needs to be done on each board only once. */

      if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __MODE2, __OUTDRV))
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to enable PWM outputs for totem-pole structure");

      if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __MODE1, __ALLCALL))
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to enable ALLCALL for PWM channels");

      nanosleep ((const struct timespec[]){{0, 5000000L}}, NULL);   // Sleep 5 microseconds, wait for osci.

      mode1res = i2c_smbus_read_byte_data (_controller_io_handle, __MODE1);
      mode1res = mode1res & ~__SLEEP; //                 # Wake up (reset sleep).

      if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __MODE1, mode1res))
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to recover from low power mode");
        
      nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec, wait for osci

      // The first time we activate a board, we mark it and set all of its servo channels to 0
      _set_pwm_interval_all(0, 0);
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
 *Example _set_pwm_interval (3, 0, 350)    // set servo #3 (fourth position on the hardware board) with a pulse of 350
 */
static void _set_pwm_interval(int servo, int start, int end) {
	RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "_set_pwm_interval enter");

    if ((servo<1) || (servo>(MAX_SERVOS))) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid servo number %d :: servo numbers must be between 1 and %d", servo, MAX_BOARDS);
        return;
    }

	int board = ((int)((servo-1)/16))+1;	// servo 1..16 is board #1, servo 17..32 is board #2, etc.
	_set_active_board(board);

	servo = ((servo-1) % 16) + 1;			// servo numbers are 1..16

  // the public API is ONE based and hardware is ZERO based
  board = _active_board - 1;				// the hardware enumerates boards as 0..61
  int channel = servo - 1;				// the hardware enumerates servos as 0..15
	RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "_set_pwm_interval board=%d servo=%d", board, servo);
    
  if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __CHANNEL_ON_L+4*channel, start & 0xFF))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM start low byte on servo %d on board %d", servo, _active_board);
  if (0 >  i2c_smbus_write_byte_data (_controller_io_handle, __CHANNEL_ON_H+4*channel, start  >> 8))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM start high byte on servo %d on board %d", servo, _active_board);
  if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __CHANNEL_OFF_L+4*channel, end & 0xFF))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM end low byte on servo %d on board %d", servo, _active_board);
  if (0 > i2c_smbus_write_byte_data (_controller_io_handle, __CHANNEL_OFF_H+4*channel, end >> 8))
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM end high byte on servo %d on board %d", servo, _active_board);
}

/**
 * \private Method to set a value for a PWM channel, based on a range of ±1.0, on the active board.
 *
 * The pulse defined by start/stop will be active on the specified servo channel until any subsequent call changes it.
 * @param servo an int value (1..16) indicating which channel to change power.
 * @param value an int value (±1.0) indicating when the size of the pulse for the channel.
 * Example _set_pwm_interval (3, 0, 350)    // set servo #3 (fourth position on the hardware board) with a pulse of 350.
 */
static void _set_pwm_interval_proportional (int servo, float value)
{
	// Need a little wiggle room to allow for accuracy of a floating point value
	if ((value < -1.0001) || (value > 1.0001)) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid proportion value %f :: proportion values must be between -1.0 and 1.0", value);
		return;
	}

	servo_config* configp = &(_servo_configs[servo-1]);
	
	if ((configp->center < 0) ||(configp->range < 0)) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Missing servo configuration for servo[%d]", servo);
		return;
	}

	int pos = (configp->direction * (((float)(configp->range) / 2) * value)) + configp->center;
        
	if ((pos < 0) || (pos > 4096)) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid computed position servo[%d] = (direction(%d) * ((range(%d) / 2) * value(%6.4f))) + %d = %d", servo, configp->direction, configp->range, value, configp->center, pos);
		return;
	}
	_set_pwm_interval(servo, 0, pos);
	RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "servo[%d] = (direction(%d) * ((range(%d) / 2) * value(%6.4f))) + %d = %d", servo, configp->direction, configp->range, value, configp->center, pos);
}

/**
 * \private Method to configure a servo on the active board.
 *
 * @param servo An int value (1..16).
 * @param center An int value gt 1.
 * @param range Int value gt 1.
 * @param direction An int  either -1 or 1.
 *Example _config_server (1, 300, 100, -1)   // configure the first servo with a center of 300 and range of 100 and reversed direction.
 */
static void _config_servo (int servo, int center, int range, int direction)
{
	if ((servo < 1) || (servo > (MAX_SERVOS))) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid servo number %d :: servo numbers must be between 1 and %d", servo, MAX_SERVOS);
		return;
	}

	if ((center < 0) || (center > 4096))
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid center value %d :: center values must be between 0 and 4096", center);

	if ((center < 0) || (center > 4096))
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid range value %d :: range values must be between 0 and 4096", range);

	if (((center - (range/2)) < 0) || (((range/2) + center) > 4096))
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid range center combination %d ± %d :: range/2 ± center must be between 0 and 4096", center, (range/2));

	_servo_configs[servo-1].center = center;
	_servo_configs[servo-1].range = range;
	_servo_configs[servo-1].direction = direction;
	// _servo_configs[servo-1].mode_pos = POSITION_UNDEFINED;

	if (servo > _last_servo)	// Used for internal optimizations.
		_last_servo = servo;

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servo #%d configured: center=%d, range=%d, direction=%d", servo, center, range, direction);
}

static int _config_servo_position (int servo, int position)
{
	if ((servo < 1) || (servo > (MAX_SERVOS))) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid servo number %d :: servo numbers must be between 1 and %d", servo, MAX_SERVOS);
		return -1;
	}
	if ((position < POSITION_UNDEFINED) || (position > POSITION_RIGHTREAR)) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid drive mode position %d :: positions are 0 = non-drive, 1 = left front, 2 = right front, 3 = left rear, and 4 = right rear", position);
		return -1;
	}
	_servo_configs[servo-1].mode_pos = position;
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servo #%d configured: position=%d", servo, position);
	return 0;
}

static int _config_drive_mode (std::string mode, float rpm, float radius, float track, float scale)
{
	int mode_val = MODE_UNDEFINED;

	// Assumes the parameter was provided in the proper case.
	if (0 == strcmp (mode.c_str(), _CONST("ackerman")))
		mode_val = MODE_ACKERMAN;
	else if (0 == strcmp (mode.c_str(), _CONST("differential")))
		mode_val = MODE_DIFFERENTIAL;
	else if (0 == strcmp (mode.c_str(), _CONST("mecanum")))
		mode_val = MODE_MECANUM;
	else {
		mode_val = MODE_INVALID;
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid drive mode %s :: drive mode must be one of ackerman, differential, or mecanum", mode.c_str());
		return -1;
	}

	if (rpm <= 0.0) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid RPM %6.4f :: the motor's output RPM must be greater than 0.0", rpm);
		return -1;
	}

	if (radius <= 0.0) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid radius %6.4f :: the wheel radius must be greater than 0.0 meters", radius);
		return -1;
	}

	if (track <= 0.0) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid track %6.4f :: the axel track must be greater than 0.0 meters", track);
		return -1;
	}

	if (scale <= 0.0) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid scale %6.4f :: the scalar for Twist messages must be greater than 0.0", scale);
		return -1;
	}

	_active_drive.mode = mode_val;
	_active_drive.rpm = rpm;
	_active_drive.radius = radius;	// The service takes the radius in meters.
	_active_drive.track = track;		// The service takes the track in meters.
	_active_drive.scale = scale;

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Drive mode configured: mode=%s, rpm=%6.4f, radius=%6.4f, track=%6.4f, scale=%6.4f", mode.c_str(), rpm, radius, track, scale);
	return 0;
}

/**
 \private Method to initialize private internal data structures at startup.
 @param devicename A string value indicating the linux I2C device.
 Example _init ("/dev/i2c-1");  // Default I2C device on RPi2 and RPi3 = "/dev/i2c-1".
 */
static void _init (const char* filename)
{
    int res;
    char mode1res;
    int i;

    /* Initialize all of the global data objects. */
    
    for (i = 0; i < MAX_BOARDS; i++)
        _pwm_boards[i] = -1; 
    _active_board = -1;

	for (i = 0; i < (MAX_SERVOS); i++) {
		// These values have not useful meaning.
		_servo_configs[i].center = -1;
		_servo_configs[i].range = -1;
		_servo_configs[i].direction = 1;
		_servo_configs[i].mode_pos = -1;
	}
	_last_servo = -1;

	_active_drive.mode = MODE_UNDEFINED;
	_active_drive.rpm = -1.0;
	_active_drive.radius = -1.0;
	_active_drive.track = -1.0;
	_active_drive.scale = -1.0;
	
	
    if ((_controller_io_handle = open (filename, O_RDWR)) < 0) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Failed to open I2C bus %s", filename);
        return; /* exit(1) */   /* Additional ERROR HANDLING information is available with 'errno'. */
    }
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I2C bus opened on %s", filename);
}

// Implementations

void servos_absolute (const std::shared_ptr<i2c_pwm_board_msgs::msg::ServoArray> msg) {
  /* This subscription works on the active_board. */
  for (std::vector<i2c_pwm_board_msgs::msg::Servo>::const_iterator sp = msg->servos.begin(); sp != msg->servos.end(); ++sp) {
    int servo = sp->servo;
    int value = sp->value;

    if ((value < 0) || (value > 4096)) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid PWM value %d :: PWM values must be between 0 and 4096", value);
      continue;
    }
    _set_pwm_interval(servo, 0, value);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "servo[%d] = %d", servo, value);
  }
}

void servos_proportional (const std::shared_ptr<i2c_pwm_board_msgs::msg::ServoArray> msg) {
  /* This subscription works on the active_board. */

  for (std::vector<i2c_pwm_board_msgs::msg::Servo>::const_iterator sp = msg->servos.begin(); sp != msg->servos.end(); ++sp) {
    int servo = sp->servo;
    float value = sp->value;
    _set_pwm_interval_proportional(servo, value);
  }
}

void servos_drive (const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
	/* This subscription works on the active_board. */

	int i;
	float delta, range, ratio;
	float temp_x, temp_y, temp_r;
	float dir_x, dir_y, dir_r;
	float speed[4];
	
	/* "msg" is a pointer to a Twist message: msg->linear and msg->angular each of which have members .x .y .z .*/
	/* the subscriber uses the maths from: http://robotsforroboticists.com/drive-kinematics/ . */	

	RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "servos_drive Twist = [%5.2f %5.2f %5.2f] [%5.2f %5.2f %5.2f]", 
			 msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);

	if (_active_drive.mode == MODE_UNDEFINED) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "drive mode not set");
		return;
	}
	if ((_active_drive.mode < MODE_UNDEFINED) || (_active_drive.mode >= MODE_INVALID)) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "unrecognized drive mode set %d", _active_drive.mode);
		return;
	}

	dir_x = ((msg->linear.x  < 0) ? -1 : 1);
	dir_y = ((msg->linear.y  < 0) ? -1 : 1);
	dir_r = ((msg->angular.z < 0) ? -1 : 1);

	temp_x = _active_drive.scale * _abs(msg->linear.x);
	temp_y = _active_drive.scale * _abs(msg->linear.y);
	temp_r = _abs(msg->angular.z);	// Radians.
		
	// temp_x = _smoothing (temp_x);
	// temp_y = _smoothing (temp_y);
	// temp_r = _smoothing (temp_r) / 2;

	// The differential rate is the robot rotational circumference / angular velocity.
	// Since the differential rate is applied to both sides in opposite amounts it is halved.
	delta = (_active_drive.track / 2) * temp_r;
	// Delta is now in meters/sec.

	// Determine if we will over-speed the motor and scal accordingly.
	ratio = _convert_mps_to_proportional(temp_x + delta);
	if (ratio > 1.0)
		temp_x /= ratio;

	
	switch (_active_drive.mode) {

	case MODE_ACKERMAN:
		/*
		  With ackerman drive, steering is handled by a separate servo.
		  We drive assigned servos exclusively by the linear.x.
		*/
		speed[0] = temp_x * dir_x;
		speed[0] = _convert_mps_to_proportional(speed[0]);
		if (_abs(speed[0]) > 1.0)
			speed[0] = 1.0 * dir_x;
		
		RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "ackerman drive mode speed=%6.4f", speed[0]);
		break;

	case MODE_DIFFERENTIAL:
		/*
		  With differential drive, steering is handled by the relative speed of left and right servos.
		  We drive assigned servos by mixing linear.x and angular.z.
		  We compute the delta for left and right components.
		  We use the sign of the angular velocity to determine which is the faster / slower.
		*/

		/* The delta is the angular velocity * half the drive track. */
		
		if (dir_r > 0) {	// turning right
			speed[0] = (temp_x + delta) * dir_x;
			speed[1] = (temp_x - delta) * dir_x;
		} else {		// turning left
			speed[0] = (temp_x - delta) * dir_x;
			speed[1] = (temp_x + delta) * dir_x;
		}

		RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "computed differential drive mode speed left=%6.4f right=%6.4f", speed[0], speed[1]);

		/* Ff any of the results are greater that 1.0, we need to scale all the results down. */
		range = _max (_abs(speed[0]), _abs(speed[1]));
		
		ratio = _convert_mps_to_proportional(range);
		if (ratio > 1.0) {
			speed[0] /= ratio;
			speed[1] /= ratio;
		}
		RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "adjusted differential drive mode speed left=%6.4f right=%6.4f", speed[0], speed[1]);

		speed[0] = _convert_mps_to_proportional(speed[0]);
		speed[1] = _convert_mps_to_proportional(speed[1]);

		RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "differential drive mode speed left=%6.4f right=%6.4f", speed[0], speed[1]);
		break;

	case MODE_MECANUM:
		/*
		  With mecanum drive, steering is handled by the relative speed of left and right servos.
		  With mecanum drive, lateral motion is handled by the rotation of front and rear servos.
		  We drive assigned servos by mixing linear.x and angular.z  and linear.y.
		*/

		if (dir_r > 0) {	// Turning right.
			speed[0] = speed[2] = (temp_x + delta) * dir_x;
			speed[1] = speed[3] = (temp_x - delta) * dir_x;
		} else {		// Turning left.
			speed[0] = speed[2] = (temp_x - delta) * dir_x;
			speed[1] = speed[3] = (temp_x + delta) * dir_x;
		}

		speed[0] += temp_y * dir_y;
		speed[3] += temp_y * dir_y;
		speed[1] -= temp_y * dir_y;
		speed[2] -= temp_y * dir_y;
		RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "computed mecanum drive mode speed leftfront=%6.4f rightfront=%6.4f leftrear=%6.4f rightreer=%6.4f", speed[0], speed[1], speed[2], speed[3]);

		range = _max (_max (_max (_abs(speed[0]), _abs(speed[1])), _abs(speed[2])), _abs(speed[3]));
		ratio = _convert_mps_to_proportional(range);
		if (ratio > 1.0) {
			speed[0] /= ratio;
			speed[1] /= ratio;
			speed[2] /= ratio;
			speed[3] /= ratio;
		}
		RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "adjusted mecanum drive mode speed leftfront=%6.4f rightfront=%6.4f leftrear=%6.4f rightreer=%6.4f", speed[0], speed[1], speed[2], speed[3]);

		speed[0] = _convert_mps_to_proportional(speed[0]);
		speed[1] = _convert_mps_to_proportional(speed[1]);
		speed[2] = _convert_mps_to_proportional(speed[2]);
		speed[3] = _convert_mps_to_proportional(speed[3]);

		RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "mecanum drive mode speed leftfront=%6.4f rightfront=%6.4f leftrear=%6.4f rightreer=%6.4f", speed[0], speed[1], speed[2], speed[3]);
		break;

	default:
		break;

	}
	
	/* Find all drive servos and set their new speed. */
	for ( i = 0; i < (_last_servo); i++) {
		// We use 'fall thru' on the switch statement to allow all necessary servos to be controlled.
		switch (_active_drive.mode) {
		case MODE_MECANUM:
			if (_servo_configs[i].mode_pos == POSITION_RIGHTREAR)
				_set_pwm_interval_proportional (i+1, speed[3]);
			if (_servo_configs[i].mode_pos == POSITION_LEFTREAR)
				_set_pwm_interval_proportional (i+1, speed[2]);
		case MODE_DIFFERENTIAL:
			if (_servo_configs[i].mode_pos == POSITION_RIGHTFRONT)
			_set_pwm_interval_proportional (i+1, speed[1]);
		case MODE_ACKERMAN:
			if (_servo_configs[i].mode_pos == POSITION_LEFTFRONT)
				_set_pwm_interval_proportional (i+1, speed[0]);
		}
	}
}

bool set_pwm_frequency (const std::shared_ptr<i2c_pwm_board_msgs::srv::IntValue::Request> req, std::shared_ptr<i2c_pwm_board_msgs::srv::IntValue::Response> res) {
  int freq;
  freq = req->value;
  if ((freq < 12) || (freq > 1024)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid PWM frequency %d :: PWM frequencies should be between 12 and 1024", freq);
    freq = 50;	// Most analog RC servos are designed for 20ms pulses.
    res->error = freq;
  }
  
  _set_pwm_frequency (freq);	// I think we must reset frequency when we change boards.
  res->error = freq;
  return true;
}

bool config_servos (const std::shared_ptr<i2c_pwm_board_msgs::srv::ServosConfig::Request> req, std::shared_ptr<i2c_pwm_board_msgs::srv::ServosConfig::Response> res) {
  /* This service works on the active_board. */
  int i;
    
  res->error = 0;

  if ((_active_board < 1) || (_active_board > 62)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Internal error - invalid board number %d :: PWM board numbers must be between 1 and 62", _active_board);
    res->error = -1;
    return true;
  }

  for (i = 0;i < req->servos.size(); i++) {
    int servo = req->servos[i].servo;
    int center = req->servos[i].center;
    int range = req->servos[i].range;
    int direction = req->servos[i].direction;

    _config_servo (servo, center, range, direction);
  }
    
  return true;
}

bool config_drive_mode (const std::shared_ptr<i2c_pwm_board_msgs::srv::DriveMode::Request> req, std::shared_ptr<i2c_pwm_board_msgs::srv::DriveMode::Response> res) {
  res->error = 0;

  int i;

  if ((res->error = _config_drive_mode (req->mode, req->rpm, req->radius, req->track, req->scale)))
    return true;

  for (i=0;i<req->servos.size();i++) {
    int servo = req->servos[i].servo;
    int position = req->servos[i].position;

    if (_config_servo_position (servo, position) != 0) {
      res->error = servo; /* this needs to be more specific and indicate a bad server ID was provided */
      continue;
    }
  }

  return true;
}

/**
   \brief service to stop all servos on all boards
   A service to stop all of the servos on all of the PWM boards and set their power state to off / coast.
   This is different from setting each servo to its center value. A centered servo is still under power and it's in a brake state.
   @param req is empty
   @param res is empty
   @returns true
   __Example__
   \code{.sh}
   # stop all servos on all boards and setting them to coast rather than brake
   rosservice call /stop_servos
   \endcode
 */
bool stop_servos (const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) {
	int save_active = _active_board;
	int i = 0;

	for (i=0; i<MAX_BOARDS; i++) {
		if (_pwm_boards[i] > 0) {
			_set_active_board (i+1);	// API is ONE based
			_set_pwm_interval_all (0, 0);
		}
	}
	_set_active_board (save_active);	// restore last active board
	return true;
}

static std::string _get_string_param (XmlRpc::XmlRpcValue obj, std::string param_name) {
  XmlRpc::XmlRpcValue &item = obj[param_name];
  if (item.getType() == XmlRpc::XmlRpcValue::TypeString)
    return item;

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "invalid paramter type for %s - expected TypeString", param_name.c_str());
  return 0;
}

static int _get_int_param (XmlRpc::XmlRpcValue obj, std::string param_name) {
  XmlRpc::XmlRpcValue &item = obj[param_name];
  if (item.getType() == XmlRpc::XmlRpcValue::TypeInt)
    return item;

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "invalid paramter type for %s - expected TypeInt", param_name.c_str());
  return 0;
}
    
static double _get_float_param (XmlRpc::XmlRpcValue obj, std::string param_name) {
  XmlRpc::XmlRpcValue &item = obj[param_name];
  if (item.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    return item;

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "invalid paramter type for %s - expected TypeDouble", param_name.c_str());
  return 0;
}
    
static int _load_params (void) {		
  auto node = std::make_shared<I2CPWMNode>("i2c_pwm_controller1");

  // Default I2C device on RPi2 and RPi3 = "/dev/i2c-1" Orange Pi Lite = "/dev/i2c-0".
  node->declare_parameter("i2c_device_number", _controller_io_device, 1); .
  std::stringstream device;
  device << "/dev/i2c-" << _controller_io_device;
  _init(device.str().c_str());

  _set_active_board (1);

  int pwm;
  node->declare_parameter("pwm_frequency", pwm , 50);
  _set_pwm_frequency (pwm);

  /*
    // Note: servos are numbered sequntially with '1' being the first servo on board #1, '17' is the first servo on board #2.
    servo_config:
     - {servo: 1, center: 333, direction: -1, range: 100}
     - {servo: 2, center: 336, direction: 1, range: 108}
  */
  // Attempt to load configuration for servos.
  if(node->has_parameter("servo_config")) {
    XmlRpc::XmlRpcValue servos;
    node->get_parameter("servo_config", servos);

    if(servos.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Retrieving members from 'servo_config' in namespace(%s)", node->get_namespace());
                
      for(int32_t i = 0; i < servos.size(); i++) {
        XmlRpc::XmlRpcValue servo;
        servo = servos[i];	// Get the data from the iterator.
        if(servo.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
          RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Retrieving items from 'servo_config' member %d in namespace(%s)", i, node->get_namespace());

          // Get the servo settings.
          int id, center, direction, range;
          id = _get_int_param (servo, "servo");
          center = _get_int_param (servo, "center");
          direction = _get_int_param (servo, "direction");
          range = _get_int_param (servo, "range");
                    
          if (id && center && direction && range) {
            if ((id >= 1) && (id <= MAX_SERVOS)) {
              int board = ((int)(id / 16)) + 1;
              _set_active_board (board);
              _set_pwm_frequency (pwm);
              _config_servo (id, center, range, direction);
            }
            else
              RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Parameter servo=%d is out of bounds", id);
          }
          else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Invalid parameters for servo=%d'", id);
        }
        else
          RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Invalid type %d for member of 'servo_config' - expected TypeStruct(%d)", servo.getType(), XmlRpc::XmlRpcValue::TypeStruct);
      }
    }
    else
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Invalid type %d for 'servo_config' - expected TypeArray(%d)", servos.getType(), XmlRpc::XmlRpcValue::TypeArray);
  }
  else
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Parameter Server namespace[%s] does not contain 'servo_config", node->get_namespace());


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
  if(node->has_parameter("drive_config")) {
    XmlRpc::XmlRpcValue drive;
    node->get_parameter("drive_config", drive);

    if(drive.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Retrieving members from 'drive_config' in namespace(%s)", node->get_namespace());

      // Get the drive mode settings.
      std::string mode;
      float radius, rpm, scale, track;
      int id, position;

      mode = _get_string_param (drive, "mode");
      rpm = _get_float_param (drive, "rpm");
      radius = _get_float_param (drive, "radius");
      track = _get_float_param (drive, "track");
      scale = _get_float_param (drive, "scale");

      _config_drive_mode (mode, rpm, radius, track, scale);

      XmlRpc::XmlRpcValue &servos = drive["servos"];
      if(servos.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Retrieving members from 'drive_config/servos' in namespace(%s)", node->get_namespace());
                
        for(int32_t i = 0; i < servos.size(); i++) {
          XmlRpc::XmlRpcValue servo;
          servo = servos[i];	// Get the data from the iterator.
          if(servo.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Retrieving items from 'drive_config/servos' member %d in namespace(%s)", i, node->get_namespace());

            // Get the servo position settings.
            int id, position;
            id = _get_int_param (servo, "servo");
            position = _get_int_param (servo, "position");
                    
            if (id && position)
               _config_servo_position (id, position); // Had its own error reporting.
            }
            else
              RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Invalid type %d for member %d of 'drive_config/servos' - expected TypeStruct(%d)", i, servo.getType(), XmlRpc::XmlRpcValue::TypeStruct);
        }
      }
      else
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Invalid type %d for 'drive_config/servos' - expected TypeArray(%d)", servos.getType(), XmlRpc::XmlRpcValue::TypeArray);
    }
    else
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Invalid type %d for 'drive_config' - expected TypeStruct(%d)", drive.getType(), XmlRpc::XmlRpcValue::TypeStruct);
  }
  else
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Parameter Server namespace[%s] does not contain 'drive_config", node->get_namespace());
}	

int main (int argc, char **argv) {	
	rclcpp::init(argc, argv);

	auto node = std::make_shared<I2CPWMNode>("i2c_pwm_controller2");

	// globals
	_controller_io_device = 1;	// default I2C device on RPi2 and RPi3 = "/dev/i2c-1" Orange Pi Lite = "/dev/i2c-0"
	_controller_io_handle = 0;
	_pwm_frequency = 50;		// set the initial pulse frequency to 50 Hz which is standard for RC servos

	rclcpp::Service<i2c_pwm_board_msgs::srv::IntValue>::SharedPtr freq_srv         = node->create_service<i2c_pwm_board_msgs::srv::IntValue>("set_pwm_frequency", &set_pwm_frequency);
	rclcpp::Service<i2c_pwm_board_msgs::srv::ServosConfig>::SharedPtr config_srv   =	node->create_service<i2c_pwm_board_msgs::srv::ServosConfig>("config_servos", &config_servos);			// 'config' will setup the necessary properties of continuous servos and is helpful for standard servos
	rclcpp::Service<i2c_pwm_board_msgs::srv::DriveMode>::SharedPtr mode_srv        =	node->create_service<i2c_pwm_board_msgs::srv::DriveMode>("config_drive_mode", &config_drive_mode);		// 'mode' specifies which servos are used for motion and which behavior will be applied when driving
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv                     = node->create_service<std_srvs::srv::Empty>("stop_servos", &stop_servos);			// the 'stop' service can be used at any time

	rclcpp::Subscription<i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr abs_sub   = node->create_subscription<i2c_pwm_board_msgs::msg::ServoArray>("servos_absolute", 500, &servos_absolute);		// the 'absolute' topic will be used for standard servo motion and testing of continuous servos
	rclcpp::Subscription<i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr rel_sub   = node->create_subscription<i2c_pwm_board_msgs::msg::ServoArray>("servos_proportional", 500, &servos_proportional);	// the 'proportion' topic will be used for standard servos and continuous rotation aka drive servos
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr          drive_sub = node->create_subscription<geometry_msgs::msg::Twist>("servos_drive", 500, &servos_drive);			// the 'drive' topic will be used for continuous rotation aka drive servos controlled by Twist messages
	
	_load_params();	// loads parameters and performs initialization
	
	rclcpp::spin(node);

	close (_controller_io_handle);
  rclcpp::shutdown();
}

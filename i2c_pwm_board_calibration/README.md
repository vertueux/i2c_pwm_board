# The I²C PWM Board Calibration package

> **Note**: It works such that it modifies only one servo per run.

Simply, a package to facilitate servo calibration. The Calibration package lets you control servos via a command-line interface. It helps you finding the center & critical values of your servos using numerical values.

## How it works

It works by publishing values to the `main_servos_absolute/` & `secondary_servos_absolute/` topic.

Know that `servo number = port + 1` on PCA9685 based boards.

## Run it

To launch the project and the executable after compilation, you'll need to enter this command on a terminal:

```bash
ros2 run i2c_pwm_board_calibration node
```

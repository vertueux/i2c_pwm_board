## Installing ROS2

This is a tutorial on how to install ROS2, configure I²C & install the I²C PWM Board package.

### Set locale

Make sure you have a locale which supports ```UTF-8```. If you are in a minimal environment (such as a docker container), the locale may be something minimal like POSIX. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### Setup Sources

You will need to add the ROS 2 apt repository to your system. First, make sure that the [Ubuntu Universe repository](https://help.ubuntu.com/community/Repositories/Ubuntu) is enabled by checking the output of this command.

```bash
apt-cache policy | grep universe
```

This should output a line like the one below :

```bash
500 http://us.archive.ubuntu.com/ubuntu jammy/universe amd64 Packages
    release v=22.04,o=Ubuntu,a=jammy,n=jammy,l=Ubuntu,c=universe,b=amd64
```

If you don’t see an output line like the one above, then enable the Universe repository with these instructions.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Now add the ROS 2 apt repository to your system. First authorize our GPG key with apt.

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

```

Then add the repository to your sources list.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS 2 packages

Update your apt repository caches after setting up the repositories.

```bash
sudo apt update
```

ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.

```bash
sudo apt upgrade
```

Desktop Install (Recommended) : ROS, RViz, demos, tutorials.

```bash
sudo apt install ros-humble-desktop
```

ROS-Base Install (Bare Bones) : Communication libraries, message packages, command line tools. No GUI tools.

```bash
sudo apt install ros-humble-ros-base
```

### Install necessary packages

Note that the ROS2 packages should be included in the two installs above. 
If you need to install them, just copy this line and replace ```PACKAGE_NAME``` by the name of the package needed.

```bash
sudo apt install ros-humble-PACKAGE_NAME
```

### Environment setup

#### Sourcing the setup script

Set up your environment by sourcing the following file.

```bash
source /opt/ros/humble/setup.bash
```

### Installing colcon

```bash
sudo apt install python3-colcon-common-extensions
```


## Install the I²C PWM Board 

You need to have these packages provided by the default desktop installation below : 

* **rclcpp, std_msgs, std_srvs, geometry_msgs**
* **rosidl_default_generators, rosidl_default_runtime**
* Have ```python3-colcon-common-extensions``` installed
* Have ```libi2c-dev``` and ```i2c-tools``` installed
* Have the [xmlrpcpp](https://github.com/bpwilcox/xmlrpcpp) package

### Clone it 

You can clone and run this package by copying the command below : 

* Note that if you want to run this project, you have to clone the xmlrpcpp packages : 

```bash
git clone --recursive https://github.com/vertueux/i2c_pwm_board.git
```

### Configure Ubuntu

Make sure that I²C is enabled by default. Check the file */boot/firmware/syscfg.txt* and see if you have the following line:

```txt
dtparam=i2c_arm=on
```

If not, maybe you can append them on */boot/firmware/usercfg.txt* and reboot, and hopefully, that works. If that doesn't work, maybe do `sudo apt update && sudo full-upgrade -y`  and see if there are any distro updates needed.

* Refer to [this post](https://askubuntu.com/questions/1273700/enable-spi-and-i2c-on-ubuntu-20-04-raspberry-pi/1273900#1273900).

You can also add the following line to */etc/modules*:

```bash
i2c-dev
```

* Refer to [this post](https://raspberrypi.stackexchange.com/questions/61905/enable-i2c-on-ubuntu-mate-raspberry-pi-3).

---

On a RPi, with `raspi-config`, you can enable I²C by navigating to *Interface Options->Advanced->I²C* and then enable it.

### Testing I²C

Now when you log in you can type the following command to see all the connected devices:

```bash
sudo i2cdetect -y 1 # Or 0, depends on the device you use.
```

## Build it

```bash
source /opt/ros/humble/setup.bash # With Debian binaries.
cd /i2c_pwm_board/
colcon build 
source install/setup.bash # Do not change directory.
```

## Run it

In order to run the project, you just have to perform this command :

```bash
ros2 run i2c_pwm_board node # You can add your I²C bus address as a digit (like 1 to open i2c-1/).
```


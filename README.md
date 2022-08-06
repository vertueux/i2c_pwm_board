<h1 align="center">
  <img src="docs/images/i2c-micro-controller.png" width="200px">
  <p align="center">
    I2C PWM Board
  </p>
</h1>

**ROS2** (**R**obot **O**perating **S**ystem **2**) node for controlling PWM boards based on the PCA9685 chip with an I2C interface. Primary use is for controlling RC Servos and DC motors via PWM. This is based on [ros-i2cpwmboard](https://gitlab.com/bradanlane/ros-i2cpwmboard) *(OUTDATED)* with updates to make it work on ROS2.

## Dependencies

You need to have ROS2 installed (of course) and theses packages provided by the default desktop installation below : 

* **rclcpp, std_msgs, std_srvs, geometry_msgs**
* **rosidl_default_generators, rosidl_default_runtime**
* Have ```python3-colcon-common-extensions``` installed
* Have ```libi2c-dev``` and ```i2c-tools``` installed
* Have the [xmlrpcpp](https://github.com/bpwilcox/xmlrpcpp) package

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

This should output a line like the one below:

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

Desktop Install (Recommended): ROS, RViz, demos, tutorials.

```bash
sudo apt install ros-humble-desktop
```

ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools.

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

### Install automatically

You can install the i2c library and colcon by running the install scripts located at `install_scripts/install_dependencies.sh`.
Simply just copy & paste this code:

```sh
cd install_script/
chmod +x install_dependencies.sh
./install_dependencies
```

### Installing colcon

```bash
sudo apt install python3-colcon-common-extensions
```

### Clone & Build it 

You can clone and run this package by copying the commands below: 
Note that if you want to run this project, you have to clone the xmlrpcpp packages and have the i2c library, which you can by copying this text below : 

```bash
git clone --recursive https://github.com/vertueux/i2cpwm_board.git
```

### Build it 

```bash
source /opt/ros/humble/setup.bash # With Debian binaries 
cd /i2cpwm_board/
colcon build --packages-select i2cpwm_board i2cpwm_board_msgs xmlrpcpp
source install/setup.bash # Do not change directory
```

--- 

*For now, it runs with warnings, but it is planned to fix them later on in the development.*

*Note also that this project is still in testing and does not have stable version.*
echo "Updating apt-get"
sudo apt update -y
sudo apt-get update -y

echo "Installing colcon"
sudo apt install -y python3-colcon-common-extensions

echo "Installing git"
sudo apt install -y git

echo "Installing the i2c library"
sudo apt install -y libi2c-dev i2c-tools
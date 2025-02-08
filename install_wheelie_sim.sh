echo 'Installing Wheelie Sim requirements'
echo ''

# AirSim
sudo bash ~/Formula-Student-Driverless-Simulator/AirSim/setup.sh

# ROS2 build
cd ~/Formula-Student-Driverless-Simulator/ros2/
rosdep install --from-paths src --ignore-src -ry
colcon build

# Binaries
cd ~/Formula-Student-Driverless-Simulator/
wget https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-linux.zip
unzip fsds-v2.2.0-linux.zip -d engine
rm fsds-v2.2.0-linux.zip

# Tmuxinator

sudo apt install tmuxinator btop

# Symlinks
sudo ln -s ~/Formula-Student-Driverless/wheelie/start_simulator.sh /usr/local/bin/wheelieSim

echo 'Installation complete'
echo ''

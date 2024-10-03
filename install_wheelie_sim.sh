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

# Zellij
case $(uname -m) in
    "x86_64"|"aarch64")
        arch=$(uname -m)
    ;;
    "arm64")
        arch="aarch64"
    ;;
    *)
        echo "Unsupported cpu arch: $(uname -m)"
        exit 2
    ;;
esac

case $(uname -s) in
    "Linux")
        sys="unknown-linux-musl"
    ;;
    "Darwin")
        sys="apple-darwin"
    ;;
    *)
        echo "Unsupported system: $(uname -s)"
        exit 2
    ;;
esac

url="https://github.com/zellij-org/zellij/releases/latest/download/zellij-$arch-$sys.tar.gz"
curl --location "$url" | sudo tar -C "/bin/" -xz
if [[ $? -ne 0 ]]
then
    echo
    echo "Extracting binary failed, cannot launch zellij :("
    echo "One probable cause is that a new release just happened and the binary is currently building."
    echo "Maybe try again later? :)"
    exit 1
fi

# Wheelie Sim



# Symlinks
sudo ln -s ~/Formula-Student-Driverless/wheelie_sim/run.sh /usr/local/bin/wheelieSim

echo 'Installation complete'
echo ''

# Source installation

## Prerequisites
- OS
    - Ubuntu 22.04 LTS or macOS Sonoma
- ROS
    - ROS 2 Humble
- Git
- pip
- curl


```bash
sudo apt update
sudo apt install git curl python3-pip
```

## Set up development environment

1. Install CUDA and cuDNN (Optional)

    Please access the NVIDIA website and install CUDA 11.8 along with the corresponding cuDNN. Once you can execute `nvcc --version` the installation is complete.

2. Install ROS 2

    ```bash
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install ros-humble-desktop ros-dev-tools
    ```

3. rosdep setup

    ```bash
    sudo rosdep init
    rosdep update
    ```

4. Install PyTorch

    ```bash
    python3 -m pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
    ```

5. Workspace setup

    ```bash
    mkdir -p ~/ros_ws/src
    cd ~/ros_ws/src
    git clone https://github.com/hakoroboken/roboware-neo.universe.git
    cd ~/ros_ws
    rosdep install -y --from-paths src --ignore-src --skip-keys=OpenCV --skip-keys=PCL --rosdistro humble
    ```
6. Compile

    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

## Additional Component Setup

1. Install Foxglove and MCAP.

    ```bash
    sudo apt install ros-humble-foxglove-bridge ros-humble-rosbag2-storage-mcap
    ```
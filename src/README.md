# MKS Stepper Motor Driver with CAN

## Table of Contents
- [Install Python Library](#install-python-library)
- [Use This Package](#use-this-package)
- [Rus Ros2 Node](#run-ros2-node)

## Install Python Library
```bash
sudo apt install python3-can
sudo apt install python3-serial
sudo apt install python3-yaml
sudo apt install python3-pynput
```

## Use This Package
1. Clone repo
    ```bash
    cd ~
    git clone https://github.com/phattanaratjeedjeen-sudo/canbus_ws.git
    ```

2. Build package
    ```bash
    cd canbus-ws
    colcon build && source install/setup.bash
    ```

3. Setup environment
    ```bash
    echo "source ~/canbus_ws/install/setup.bash" --> ~/.bashrc
    source ~/.bashrc
    ```

## Run ros2 node 
1. Run teleop node
    ```bash
    ros2 run hardware_control teleop_jog.py 
    ```

2. Run motor control node
    ```bash
    # On new terminal
    ros2 run hardware_control motor_control.py
    ```
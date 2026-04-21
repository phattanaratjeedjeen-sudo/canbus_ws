# MKS Stepper Motor Driver with CAN


## Table of Contents
- [Install CAN Lib](#install-can-lib)
- [Network Interfaces Setup](#network-interfaces-setup)
- []

## Install CAN Lib
```bash
cd ~/canbus_ws

# create python env
python3 -m venv .venv
source .venv/bin/activate

# install lib
pip install python-can
```


## Network Interfaces Setup
**Pyhsical CAN Hardware**
```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```
**Virtual CAN**
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

sudo apt install python3-can
sudo apt install python3-serial
sudo apt install python3-yaml
sudo apt install python3-pynput

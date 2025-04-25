---
description: >-
  Step-by-step guide to set up MAVROS for live MAVLINK data visualization.
---

# Installation and Setup Guide

## Prerequisites

Ensure the following requirements are met before proceeding:

1. **Operating System**: Ubuntu 22.04 LTS.
2. **Required Packages**: Install these via `apt`:
   ```bash
   sudo apt update && sudo apt install git wget curl make build-essential python3 python3-pip
   ```
3. **Sudo Permissions**: Ensure you have administrative privileges.

---

## Step 1: Install PX4

Follow the official PX4 installation guide for Ubuntu:

[PX4 Development Environment Setup](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)

> **Note**: Use the instructions for the [Gazebo Simulator](https://docs.px4.io/main/en/sim_gazebo_gz/) ("Harmonic") on Ubuntu 22.04.

---

## Step 2: Install ROS2

Follow the official ROS2 installation guide:

[ROS2 Installation Guide](https://docs.px4.io/main/en/ros2/user_guide.html#installation-setup)

> **Note**: Use the instructions for ROS2 "Humble".

---

## Step 3: Install QGroundControl

Follow the steps to install QGroundControl:

[QGroundControl Daily Build](https://docs.px4.io/main/en/dev_setup/qgc_daily_build.html)

> **Important**: Download QGroundControl in your home directory. Run the following command to ensure this:
> ```bash
> cd ~
> ```

---

## Step 4: Install PlotJuggler

Install PlotJuggler using the Snap package manager:

```bash
sudo snap install plotjuggler
```

---

## Step 5: Install MAVROS

Install MAVROS and its dependencies with the following commands:

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
sudo sh install_geographiclib_datasets.sh
```

---

## Step 6: Enable Port Forwarding in QGroundControl

To enable port forwarding in QGroundControl:

1. Launch QGroundControl:
   ```bash
   cd ~
   ./QGroundControl.AppImage
   ```
2. Follow the steps shown in the screenshots below to enable port forwarding:

   ![Step 2](.gitbook/assets/Step2.png)
   ![Step 3](.gitbook/assets/Step3.png)
   ![Step 4](.gitbook/assets/Step4.png)
   ![Step 5](.gitbook/assets/Step5.png)

---

## Step 7: Testing the Setup

Open four terminal windows and execute the following commands in each:

### Terminal 1: Start PX4 with Gazebo

```bash
cd PX4-Autopilot/
make px4_sitl gz_x500
```

### Terminal 2: Launch QGroundControl

```bash
cd ~
./QGroundControl.AppImage
```

### Terminal 3: Start MAVROS

```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557 -p target_component_id:=1 -r __ns:=/mavros
```

### Terminal 4: Launch PlotJuggler

```bash
plotjuggler
```

---

## Step 8: Using PlotJuggler

To visualize live data in PlotJuggler:

1. Click the **Start** button under the "Streaming" section on the left side of the screen.
2. Ensure "ROS Topic Subscriber" is selected.

![PlotJuggler Interface](.gitbook/assets/image.png)

> **Note**: Verify that the correct ROS topic is being subscribed to for live data visualization.

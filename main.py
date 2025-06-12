# This script sets up a data visualization environment using either Foxglove or Plotjuggler.
import argparse
import subprocess
import sys
import os
import signal
import time

# Store all subprocess.Popen objects
processes = []

# run_cmd function is used to execute shell commands in the background
def run_cmd(cmd, cwd=None):
    # Run command as current user with full environment, in correct directory
    p = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        cwd=cwd,
        env=os.environ.copy()
    )
    processes.append(subprocess.Popen(cmd, cwd=cwd, env=os.environ.copy()))

# Function to handle cleanup on exit
def terminate_processes():
    print("Terminating all processes...")
    for process in processes:
        try:
            process.send_signal(signal.SIGINT)  # Send SIGINT to gracefully stop the process
            process.wait(timeout=5)  # Wait for the process to terminate
        except subprocess.TimeoutExpired:
            process.kill()  # Force kill if it doesn't terminate in time
    print("All processes terminated.")

# Setup function to initialize the data visualization environment
def run_setup(choice):
    print("Initiating Data Visualization Setup...")
    home = os.path.expanduser("~")
    # Start PX4 and Gazebo
    run_cmd(["make", "px4_sitl", "gz_x500"], cwd=os.path.join(home, "PX4-Autopilot"))
    # Start QGroundControl
    run_cmd(["./QGroundControl-x86_64.AppImage"], cwd=home)
    # Start ROS2 MAVROS node
    run_cmd([
        "ros2", "run", "mavros", "mavros_node",
        "--ros-args", "-p", "fcu_url:=udp://:14540@127.0.1:14557",
        "-p", "target_component_id:=1", "-r", "__ns:=/mavros"
    ])
    # Start Foxglove or Plotjuggler
    if choice == 1:
        run_cmd(["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])
        run_cmd(["foxglove-studio", "foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:8765/"])
    elif choice == 2:
        run_cmd(["plotjuggler"])
    # Wait for CTRL-C to exit
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting Data Visualization Setup...")
        terminate_processes()
        sys.exit(0)

# Program entry point
def main():
    parser = argparse.ArgumentParser(description="Choose a mode to run the script.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("-f", "--foxglove", action="store_true", help="Run in file mode")
    group.add_argument("-p", "--plotjuggler", action="store_true", help="Run in process mode")

    args = parser.parse_args()

    # Determine which visualization tool to use based on command line arguments
    if args.foxglove:
        choice = 1
    elif args.plotjuggler:
        choice = 2

    # Initiate the setup with the chosen visualization tool
    run_setup(choice)

if __name__ == "__main__":
    main()

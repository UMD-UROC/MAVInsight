"""Data visualization environment setup script.

This script sets up a data visualization environment using either Foxglove or Plotjuggler.
It starts PX4, Gazebo, QGroundControl, MAVROS, and the chosen visualization tool.
"""
import argparse
import subprocess
import sys
import os
import signal
import time

# Store all subprocess.Popen objects
processes = []

def run_cmd(cmd, cwd=None):
    """Execute shell commands in the background.
    
    Args:
        cmd: Command to execute as a list of strings
        cwd: Working directory for the command (optional)
    """
    # Run command as current user with full environment, in correct directory
    with subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        cwd=cwd,
        env=os.environ.copy()
    ) as process:
        processes.append(process)

def terminate_processes():
    """Handle cleanup on exit by terminating all running processes."""
    print("Terminating all processes...")
    for process in processes:
        try:
            process.send_signal(signal.SIGINT)  # Send SIGINT to gracefully stop the process
            # Wait for the process to terminate
            # NOTE: if process doesn't quit in the alloted time it will be force killed
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()  # Force kill if it doesn't terminate in time
    print("All processes terminated.")

def run_setup(choice):
    """Initialize the data visualization environment.
    
    Args:
        choice: Integer indicating visualization tool (1=Foxglove, 2=Plotjuggler)
    """
    print("Initiating Data Visualization Setup...")
    home = os.path.expanduser("~")
    # Start PX4 and Gazebo
    run_cmd(["make", "px4_sitl", "gz_x500"], cwd=os.path.join(home, "PX4-Autopilot"))
    print("PX4 and Gazebo started successfully.")

    # Start QGroundControl
    run_cmd(["./QGroundControl-x86_64.AppImage"], cwd=home)
    print("QGroundControl started successfully.")

    # Start ROS2 MAVROS node
    run_cmd([
        "ros2", "run", "mavros", "mavros_node",
        "--ros-args", "-p", "fcu_url:=udp://:14540@127.0.1:14557",
        "-p", "target_component_id:=1", "-r", "__ns:=/mavros"
    ])
    print("MAVROS node started successfully.")

    # Start Foxglove or Plotjuggler
    if choice == 1:
        # Start Foxglove Bridge
        run_cmd(["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])
        print("Foxglove Bridge started successfully.")

        # Open Foxglove Studio with the WebSocket URL
        run_cmd([
            "foxglove-studio",
            "foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:8765/"
        ])
        print("Foxglove Studio opened successfully.")
    elif choice == 2:
        # Start Plotjuggler
        run_cmd(["plotjuggler"])
        print("Plotjuggler started successfully.")

    # Wait for CTRL-C to exit
    print("Data Visualization Setup Initiated. Press CTRL-C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting Data Visualization Setup...")
        terminate_processes()
        sys.exit(0)

def main():
    """Program entry point that handles command line arguments and starts setup."""
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
    else:
        # This should never happen due to required=True, but added for safety
        parser.error("No visualization tool selected")
        return

    # Initiate the setup with the chosen visualization tool
    run_setup(choice)

if __name__ == "__main__":
    main()

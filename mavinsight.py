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
    processes.append(p)

# Function to handle cleanup on exit
def terminate_processes():
    print("Terminating all processes...")
    for process in processes:
        try:
            process.send_signal(signal.SIGINT)  # Send SIGINT to gracefully stop the process
            process.wait(timeout=10)  # Wait for the process to terminate NOTE: if process doesn't quit in the alloted time it will be force killed
        except subprocess.TimeoutExpired:
            process.kill()  # Force kill if it doesn't terminate in time
    print("All processes terminated.")

# Setup function to initialize the data visualization environment
def run_setup(choice, nosim, noros):
    print("Initiating Data Visualization Setup...")
    home = os.path.expanduser("~")

    # If no_sim is false run these commands
    if not nosim:
        # Start PX4 and Gazebo
        run_cmd(["make", "px4_sitl", "gz_x500_gimbal"], cwd=os.path.join(home, "PX4-Autopilot"))
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
        # Start UROC 3D Visualize Node
        if not noros:
            run_cmd(["colcon", "build", "--packages-select", "py_uroc"], cwd=os.path.expanduser("~/ros2_ws"))
            run_cmd(["bash", "-c", "source install/local_setup.bash && ros2 launch umd_uroc_data_visualizer visualize.py"], cwd=os.path.expanduser("~/ros2_ws"))
            print("UROC ROS2 python package started successfully.")

        # Start Foxglove Bridge
        run_cmd(["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])
        print("Foxglove Bridge started successfully.")

        # Open Foxglove Studio with the WebSocket URL
        run_cmd(["foxglove-studio", "foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:8765/"])
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

# Program entry point
def main():
    parser = argparse.ArgumentParser(description="Choose a mode to run the script.")

    # Create mutually exclusive group only for visualization tools
    vis_group = parser.add_mutually_exclusive_group(required=True)
    vis_group.add_argument("-f", "--foxglove", action="store_true", help="Run with Foxglove")
    vis_group.add_argument("-p", "--plotjuggler", action="store_true", help="Run with Plotjuggler")

    # Add nosimulation as a separate optional argument
    parser.add_argument("-ns", "--nosimulation", action="store_true", help="Run without simulation (no PX4 or Gazebo)")
    parser.add_argument("-nr", "--noros", action="store_true", help="Run without umd_uroc_data_visualizer")

    args = parser.parse_args()

    # Determine which visualization tool to use based on command line arguments
    if args.foxglove:
        choice = 1
    elif args.plotjuggler:
        choice = 2
    else:
        print("No valid option selected. Please choose either --foxglove or --plotjuggler.")
        sys.exit(1)

    nosim = args.nosimulation
    noros = args.noros

    # Initiate the setup with the chosen visualization tool
    run_setup(choice, nosim, noros)

if __name__ == "__main__":
    main()

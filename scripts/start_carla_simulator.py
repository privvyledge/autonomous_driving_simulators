#!/usr/bin/python3
"""
Uses subprocess to start the carla simulator.
Steps:
    Takes optional arguments, e.g from ROS launch context or the command line
    Checks if another carla simulator instance is running and shuts that down.
    Opens a new instance of the carla simulator with the args.
    Cleanly closes the simulator on exception.

Todo:
    * maybe try threading or using rclpy.spin() to keep running indefinitely
        because currently the node exits as soon the simulator starts
"""
import os
import time
import signal
import subprocess
from argparse import ArgumentParser


def main(*args):
    # (optional) check for running carla processes
    # running_carla_processes = subprocess.run(["pgrep", "-f", "CarlaUE4"], capture_output=True, text=True)

    running_carla_processes = subprocess.Popen(["pgrep", "-f", "CarlaUE4"], stdout=subprocess.PIPE, text=True)
    process_pids, error = running_carla_processes.communicate()
    if len(process_pids.strip()) > 0:
        print("Carla simulator is running.")

    # first kill the process if it's already running
    signal_type = {
        "SIGINT": signal.SIGINT,
        "SIGTERM": signal.SIGTERM,
        "SIGKILL": signal.SIGKILL}
    kill_signal = f"-{signal_type[args[0].signal]}"
    # kill_carla_process = subprocess.run(f"kill {kill_signal} $(pgrep -f CarlaUE4)",
    #                                shell=True, stdout=subprocess.PIPE, text=True)

    kill_cmd = ["kill", kill_signal]
    kill_cmd.extend(process_pids.split())
    kill_carla_process = subprocess.Popen(kill_cmd, text=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)
    kill_carla_process.communicate()

    # launch a new carla process
    try:
        # start_carla_process = subprocess.run(args[1], capture_output=True)
        start_carla_process = subprocess.Popen(args[0].carla_simulator_run_command.split(),
                                               # stdout=subprocess.DEVNULL,
                                               # stderr=subprocess.DEVNULL,
                                               text=True)
        start_carla_process.communicate()
        #start_carla_process.wait()
    except KeyboardInterrupt:
        start_carla_process.send_signal(signal_type[args[0].signal])
        start_carla_process.terminate()
        # start_carla_process.kill()
        start_carla_process.wait()


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument("--signal", type=str, default="SIGTERM", required=False,
                        choices=["SIGTERM", "SIGINT", "SIGKILL"])
    parser.add_argument("--carla_simulator_run_command", type=str,
                        default="/home/carla/carla_simulator/CarlaUE4.sh -nosound -prefernvidia", required=False)
    arguments, unknown = parser.parse_known_args()  # parser.parse_args()
    main(arguments)

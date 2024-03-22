#!/usr/bin/env python3
import sys
import argparse
import subprocess
import shlex

from pyfiglet import Figlet

def execute_gstreamer_pipeline(device_index, ip_address, port):
    """Executes the GStreamer pipeline with the provided arguments,
    ensuring device index conversion for compatibility with v4l2src.

    Args:
        device_index (int): The index of the video device (e.g., 0 for /dev/video0).
        ip_address (str): The IP address of the destination.
        port (int): The port number for UDP communication.

    Raises:
        RuntimeError: If the GStreamer pipeline execution fails.
    """

    if device_index < 0:
        raise ValueError("Device index cannot be negative.")

    device_path = f"/dev/video{device_index}"  # Convert index to device path

    gstreamer_command = f"gst-launch-1.0 v4l2src device={device_path} ! videoconvert ! videoscale ! video/x-raw,width=640,height=480 ! jpegenc ! rtpjpegpay ! udpsink host={ip_address} port={port}"

    print("Executing GStreamer command:", gstreamer_command)

    process = subprocess.Popen(shlex.split(gstreamer_command), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()

    if process.returncode != 0:
        raise RuntimeError(f"GStreamer pipeline execution failed: {stderr.decode()}")

    print("GStreamer pipeline started successfully!")

def main():
    """Parses command-line arguments and executes the GStreamer pipeline with security awareness."""

    parser = argparse.ArgumentParser(description="Run a GStreamer pipeline to stream video.")
    parser.add_argument("-ip", "--ip-address", required=True, help="IP address of the destination.")
    parser.add_argument("-port", "--port", type=int, required=True, help="Port number for UDP communication.")
    parser.add_argument("-device", "--device", type=int, default=0, help="Index of the video device (e.g., 0 for /dev/video0).")

    args = parser.parse_args()

    try:
        # Security precaution: Ensure port is within a valid range
        if args.port < 1024 or args.port > 65535:
            raise ValueError("Port number must be between 1024 and 65535.")

        execute_gstreamer_pipeline(args.device, args.ip_address, args.port)
    except (RuntimeError, ValueError) as e:
        print("Error starting GStreamer pipeline:", e)

if __name__ == "__main__":
    f = Figlet(font='slant')
    print(f.renderText('Sender Side')+"   -------by Team Horizon\n\n\n")
    main()

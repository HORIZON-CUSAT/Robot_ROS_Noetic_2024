#!/usr/bin/env python3
import argparse
import subprocess
from pyfiglet import Figlet

def main():
    """Parses the port argument and executes the GStreamer command."""

    parser = argparse.ArgumentParser(description="Client script")
    parser.add_argument("-port", type=int, required=True, help="Port number for communication")
    args = parser.parse_args()

    port = args.port

    # Build the GStreamer command with the parsed port
    command = f"gst-launch-1.0 -e -v udpsrc port={port} ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink"

    try:
        # Execute the command in a subshell
        subprocess.call(command, shell=True)
    except subprocess.CalledProcessError as error:
        print(f"Error executing GStreamer command: {error}")


if __name__ == "__main__":
    f = Figlet(font='slant')
    print(f.renderText('Receiver')+"   -------by Team Horizon\n\n\n")
    
    main()

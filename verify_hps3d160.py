#!/usr/bin/env python3
"""
Verify serial communication with the HPS-3D160-U lidar over its USB CDC-ACM
virtual COM port (e.g. /dev/ttyACM0).

The sensor does not stream data unsolicited -- it uses a request/response
protocol, so simply reading the port will produce nothing. This script sends
the sensor's documented "Read sensor device address" command (a harmless,
read-only query) and prints whatever raw bytes come back.

No third-party dependencies (no pyserial) -- uses termios/os directly so it
runs on a bare Python 3 install.
"""
import os
import select
import sys
import termios
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
READ_TIMEOUT_S = 2.0

# "Read sensor device address" command, byte-for-byte from the HPS-3D160
# datasheet (header, length, command byte, broadcast address, fixed param,
# then the datasheet's precomputed CRC16 for this exact frame).
CMD_READ_DEVICE_ADDRESS = bytes.fromhex("F50A05BAFF021FD6")


def open_raw(path: str) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    # Put the line into raw mode: no echo, no line buffering, no signal
    # chars, 8N1 -- CDC-ACM ignores baud rate, but termios still wants one.
    termios.tcflush(fd, termios.TCIOFLUSH)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
    iflag = 0
    oflag = 0
    lflag = 0
    cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
    ispeed = ospeed = termios.B115200
    attrs = [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def read_available(fd: int, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    chunks = []
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        ready, _, _ = select.select([fd], [], [], max(remaining, 0))
        if not ready:
            break
        chunk = os.read(fd, 4096)
        if not chunk:
            break
        chunks.append(chunk)
    return b"".join(chunks)


def main() -> int:
    print(f"Opening {PORT} ...")
    try:
        fd = open_raw(PORT)
    except OSError as exc:
        print(f"Failed to open {PORT}: {exc}")
        return 1

    try:
        print(f"Sending 'read device address' command: {CMD_READ_DEVICE_ADDRESS.hex(' ')}")
        os.write(fd, CMD_READ_DEVICE_ADDRESS)

        response = read_available(fd, READ_TIMEOUT_S)
        if not response:
            print("No response received within timeout. "
                  "Check power/cabling; the sensor may not be responding.")
            return 1

        print(f"Received {len(response)} byte(s): {response.hex(' ')}")

        if response[0:2] == b"\xf5\x5f":
            print("Header matches expected response header (F5 5F) -- "
                  "the lidar is responding correctly.")
        else:
            print("Unexpected header -- got a response, but it doesn't "
                  "match the documented frame format.")
        return 0
    finally:
        os.close(fd)


if __name__ == "__main__":
    raise SystemExit(main())

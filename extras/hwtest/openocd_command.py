"""Send explicit commands to an already-running OpenOCD TCL server."""
import argparse
import socket

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("commands", nargs="+")
args = parser.parse_args()
with socket.create_connection(("127.0.0.1", 6666), timeout=3) as debug:
    debug.settimeout(15)
    for command in args.commands:
        debug.sendall(command.encode("ascii") + b"\x1a")
        response = bytearray()
        while not response.endswith(b"\x1a"):
            part = debug.recv(4096)
            if not part:
                raise RuntimeError("OpenOCD disconnected")
            response.extend(part)
        print(command + ": " + response[:-1].decode("ascii", errors="replace"))

# Remote Control - As The Controller Device
import json, rpc, serial, serial.tools.list_ports, struct, sys

# color detection remote call for pink ballon
def exe_color_detection(interface):
    result = interface.call("color_detection")
    if result is not None and len(result):
        res = struct.unpack("<HHHH", result)
        print("Largest Color Detected: {} {} {} {}".format(res[0], res[1], res[2], res[3]))

if __name__ == "__main__":
    interface = rpc.rpc_network_master(slave_ip="192.168.0.67", my_ip="", port=0x1DBA)
    while(True):
        sys.stdout.flush()
        exe_color_detection(interface)
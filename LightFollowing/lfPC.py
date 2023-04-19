# Remote Control - As The Controller Device
import json, rpc, serial, serial.tools.list_ports, struct, sys

# color detection remote call for pink ballon
def exe_light_following(interface):
    result = interface.call("light_following")
    if result is not None and len(result):
        res = struct.unpack("<HH", result)
        print("r-l & intensity: {}".format(res[0], res[1]))

if __name__ == "__main__":
    interface = rpc.rpc_network_master(slave_ip="192.168.0.56", my_ip="", port=0x1DBA)
    while(True):
        sys.stdout.flush()
        exe_light_following(interface)
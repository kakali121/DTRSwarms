# color detection remote call
def exe_color_detection(interface):
    result = interface.call(“color_detection”)
    if result is not None and len(result):
        res = struct.unpack(“<H”, result)
        print(“Largest Color Detected: {}“.format(res[0]))

if __name__ == “__main__“:
    interface = rpc.rpc_network_master(slave_ip=“192.168.0.56”, my_ip=“”, 
port=0x1DBA)
    counter = 0
    while(True):
        counter += 1
        print(counter)
        sys.stdout.flush()
        exe_color_detection(interface)

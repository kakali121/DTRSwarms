import math
import sensor, image, time, pyb, omv
import image, network, rpc, struct, tf
from pyb import UART

omv.disable_fb(True)

# Set up network interface
def setup_network(ssid, key):
	network_if = network.WLAN(network.STA_IF)
	network_if.active(True)
	network_if.connect(ssid, key)
	print(network_if.ifconfig)
	interface = rpc.rpc_network_slave(network_if)
	return interface

def init_sensor(pixformat=sensor.RGB565, framesize=sensor.HQVGA, windowsize=None,
                gain=18, autoexposure=False, exposure=10000, autowhitebal=False,
                contrast=0, saturation=0):
    sensor.reset()
    sensor.set_pixformat(pixformat)
    sensor.set_framesize(framesize)
    if windowsize:
        sensor.set_windowing(windowsize)
    sensor.skip_frames(time = 1000)
    sensor.set_auto_gain(False, gain_db=gain)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=exposure)
    sensor.set_contrast(contrast)
    sensor.set_saturation(saturation)
    sensor.skip_frames(time = 1000)

    sensor.__write_reg(0x0E, 0b00000000)
    sensor.__write_reg(0x3E, 0b00000000)
    #sensor.__write_reg(0x01, 0b11000000)
    #sensor.__write_reg(0x02, 0b01111000)
    #sensor.__write_reg(0x03, 0b01110100)
    sensor.__write_reg(0x2D, 0b00000000)
    sensor.__write_reg(0x2E, 0b00000000)
    sensor.__write_reg(0x35, 0b10000000)
    sensor.__write_reg(0x36, 0b10000000)
    sensor.__write_reg(0x37, 0b10000000)
    sensor.__write_reg(0x38, 0b10000000)
    sensor.__write_reg(0x39, 0b10000000)
    sensor.__write_reg(0x3A, 0b10000000)
    sensor.__write_reg(0x3B, 0b10000000)
    sensor.__write_reg(0x3C, 0b10000000)
    sensor.skip_frames(time = 1000)

def light_following(data):
    n_cols, n_rows = 20, 40
    img = sensor.snapshot()
    l = 0
    r = 0
    n_pixels = 0
    for j in range(n_rows):
        for i in range(n_cols):
            l += img.get_pixel(i, j)
            r += img.get_pixel(240 - n_cols + i , j)
            n_pixels += 1
    l /= 255 * n_pixels
    r /= 255 * n_pixels
    w = r - l
    m = (r + l) / 2
    msg = "w=" + str(w) + " m=" + str(m)
    print(msg)
    return struct.pack("<HH", w, m)

if __name__ == "__main__":
    red_led = pyb.LED(1)
    green_led = pyb.LED(2)
    blue_led = pyb.LED(3)

    blue_led.on()
    green_led.on()
    time.sleep_ms(500)
    blue_led.off()
    green_led.off()
    time.sleep_ms(500)

    SSID='AIRLab-BigLab' # Network SSID
    KEY='Airlabrocks2022'  # Network key
    interface = setup_network(SSID, KEY)
    
    clock = time.clock()
    init_sensor(pixformat=sensor.GRAYSCALE, framesize=sensor.HQVGA, #240x160
                gain=-50, autoexposure=False, autowhitebal=False, contrast=-3, saturation=0)

    while(True):
        interface.register_callback(light_following)
        interface.loop()